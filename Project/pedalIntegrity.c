#include "stm32f4xx.h"
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "CAN.h"
#include "ION_CAN.h"
#include "pedalIntegrity.h"
#include "GPIO.h"
#include "ADC.h"

#define	PEDAL_STATE_IMPLAUSIBLE	0
#define	PEDAL_STATE_OK	1


bool calibrating = false;
uint16_t calibrationLow [N_SENSORS];
uint16_t calibrationHigh[N_SENSORS-1];
uint16_t calibratedPedalsMin, calibratedTorqueMax, calibratedBrakeMax, calibratedSteeringLeft, calibratedSteeringRight, calibratedSteeringCenter = 0;

uint32_t flashStartAddress = 0x080E0000; //starting from 896KB, the beginning of last sector

void saveCalibration(void);
void sendCalibration(void);

uint8_t processPedalPair(uint8_t pair, uint16_t sensorMin, uint16_t sensor, uint16_t sensorMax, uint16_t invertedMin, uint16_t inverted, uint16_t invertedMax);

uint16_t pedalValues[3];

void InitPedalIntegrity(void)
{
	uint8_t i;
	uint16_t magic = *(int16_t *)(flashStartAddress);
		
	if(magic == 0xEF21)
	{
		// Load saved calibration
		for(i=0; i<N_SENSORS-1; i++){
			calibrationLow[i]  = *(int16_t *)(flashStartAddress + (1 + i*2) * 32);
			calibrationHigh[i] = *(int16_t *)(flashStartAddress + (1 + i*2 + 1) * 32);
		}
		
		sendCalibration();
	}
	else {
		// Start new calibration
		for(i=0; i<3; i++){
			calibrationLow[i]  = 0xFFF;
			calibrationHigh[i] = 0x000;
		}
		
		calibrating = true;		
	}	
}

void startCalibration(void)
{
	calibrating = true;
}

void saveCalibration(void){
	FLASH_Unlock();

	/* Clear All pending flags */
	FLASH_ClearFlag( FLASH_FLAG_EOP |  FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);

	FLASH_ProgramHalfWord(flashStartAddress, 0xEF21);
	
	uint8_t i;
	for(i=0; i<7; i++){
		FLASH_ProgramHalfWord(flashStartAddress + (1 + i*2) * 32, calibrationLow[i]);
		FLASH_ProgramHalfWord(flashStartAddress + (1 + i*2 + 1) * 32, calibrationHigh[i]);
	}

	FLASH_Lock();

	sendCalibration();
}

void sendCalibration(void){
	
	uint8_t data[8];
	for(uint8_t i = 0; i<2; i++){
		data[0 + i*4] = (uint8_t)(calibrationLow[1 + i] >> 8);
		data[1 + i*4] = (uint8_t)(calibrationLow[1 + i] & 0xFF);
		
		data[2 + i*4] = (uint8_t)(calibrationHigh[1 + i] >> 8);
		data[3 + i*4] = (uint8_t)(calibrationHigh[1 + i] & 0xFF);
	}
	
	CANTx(0x0FF, 8, data);
	
}

uint32_t calibrationHits = 1,
		 calibrationSamples = 1;

void processPedals(uint16_t rawSensorValues[7]){
	
	/*
	rawSensorValues:
	0	Styresnekke
	1	Throttle
	2	
	*/
	
	uint8_t i;
	
	if(calibrating){
		
		for(i=0; i<7; i++){
			uint16_t val = rawSensorValues[i];
		
			calibrationSamples++;
			
			if(val > calibrationHigh[i]){
				calibrationHigh[i] = val;
				calibrationHits++;
			}
			if(val < calibrationLow[i]){
				calibrationLow[i] = val;
				calibrationHits++;
			}
		}
		
		LED_SetState(LED_GREEN, ENABLE);
		
		if(calibrationSamples / calibrationHits > 25){
			calibrating = false;
			LED_SetState(LED_GREEN, DISABLE);
			
			LED_SetState(LED_BLUE, ENABLE);
			saveCalibration();
			LED_SetState(LED_BLUE, DISABLE);
		}
		
		return;
	}
	
	// Check internal sensors for implausibility
	for(i=0; i<3; i+=2){ // TODO: Integrity check all pedals
		
		uint8_t sensorIdx   = 0 + i*2 + 1;
		uint8_t invertedIdx = 1 + i*2 + 1;
		
		if(processPedalPair(i, calibrationLow[sensorIdx], rawSensorValues[sensorIdx], calibrationHigh[sensorIdx],
							   calibrationLow[invertedIdx], rawSensorValues[invertedIdx], calibrationHigh[invertedIdx]) != PEDAL_STATE_OK){
			reportPedalImplausability(i);
			//break;
		}
	}
	
	// Send pedal-values to CANBus
	uint16_t throttle1 = pedalValues[0];
	//uint16_t throttle2 = pedalValues[1];
	uint16_t brake 	   = pedalValues[2];
	
	uint8_t data[4];
	data[0] = throttle1 >> 8;
	data[1] = throttle1 & 0xFF;
	data[2] = brake >> 8;
	data[3] = brake & 0xFF;
	CANTx(0x100, 4, data);		
}

uint8_t processPedalPair(uint8_t pair, uint16_t sensorMin, uint16_t sensor, uint16_t sensorMax, uint16_t invertedMin, uint16_t inverted, uint16_t invertedMax){
	
	uint16_t range 			= sensorMax - sensorMin;
	uint16_t rangeInverted 	= invertedMax - invertedMin;
	
	// Normalize
	float a = (float)(sensor - sensorMin) / (float)range;
	float b = (float)((invertedMax - invertedMin) - (inverted - invertedMin)) / (float)rangeInverted;
	
	uint16_t aADC = (uint16_t)(a * 0xFFF);
	uint16_t bADC = (uint16_t)(b * 0xFFF);
	
	float diff = fabs(a - b);
	
	if(diff > 0.1f || a < -0.1f || a > 1.1f || b < -0.1f || b > 1.1f){ // 10% differanse
		pedalValues[pair] = 0;
		return PEDAL_STATE_IMPLAUSIBLE;
	}
	
	pedalValues[pair] = (uint16_t)(0xFFF * a);
	
	return PEDAL_STATE_OK;
}

void reportPedalImplausability(uint8_t pedalIdx){
	
	LED_SetState(LED_RED, ENABLE);
	
	uint8_t data[1];
	data[0] = pedalIdx;
	CANTx(CAN_ERR_PEDAL_IMPLAUSIBILITY, 1, data);
}

/* MORTENS KODER */
void calibration(uint16_t rawSensorValues[N_SENSORS-1], uint8_t calibrate)
{
	switch (calibrate){


		case 1 :
			calibrationLow[0] = rawSensorValues[0]; // MIN TORQUE 1 && 2
			calibrationLow[1] = rawSensorValues[1]; 
			calibrationLow[2] = rawSensorValues[2];	// MIN BRAKE 1 && 2
			calibrationLow[3] = rawSensorValues[3];
			#ifdef USE_ALL_TORQUE_ENCODERS
			calibrationLow[6] = rawSensorValues[5]; // MIN TORQUE 3 && 4
			calibrationLow[7] = rawSensorValues[6];
			#endif
			calibratedPedalsMin = 1;
			CANTx(CAN_MSG_PEDALS_MIN_CALIBRATION_COMPLETE, 1, 0);

		case 2 :
			calibrationHigh[0] = rawSensorValues[0];// MAX TORQUE 1 && 2
			calibrationHigh[1] = rawSensorValues[1];
			#ifdef USE_ALL_TORQUE_ENCODERS
			calibrationHigh[5] = rawSensorValues[5]; // MAX TORQUE 3 && 4
			calibrationHigh[6] = rawSensorValues[6];
			#endif
			calibratedTorqueMax = 1;
			CANTx(CAN_MSG_PEDALS_MAX_TORQUE_CAlLIBRATION_COMPLETE, 1, 0);
		
		case 3 :
			calibrationHigh[2] = rawSensorValues[2]; // MAX BRAKE 1 & 2
			calibrationHigh[3] = rawSensorValues[3];
			calibratedBrakeMax = 1;
			CANTx(CAN_MSG_PEDALS_MAX_BRAKE_CAlLIBRATION_COMPLETE, 1, 0);
		
		case 4 :
			calibrationLow[4] = rawSensorValues[4];// Left steering value
			calibratedSteeringLeft = 1;
			CANTx(CAN_MSG_PEDALS_LEFT_STEERING_CALIBRATION_COMPLETE, 1, 0);
		
		case 5 :
			calibrationLow[5] = rawSensorValues[4]; // Center steering value
			calibratedSteeringCenter = 1;
			CANTx(CAN_MSG_PEDALS_CENTER_STEERING_CALIBRATION_COMPLETE, 1, 0);

		case 6 :
			calibrationHigh[4] = rawSensorValues[4];// Right steering value
			calibratedSteeringRight = 1;
			CANTx(CAN_MSG_PEDALS_RIGHT_STEERING_CALIBRATION_COMPLETE, 1, 0);

	}

	if (calibratedSteeringRight && calibratedSteeringCenter && calibratedSteeringLeft 
			&& calibratedBrakeMax && calibratedTorqueMax && calibratedPedalsMin)
	{
		CANTx(CAN_MSG_PEDALS_AND_STEERING_CALIBRATION_COMPLETE, 1, 0);
		
		//save calibration
	}

}

void processEncoders(uint16_t rawSensorValues[N_SENSORS-1])
{
	//TODO: FINISH PROCESS OF ENCODERS WITH 15-bit value
	
	// Assuming all if's and but's are passed:
	
	uint8_t data[6];
	data[0] = (rawSensorValues[0]-calibrationLow[0])>>8;
	data[1] = (rawSensorValues[0]-calibrationLow[0]);	
	data[2] = (rawSensorValues[2]-calibrationLow[2])>>8;
	data[3] = (rawSensorValues[2]-calibrationLow[2]);	
	if (rawSensorValues[4] > calibrationLow[5]) // Steering right
	{		
		data[4] = 1; 
		data[5] = (int)(37*(rawSensorValues[4]-calibrationLow[5])/(calibrationHigh[4]-calibrationLow[5])+0.5);
	}
	else //steering left
	{
		data[4] = 0;
		//calculating turn in degrees where 37 is the highest turn radius.
		data[5] = (int)(37*(calibrationLow[5]-rawSensorValues[4])/(calibrationLow[5]-calibrationLow[4])+0.5);
	}
	
	
}
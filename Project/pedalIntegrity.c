#include "stm32f4xx.h"
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "CAN.h"
#include "ION_CAN.h"
#include "pedalIntegrity.h"
#include "GPIO.h"
#include "main.h"

#define	PEDAL_STATE_OK	0
#define	PEDAL_STATE_RANGE_UNDER	1
#define	PEDAL_STATE_IMPLAUSIBLE	2
#define	PEDAL_STATE_RANGE_OVER	3


uint16_t calibrating = 0;

uint16_t pedalCalibrationLow [7];
uint16_t pedalCalibrationHigh[7];
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
		for(i=0; i<7; i++){
			pedalCalibrationLow[i]  = *(int16_t *)(flashStartAddress + (1 + i*2) * 32);
			pedalCalibrationHigh[i] = *(int16_t *)(flashStartAddress + (1 + i*2 + 1) * 32);
		}
		
		//sendCalibration();
	}
	else {
		// Needs calibration
		while(calibrating == 0){
			delay(0xFFF);
			LED_SetState(LED_RED, ENABLE);
			delay(0xFFF);
			LED_SetState(LED_RED, DISABLE);

		}
	}	
}

void calibrateSensors(uint16_t sensors){
	calibrating = sensors;
}

void saveCalibration(void){
	FLASH_Unlock();

	/* Clear All pending flags */
	FLASH_ClearFlag( FLASH_FLAG_EOP |  FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);

	FLASH_ProgramHalfWord(flashStartAddress, 0xEF21);
	
	uint8_t i;
	for(i=0; i<7; i++){
		FLASH_ProgramHalfWord(flashStartAddress + (1 + i*2) * 32, pedalCalibrationLow[i]);
		FLASH_ProgramHalfWord(flashStartAddress + (1 + i*2 + 1) * 32, pedalCalibrationHigh[i]);
	}

	FLASH_Lock();

	sendCalibration();
}

// Send the calibration-data over CAN-bus (for debugging)
void sendCalibration(void){
	
	uint8_t data[8];
	for(uint8_t i = 0; i<2; i++){
		data[0 + i*4] = (uint8_t)(pedalCalibrationLow[1 + i] >> 8);
		data[1 + i*4] = (uint8_t)(pedalCalibrationLow[1 + i] & 0xFF);
		
		data[2 + i*4] = (uint8_t)(pedalCalibrationHigh[1 + i] >> 8);
		data[3 + i*4] = (uint8_t)(pedalCalibrationHigh[1 + i] & 0xFF);
	}
	
	CANTx(0x0FF, 8, data);
	
}

uint8_t consecutiveErrors[3];

uint32_t calibrationSamples = 0;

void processPedals(uint16_t rawSensorValues[7]){
	
	/*
	rawSensorValues:
	0	 Steering
	1	 Torque
	2	~Torque
	3	 Torque
	4 	~Torque
	5	 Brake
	6	~Brake
	*/
	
	uint8_t i;
	
	if(calibrating != 0){
		
		for(i=0; i<7; i++){
			if(((calibrating >> i) & 0x1) == 1){
			
				uint16_t val = rawSensorValues[i];
				
				bool calHigh = ((calibrating & SENSOR_CALIBRATE_HIGH) != 0);
				bool calLow  = ((calibrating & SENSOR_CALIBRATE_LOW)  != 0);
				
				if((SENSOR_INVERTED >> i) & 0x1){
					calHigh = !calHigh;
					calLow  = !calLow;
				}
				
				if(calHigh && (val > pedalCalibrationHigh[i] || calibrationSamples == 0)){
					pedalCalibrationHigh[i] = val;
				}
				
				if(calLow && (val < pedalCalibrationLow[i] || calibrationSamples == 0)){
					pedalCalibrationLow[i] = val;
				}
			}
		}
		
		calibrationSamples++;
		
		LED_SetState(LED_GREEN, ENABLE);
		
		// End calibration
		if(calibrationSamples > 25){
			
			// TODO: Verify calibration
			uint8_t calibrationResult = 0;
			if(calibrating & SENSOR_CALIBRATE_LOW){
				
			}
			else if(calibrating & SENSOR_CALIBRATE_HIGH){
				
			}
			
			// Send calibration completed
			uint8_t data[2];
			data[0] = (calibrating & SENSOR_CALIBRATE_STEERING ? 1<<2 : 0) | (calibrating & SENSOR_CALIBRATE_BRAKE ? 1<<1 : 0) | (calibrating & SENSOR_CALIBRATE_TORQUE ? 1<<0 : 0);
			data[1] = calibrationResult;
			CANTx(CAN_MSG_PEDALS_CALIBRATION_COMPLETE, 2, data);
			
			// Save calibration
			LED_SetState(LED_BLUE, ENABLE);
			saveCalibration();
			LED_SetState(LED_BLUE, DISABLE);
			
			// End calibration
			calibrating = 0;
			calibrationSamples = 0;
			LED_SetState(LED_GREEN, DISABLE);
		}
		
		return;
	}
	
	// Check internal sensors for implausibility
	FunctionalState implausability = DISABLE;
	for(i=0; i<3; i+=2){ // TODO: Integrity check all pedals
		
		uint8_t sensorIdx   = 0 + i*2 + 1;
		uint8_t invertedIdx = 1 + i*2 + 1;
		
		uint8_t pairState = processPedalPair(i, pedalCalibrationLow[sensorIdx], rawSensorValues[sensorIdx], pedalCalibrationHigh[sensorIdx],
							   pedalCalibrationLow[invertedIdx], rawSensorValues[invertedIdx], pedalCalibrationHigh[invertedIdx]);
		
		if(pairState != PEDAL_STATE_OK)
		{	
			if(consecutiveErrors[i] < 50){
				consecutiveErrors[i]++;				
			}
			
			if(consecutiveErrors[i] > 10){
				reportPedalImplausability(i, pairState);
				implausability = ENABLE;
			}
		}
		// Cooldown
		else if(consecutiveErrors[i] > 0){
			consecutiveErrors[i]--;
			reportPedalImplausability(i, pairState); // This will report PEDAL_STATE_OK during cooldown
			implausability = ENABLE;
		}
	}
	
	LED_SetState(LED_RED, implausability);
	
	
	// Send pedal-values to CANBus
	uint16_t torque1 = pedalValues[0];
	//uint16_t torque2 = pedalValues[1];
	uint16_t brake 	   = pedalValues[2];
	
	uint8_t data[4];	
	data[0] = torque1 >> 8;
	data[1] = torque1 & 0xFF;
	data[2] = brake >> 8;
	data[3] = brake & 0xFF;
	CANTx(CAN_MSG_PEDALS, 4, data);
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
	
	if(diff > 0.1f){ // 10% differanse
		return PEDAL_STATE_IMPLAUSIBLE;
	}
	else if(a < -0.1f || b < -0.1f){
		return PEDAL_STATE_RANGE_UNDER;
	}
	else if(a > 1.1f || b > 1.1f){
		return PEDAL_STATE_RANGE_OVER;				
	}
	
	if(a > 1.0f) a = 1.0;
	if(a < 0.0f) a = 0.0;
	
	// 5% deadband
	a = (a - 0.05f) / (1.0f - 0.05f);
	
	pedalValues[pair] = (uint16_t)(0xFFF * a);
	
	return PEDAL_STATE_OK;
}

void reportPedalImplausability(uint8_t pedalIdx, uint8_t pedalState){
	
	pedalValues[pedalIdx] = 0;
	
	uint8_t data[2];
	data[0] = pedalIdx;
	data[1] = pedalState;
	CANTx(CAN_ERR_PEDALS_IMPLAUSIBILITY, 2, data);
}

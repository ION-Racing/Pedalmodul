#include "stm32f4xx.h"
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "CAN.h"
#include "ION_CAN.h"
#include "pedalIntegrity.h"
#include "GPIO.h"

#define	PEDAL_STATE_IMPLAUSIBLE	0
#define	PEDAL_STATE_OK	1


bool calibrating = false;

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
		
	if(magic == 0xEF22)
	{
		// Load saved calibration
		for(i=0; i<7; i++){
			pedalCalibrationLow[i]  = *(int16_t *)(flashStartAddress + (1 + i*2) * 32);
			pedalCalibrationHigh[i] = *(int16_t *)(flashStartAddress + (1 + i*2 + 1) * 32);
		}
		
		sendCalibration();
	}
	else {
		// Start new calibration
		for(i=0; i<3; i++){
			pedalCalibrationLow[i]  = 0xFFF;
			pedalCalibrationHigh[i] = 0x000;
		}
		
		calibrating = true;		
	}	
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
			
			if(val > pedalCalibrationHigh[i]){
				pedalCalibrationHigh[i] = val;
				calibrationHits++;
			}
			if(val < pedalCalibrationLow[i]){
				pedalCalibrationLow[i] = val;
				calibrationHits++;
			}
			
			/*if(i==0){
				uint8_t data[6];
				data[0] = val >> 8;
				data[1] = val & 0xFF;
				data[2] = pedalCalibrationHigh[i] >> 8;
				data[3] = pedalCalibrationHigh[i] & 0xFF;
				data[4] = pedalCalibrationLow[i] >> 8;
				data[5] = pedalCalibrationLow[i] & 0xFF;
				CANTx(0x101, 6, data);
			}*/
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
	for(i=0; i<2; i++){ // TODO: Integrity check all pedals
		
		uint8_t sensorIdx   = 0 + i*2 + 1;
		uint8_t invertedIdx = 1 + i*2 + 1;
		
		if(processPedalPair(i, pedalCalibrationLow[sensorIdx], rawSensorValues[sensorIdx], pedalCalibrationHigh[sensorIdx],
							   pedalCalibrationLow[invertedIdx], rawSensorValues[invertedIdx], pedalCalibrationHigh[invertedIdx]) != PEDAL_STATE_OK){
			reportPedalImplausability(i);
			//break;
		}
		else {
			
			uint16_t throttle1 = pedalValues[0];
			uint16_t throttle2 = pedalValues[1];
			
			uint8_t data[4];
			data[0] = throttle1 >> 8;
			data[1] = throttle1 & 0xFF;
			data[2] = throttle2 >> 8;
			data[3] = throttle2 & 0xFF;
			//CANTx(0x100, 4, data);
			
		}
	}
	
	return;
	
	uint8_t data[8];
	for(i = 0; i<3; i++){
		data[0 + i*2] = (uint8_t)(rawSensorValues[1+i*2] >> 8);
		data[1 + i*2] = (uint8_t)(rawSensorValues[1+i*2] & 0xFF);
		
		//data[i] = (uint8_t)(rawSensorValues[i] >> 7);
	}
	
	CANTx(0x100, 6, data);
		
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
	
	if(pair == 1){
	uint8_t data[8];
	data[0] = aADC >> 8;
	data[1] = aADC & 0xFF;
	data[2] = bADC >> 8;
	data[3] = bADC & 0xFF;
	/*data[4] = adcDiff >> 8;
	data[5] = adcDiff & 0xFF;
	data[6] = range >> 8;
	data[7] = range & 0xFF;
	uint16_t flash = *(int16_t *)(flashStartAddress);
	data[6] = flash >> 8;
	data[7] = flash & 0xFF;*/
	CANTx(0x100, 4, data);
	}
	
	if(diff > 0.1f || a < -0.1f || a > 1.1f || b < -0.1f || b > 1.1f){ // 10% differanse
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

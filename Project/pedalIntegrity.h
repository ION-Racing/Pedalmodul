#include "stm32f4xx.h"

//#define USE_ALL_TORQUE_ENCODERS			// uncomment to use all four torque encoders

#ifdef USE_ALL_TORQUE_ENCODERS
	#define BUFFERSIZE  512
	#define N_SENSORS		8
#else 
	#define BUFFERSIZE  384
	#define N_SENSORS		6
#endif

void InitPedalIntegrity(void);
void processPedals(uint16_t rawSensorValues[6]);
void reportPedalImplausability(uint8_t pedalIdx);
void startCalibration(void);
void calibration(uint16_t rawSensorValues[N_SENSORS-1], uint8_t calibrate);
void processEncoders(uint16_t rawSensorValues[N_SENSORS-1]);

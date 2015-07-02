void InitPedalIntegrity(void);
void processPedals(uint16_t rawSensorValues[6]);
void reportPedalImplausability(uint8_t pedalIdx, uint8_t pedalState);

#define	SENSOR_INVERTED				((1<<2) | (1<<4) | (1<<6))

#define SENSOR_CALIBRATE_HIGH		0x400
#define SENSOR_CALIBRATE_CENTER		0x200
#define SENSOR_CALIBRATE_LOW		0x100

#define	SENSOR_CALIBRATE_STEERING	0x001 // 0
#define	SENSOR_CALIBRATE_TORQUE		0x006 // 1&2
#define	SENSOR_CALIBRATE_BRAKE		0x060 // 5&6

#define	PEDAL_CALIBRATE_TORQUE_MIN	0x106
#define	PEDAL_CALIBRATE_TORQUE_MAX	0x406
#define	PEDAL_CALIBRATE_BRAKE_MIN	0x160
#define	PEDAL_CALIBRATE_BRAKE_MAX	0x460
#define	STEERING_CALIBRATE_MIN		0x101
#define	STEERING_CALIBRATE_CENTER	0x201
#define	STEERING_CALIBRATE_MAX		0x401

#define	CALIBRATION_ERR_LOWER_BOUND	0x01
#define	CALIBRATION_ERR_RANGE		0x02
#define	CALIBRATION_ERR_UPPER_BOUND	0x04

void calibrateSensors(uint16_t sensors);
/*
Sensors bits:
0-6		select sensor
7		don't care
8		low
9		center
10		high
*/

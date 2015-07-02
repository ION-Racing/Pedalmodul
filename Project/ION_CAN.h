// ION CAN Messages v6 - 2015-07-03

#define	CAN_ERR_HV_SHUTDOWN								0x000
#define	CAN_ERR_IMD										0x001
#define	CAN_ERR_BMS										0x002
#define	CAN_ERR_OVERTEMP								0x003
#define	CAN_MSG_PRECHARGE								0x010
#define	CAN_MSG_CURRENT_SENSOR							0x011
#define	CAN_MSG_CURRENT_SENSOR_CALIBRATE				0x012
#define	CAN_MSG_CURRENT SENSOR_CALIBRATION_COMPLETE		0x013
#define	CAN_MSG_HV_SENSOR								0x014
#define	CAN_MSG_LV_SENSOR								0x015
#define	CAN_MSG_POWER									0x016

#define	CAN_MSG_MOTOR_LEFT_RX							0x181
#define	CAN_MSG_MOTOR_RIGHT_RX							0x182
#define	CAN_MSG_MOTOR_LEFT_TX							0x201
#define	CAN_MSG_MOTOR_RIGHT_TX							0x202

#define	CAN_ERR_PEDALS_IMPLAUSIBILITY					0x300
#define	CAN_MSG_PEDALS									0x310
#define	CAN_MSG_STEERING								0x311
#define	CAN_MSG_PEDALS_CALIBRATE						0x320
#define	CAN_MSG_PEDALS_CALIBRATION_COMPLETE				0x321

#define	CAN_MSG_USER_START								0x400
#define	CAN_MSG_USER_STOP								0x401
#define	CAN_MSG_WHEEL_RPM_FRONT							0x450
#define	CAN_MSG_SPEED									0x460

#define	CAN_MSG_BMS_PRODUCT								0x620
#define	CAN_MSG_BMS_REVISION							0x621
#define	CAN_MSG_BMS_FLAGS								0x622
#define	CAN_MSG_BMS_VOLTAGE								0x623
#define	CAN_MSG_BMS_CURRENT								0x624
#define	CAN_MSG_BMS_ENERGY								0x625
#define	CAN_MSG_BMS_CHARGE								0x626
#define	CAN_MSG_BMS_TEMP								0x627
#define	CAN_MSG_BMS_RESISTANCE							0x628

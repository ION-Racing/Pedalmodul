void InitCAN(void);
uint8_t CANTx(uint32_t address, uint8_t length, uint8_t data[8]);


/* Macro list of all the Tx/Rx message ID's for this module
-----------------------------------------------------------
0x0 = unknown ID.
*/


/* Tx list 

Below is some information about the contents of the messages.
Example	: w1.1 = wheelsensor 1 part 1.
				: w1.2 = wheelsensor 1 part 2.
-------------------------------------
ID			D1		D2		D3		D4		D5
0x200		W1.1	w1.2	w2.1	w2.2	-
0x40		-			-			-			-			-
0x41		-			-			-			-			-
0x600		-			-			-			-			-

*/

#define TX_IWD_TIMEOUT	0x600

/*
CAN transmit messages
*/
#define CAN_ECHO 0x30					//CAN echo message
#define ECU_FAILED_ECHO 0x41 	//ECU did not respond	
#define RPIU_FAILED_ECHO 0x42	//RPIU did not respond
#define DU_FAILED_ECHO 0x43		//DU did not respond
#define PU_FAILED_ECHO 0x44		//PU did not respond


#define CAN_ERR_PEDALS_IMPLAUSIBILITY 														0x300
#define CAN_MSG_PEDALS_STEERING																		0x310

#define CAN_MSG_PEDALS_MIN_CALIBRATE															0x320
#define CAN_MSG_PEDALS_MIN_CALIBRATION_COMPLETE										0x321
#define CAN_MSG_PEDALS_MAX_TORQUE_CALIBRATE												0x322
#define CAN_MSG_PEDALS_MAX_TORQUE_CAlLIBRATION_COMPLETE						0x323
#define CAN_MSG_PEDALS_MAX_BRAKE_CALIBRATE												0x324
#define CAN_MSG_PEDALS_MAX_BRAKE_CAlLIBRATION_COMPLETE						0x325
#define CAN_MSG_PEDALS_LEFT_STEERING_CALIBRATE										0x326
#define CAN_MSG_PEDALS_LEFT_STEERING_CALIBRATION_COMPLETE					0x327
#define CAN_MSG_PEDALS_CENTER_STEERING_CALIBRATE									0x328
#define CAN_MSG_PEDALS_CENTER_STEERING_CALIBRATION_COMPLETE				0x329
#define CAN_MSG_PEDALS_RIGHT_STEERING_CALIBRATE										0x330
#define CAN_MSG_PEDALS_RIGHT_STEERING_CALIBRATION_COMPLETE				0x331
#define CAN_MSG_PEDALS_AND_STEERING_CALIBRATION_COMPLETE					0x332	



/*
CAN recive messages
*/
#define ECU_KICKED_CAN 	0x31 		// Engine Control Unit
#define RPIU_KICKED_CAN 0x32 		// RPI Unit
#define DU_KICKED_CAN 	0x33		// Dashboard Unit
#define PU_KICKED_CAN 	0x34		// Pedalbox Unit



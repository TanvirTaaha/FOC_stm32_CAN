#ifndef _MOTOR_H_
#define _MOTOR_H_
#define DEBUG
#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleCan.h>


#define MOTOR_INDEX 2


// first 4 bits from left are for motor index and next 4 bits are for msg type
// 1010 -> mcu to motor
// 0101 -> motor to mcu
#define CAN_ID_FILTER_TX 0b1010
#define CAN_ID_FILTER_RX 0b1111
// #define CAN_DATA_REQ_ID 0b101011
#define CAN_DATA_REQ_ID 1 << 6


void setup_can_comm(void);
void loop_can_comm(void);
void isr_handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData);
void init_CAN(void);
bool sendCanMessage();
void ButtonDown(void);
void isr_handleInterrupt(void);
void setTxSpeedAndPos(float speed, float position);
extern float target_velocity;
extern uint32_t received_can_id;

extern bool new_rx_data;
extern bool new_tx_data;
extern bool is_data_requested;
#endif

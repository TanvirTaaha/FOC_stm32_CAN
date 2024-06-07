#ifndef _MOTOR_H_
#define _MOTOR_H_
#define DEBUG
#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleCan.h>


void setup_can_comm(void);
void loop_can_comm(void);
void isr_handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData);
void init_CAN(void);
bool sendCanMessage();
void ButtonDown(void);
void isr_handleInterrupt(void);
void setTxSpeedAndPos(float speed, float position);
extern float target_velocity;

#endif

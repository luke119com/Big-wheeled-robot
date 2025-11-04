#ifndef CAN_H
#define CAN_H

#include "Arduino.h"
#include "CAN_comm.h"
#include "robot.h"

#define SEND_INTERVAL 1 // 限制发送频率，单位为毫秒
extern float Am_kp;

void CAN_Control();
void startMotor(int motorIndex);
void posInit();
void enableMotor();
#endif
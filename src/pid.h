#ifndef PID_H
#define PID_H

#include "Arduino.h"
#include "robot.h"

/*线性拟合PID结构体*/
struct PIDValues {
  float linear_vel_kp;
  float linear_balance_kp;
  float linear_balance_kd;
  float linear_robot_kp;
};

extern float vel_kp ;
extern float balance_kp ;
extern float balance_kd ;
extern float turn_kp ;

extern float wheel_motor1_target, wheel_motor2_target; // 电机目标值
extern int speed_limit; //轮毂电机速度限制
void wheel_control();
String serialReceiveUserCommand();
PIDValues  interpolatePID(int y_height);
float clampToRange(float value, float minVal, float maxVal);

#endif
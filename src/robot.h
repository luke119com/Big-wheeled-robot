#ifndef ROBOT_H
#define ROBOT_H

#include "Arduino.h"
#include "bipedal_data.h"
#include "ppm.h"
#include "CAN_comm.h"
#include "can.h"
#include "pid.h"

// 逆解控制参数
#define L1 150
#define L2 250
#define L3 250
#define L4 150
#define L5 120
#define L6 0
extern float robot_kp;
extern float roll_kp , roll_kd;
extern float steering ;
extern float remoteBalanceOffset;
extern int Shake_shoulder_vakue;
extern float x1 , x2, Y1 , y2 ;
extern bool serialFootPoseMode;
extern float serialLeftTargetX, serialLeftTargetY;
extern float serialRightTargetX, serialRightTargetY;
extern float leftY , rightY ;
extern float leftX , rightX ;
extern float motorLeftFront, motorLeftRear, motorRightFront, motorRightRear;
// 遥控器控制参数
extern float forwardBackward ;
// 轮足中两个轮毂电机的轮速
extern float motor1_vel, motor2_vel;
// 陀螺仪读取参数
extern float roll, pitch, yaw, init_pitch;
extern float accX, accY, accZ, imuTemp;
extern float gyroX, gyroY, gyroZ;
extern float balance_offset;
extern int ZeparamremoteValue; //腿高变化
extern float roll_EH;



void jump_control();
void inverseKinematics();
void robot_control();
float constrainValue(float value, float minValue, float maxValue);
void mapPPMToRobotControl();
int mapJoystickValuerollzeparam(int inputValue);
float mapJoystickValuevel(int inputValue);
float mapJoystickValueInt(int inputValue);
float mapJoystickValuesteering(int inputValue);
int trot(int inputValue);
void get_origin_pos();
void setSerialFootPoseTargets(float leftTargetX, float leftTargetY, float rightTargetX, float rightTargetY);
void setSerialFootPoseMode(bool enabled);
bool validateSerialFootPoseTargets(float leftTargetX, float leftTargetY, float rightTargetX, float rightTargetY);


#endif

#include "robot.h"

#define MOTOR_RF_OFFSET   -PI/2 // Right Front (右前)3 
#define MOTOR_RR_OFFSET   -1.57f // Right Rear (右后)1
#define MOTOR_LR_OFFSET   -PI/2  // Left Rear (左后)2
#define MOTOR_LF_OFFSET   -1.5f  // Left Front (左前)4
#define POS_OF_RR0DRGEE  motorPos1   //右后0°位置
#define POS_OF_RF0DRGEE  motorPos3    //右前0°位置
#define POS_OF_LR0DRGEE  motorPos2    //左后0°位置
#define POS_OF_LF0DRGEE  motorPos4    //左前0°位置



IKparam IKParam;
// 左腿相关运动学基本参数
motionControlParams LeftMotionControlParams;
// 右腿相关运动学基本参数
motionControlParams RightMotionControlParams;
motorsparam motorsParam;
float leg_balance_kp = 0.8f;   // small assist: mm per degree of pitch
float leg_balance_kd = 0.1f;  // small assist: mm per deg/s of gyroY
float leg_balance_limit = 10.0f;

float roll_kp = 0.0025, roll_kd = -0.007;  //机器人自稳Kp、Kd值

float leftY = 300, rightY = 300;  //机器人Y轴方向腿部高度
float leftX = 60, rightX = 60;    //机器人X轴方向腿部高度
float x1 = leftX, x2 = rightX, Y1 = leftY, y2 = rightY;
bool serialFootPoseMode = false;
float serialLeftTargetX = 60;
float serialLeftTargetY = 165;
float serialRightTargetX = 60;
float serialRightTargetY = 145;
float motorLeftFront, motorLeftRear, motorRightFront, motorRightRear;
int ZeparamremoteValue = 0;
float roll_EH;
float forwardBackward = 0;
float motor1_vel, motor2_vel;
int lastZeparamremoteValue = 0; // 上次输出的 ZeparamremoteValue
int EH = 0;
float Tartget_Roll_angle;
float roll, pitch, yaw, init_pitch; //陀螺仪xyz轴值
float gyroX, gyroY, gyroZ;//陀螺仪xyz轴加速度值
float target_roll = 0.0;
float steering = 0;
float remoteBalanceOffset = 0; // 大往前小往后
uint8_t origin_pos_flag = 1;
MIT devicesState[8];
float robot_kp = 5.0;//3
float jump_vlaue; //起跳高度
float balance_offset = 0;
float motorPos1 = 0;
float motorPos2 = 0;
float motorPos3 = 0;
float motorPos4 = 0;
bool recardPos = false;

// 用于获取电机初始位置函数
static bool legPoseReachable(float x, float y)
{
  float alphaTerm = x * x + y * y + L1 * L1 - L2 * L2;
  float betaTerm = (x - L5) * (x - L5) + L4 * L4 + y * y - L3 * L3;
  float alphaDisc = (2.0f * x * L1) * (2.0f * x * L1) + (2.0f * y * L1) * (2.0f * y * L1) - alphaTerm * alphaTerm;
  float betaDisc = (2.0f * L4 * (x - L5)) * (2.0f * L4 * (x - L5)) + (2.0f * L4 * y) * (2.0f * L4 * y) - betaTerm * betaTerm;
  return alphaDisc >= -1e-3f && betaDisc >= -1e-3f;
}

void setSerialFootPoseTargets(float leftTargetX, float leftTargetY, float rightTargetX, float rightTargetY)
{
  serialLeftTargetX = leftTargetX;
  serialLeftTargetY = leftTargetY;
  serialRightTargetX = rightTargetX;
  serialRightTargetY = rightTargetY;
}

void setSerialFootPoseMode(bool enabled)
{
  bool wasEnabled = serialFootPoseMode;
  serialFootPoseMode = enabled;
  if (enabled)
  {
    forwardBackward = 0;
    steering = 0;
    wheel_motor1_target = 0;
    wheel_motor2_target = 0;
  }
  else if (wasEnabled)
  {
    // Keep the last serial IK pose as the new nominal stance after leaving serial mode.
    leftX = serialLeftTargetX;
    leftY = serialLeftTargetY;
    rightX = serialRightTargetX;
    rightY = serialRightTargetY;
    x1 = leftX;
    Y1 = leftY;
    x2 = rightX;
    y2 = rightY;
    ZeparamremoteValue = 0;
    Shake_shoulder_vakue = 0;
    jump_vlaue = 0;
    forwardBackward = 0;
    steering = 0;
    wheel_motor1_target = 0;
    wheel_motor2_target = 0;
  }
}

bool validateSerialFootPoseTargets(float leftTargetX, float leftTargetY, float rightTargetX, float rightTargetY)
{
  return legPoseReachable(leftTargetX, leftTargetY) && legPoseReachable(rightTargetX, rightTargetY);
}

void get_origin_pos()
{
  if (origin_pos_flag == 1)
  {
    // 左腿关节电机初始位置
    motorsParam.motorleftsita0_origin = -devicesState[0].pos;
    motorsParam.motorleftsita1_origin = -devicesState[1].pos;

    // 右腿关节电机初始位置
    motorsParam.motorrightsita0_origin = -devicesState[3].pos;
    motorsParam.motorrightsita1_origin = -devicesState[2].pos;

    if (motorsParam.motorleftsita0_origin != 0 && motorsParam.motorleftsita1_origin != 0 && motorsParam.motorrightsita0_origin != 0 && motorsParam.motorrightsita1_origin != 0)
      origin_pos_flag = 0;
  }
}

// 将读取的PPM数据进行映射处理 转换为机器人控制参数
void mapPPMToRobotControl()
{
  if(Shake_shoulder == 0)
  {
    // 这里用于控制腿部高度变化控制 采用增量式PID是为了让腿部快速变化时，保持稳定
    float error = (1000 - (filteredPPMValues[0] - 70))/1.5; // 计算误差
    ZeparamremoteValue = -0.30 * error;      // 计算当前的值
    ZeparamremoteValue = constrainValue(ZeparamremoteValue, 0, 150);
    ZeparamremoteValue = ZeparamremoteValue + (ZeparamremoteValue - lastZeparamremoteValue) * 0.12; // 基于误差更新输出（增量控制）
    lastZeparamremoteValue = ZeparamremoteValue;
  }                                                    // 存储当前值作为下一次计算的参考
  // 用于roll轴自稳
  if(EH_rollflag == 1) //自稳开关
  {
    target_roll = target_roll +  roll_kp * (roll - 0);
    target_roll = constrainValue(target_roll, -50, 50);
  }
  else
  {
    target_roll = 0;
  }
  // 用于控制前进后退
  forwardBackward = mapJoystickValuevel(filteredPPMValues[1]);
  // 用于调节重心偏置
  remoteBalanceOffset = mapJoystickValueInt(filteredPPMValues[2]);
  // 用于控制转向
  steering = -0.03 * (-mapJoystickValuesteering(filteredPPMValues[5]) - gyroZ);
  steering = constrainValue(steering, -10, 10);
  //
  roll_EH = sin((target_roll * 3.14) / 180) * (LeftMotionControlParams.robotl) / 2;
  roll_EH = constrainValue(roll_EH, -80, 80);

  Serial.println(forwardBackward);
}

void jump_control()
{
  static unsigned long startMillis = 0; // 用于记录起始时间
  static bool timingStarted = false;    // 标志是否开始计时
  if (jump_flag == 1)   //起跳flag
  {
    if (!timingStarted)  
    {
      // 开始计时
      startMillis = millis();
      timingStarted = true;
    }
    if (millis() - startMillis >= 90)
    { // 判断是否达到100ms
       Am_kp = 0;
       jump_vlaue = 0;
    }
    else
    {
       Am_kp = 8;
       jump_vlaue =180;
    }
  }
  else
  {
    // 重置计时器和标志位
    Am_kp = 0;
    timingStarted = false;
    startMillis = 0;
  }
}


float safe_sqrt(float x) {
    return x >= 0.0f ? sqrtf(x) : 0.0f;
}

float safe_div(float num, float denom) {
    if (fabs(denom) < 1e-6f) denom = (denom < 0 ? -1e-6f : 1e-6f); // 防止分母0
    return num / denom;
}



uint32_t prevTs=0;
// // 运动学逆解函数
void inverseKinematics()
{
  float alpha1, alpha2, beta1, beta2;
  uint32_t current_ts = micros();
  // 右腿逆解运算
  float aRight = 2 * x2 * L1;
  float bRight = 2 * y2 * L1;
  float cRight = x2 * x2 + y2 * y2 + L1 * L1 - L2 * L2;
  float dRight = 2 * L4 * (x2 - L5);
  float eRight = 2 * L4 * y2;
  float fRight = ((x2 - L5) * (x2 - L5) + L4 * L4 + y2 * y2 - L3 * L3);

  float rightAlphaDisc = safe_sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight));
  float rightBetaDisc = safe_sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight));
  IKParam.alphaRight = 2 * atan(safe_div(bRight + rightAlphaDisc, aRight + cRight));
  IKParam.betaRight = 2 * atan(safe_div(eRight - rightBetaDisc, dRight + fRight));

  alpha1 = 2 * atan(safe_div(bRight + rightAlphaDisc, aRight + cRight));
  alpha2 = 2 * atan(safe_div(bRight - rightAlphaDisc, aRight + cRight));
  beta1 = 2 * atan(safe_div(eRight + rightBetaDisc, dRight + fRight));
  beta2 = 2 * atan(safe_div(eRight - rightBetaDisc, dRight + fRight));

  alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
  alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

  if (alpha1 >= PI / 4)
    IKParam.alphaRight = alpha1;
  else
    IKParam.alphaRight = alpha2;
  if (beta1 >= 0 && beta1 <= PI / 4)
    IKParam.betaRight = beta1;
  else
    IKParam.betaRight = beta2;

  // 左腿逆解运算
  float aLeft = 2 * x1 * L1;
  float bLeft = 2 * Y1 * L1;
  float cLeft = x1 * x1 + Y1 * Y1 + L1 * L1 - L2 * L2;

  float dLeft = 2 * L4 * (x1 - L5);
  float eLeft = 2 * L4 * Y1;
  float fLeft = ((x1 - L5) * (x1 - L5) + L4 * L4 + Y1 * Y1 - L3 * L3);

  // alpha的计算
  float leftAlphaDisc = safe_sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft));
  float leftBetaDisc = safe_sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft));
  alpha1 = 2 * atan(safe_div(bLeft + leftAlphaDisc, aLeft + cLeft));
  alpha2 = 2 * atan(safe_div(bLeft - leftAlphaDisc, aLeft + cLeft));
  beta1 = 2 * atan(safe_div(eLeft + leftBetaDisc, dLeft + fLeft));
  beta2 = 2 * atan(safe_div(eLeft - leftBetaDisc, dLeft + fLeft));

  // 角度解算范围限制
  alpha1 = (alpha1 >= 0) ? alpha1 : (alpha1 + 2 * PI);
  alpha2 = (alpha2 >= 0) ? alpha2 : (alpha2 + 2 * PI);

  if (alpha1 >= PI / 4)
    IKParam.alphaLeft = alpha1;
  else
    IKParam.alphaLeft = alpha2;
  if (beta1 >= 0 && beta1 <= PI / 4)
    IKParam.betaLeft = beta1;
  else
    IKParam.betaLeft = beta2;

  // // alphaRight
  // motorRightRear = (2.79 + 1.57 * 8) - (IKParam.betaRight * 8);    // 1
  // motorRightFront = (4.05 + 1.57 * 8) - (IKParam.alphaRight * 8); // 3
  // // alphaLeft
  // motorLeftRear = (2.68 + 1.57 * 8) - (IKParam.betaLeft * 8);     // 2
  // motorLeftFront = (4.5 + 1.57 * 8) - (IKParam.alphaLeft * 8);   // 4

  motorRightRear = (IKParam.alphaRight-PI/2);    // 1
  motorRightFront = ((MOTOR_RF_OFFSET)+(IKParam.betaRight)); // 3

  // alphaLeft
  motorLeftRear = MOTOR_LR_OFFSET+(IKParam.betaLeft );     // 2
  motorLeftFront =  (IKParam.alphaLeft-PI/2);   // 4

}


// 轮足机器人控制部分程序
// int aaa = 0;
int Shake_shoulder_vakue = 0;
void robot_control()
{
    if (serialFootPoseMode)
    {
      x1 = serialLeftTargetX;
      Y1 = serialLeftTargetY;
      x2 = serialRightTargetX;
      y2 = serialRightTargetY;
      forwardBackward = 0;
      steering = 0;
      wheel_motor1_target = 0;
      wheel_motor2_target = 0;
      return;
    }

    Y1 = leftY + ZeparamremoteValue + EH_rollflag * roll_EH + jump_vlaue - Shake_shoulder_vakue;
    y2 = rightY + ZeparamremoteValue - EH_rollflag * roll_EH + jump_vlaue + Shake_shoulder_vakue;

  float leg_balance = leg_balance_kp * pitch + leg_balance_kd * gyroY;
  leg_balance = constrainValue(leg_balance, -leg_balance_limit, leg_balance_limit);

  x1 = leftX + -robot_kp * (-forwardBackward - (motor1_vel + (motor2_vel)) / 2) + -0.02 * gyroY + leg_balance; //-0.02 * gyroY
  x2 = rightX + robot_kp * (-forwardBackward - (motor1_vel + (motor2_vel)) / 2) + -0.02 * gyroY + leg_balance; //-0.02 * gyroY
  // 限幅Y轴幅度 避免超限导致逆解出问题
  Y1 = constrainValue(Y1, 130, 380);
  y2 = constrainValue(y2, 130, 380);
  // if (aaa++ > 300)
  // {
  //   aaa = 0;
  //   Serial.print(Y1);
  //   Serial.print("\t");
  //   Serial.print(y2);
  //   Serial.print("\t");
  //   Serial.println(ZeparamremoteValue);
  // }
  // Serial.print(ZeparamremoteValue); 
  //   Serial.print(",");
  // Serial.print(Y1);
  // Serial.print(",");
  // Serial.println(y2);
  // Serial.print(",");
  // Serial.print(x1);
  // Serial.print(",");
 
}

float constrainValue(float value, float minValue, float maxValue)
{
  if (value > maxValue)
    return maxValue;
  if (value < minValue)
    return minValue;
  return value;
}

int mapJoystickValuerollzeparam(int inputValue)
{
  if (inputValue < 1000)
    inputValue = 1000;
  if (inputValue > 2000)
    inputValue = 2000;

  int output = ((inputValue - 1500) / 20);

  return output;
}

float mapJoystickValuevel(int inputValue)
{
  if (inputValue < 1000)
    inputValue = 1000;
  if (inputValue > 2000)
    inputValue = 2000;
  if (inputValue < 1600 && inputValue > 1400)
    inputValue = 1500;
  float mappedValue = (inputValue - 1500) / 100.0;
  return mappedValue;
}

// 摇杆轴数据的映射处理
float mapJoystickValueInt(int inputValue)
{
  if (inputValue < 1000)
    inputValue = 1000;
  if (inputValue > 2000)
    inputValue = 2000;
  float mappedValue = (inputValue - 1500) / 100.0;
  if (mappedValue > -0.7 && mappedValue < 0.7)
  {
    mappedValue = 0;
  }
  return mappedValue;
}
float mapJoystickValuesteering(int inputValue)
{
  if (inputValue < 1000)
    inputValue = 1000;
  if (inputValue > 2000)
    inputValue = 2000;
  if (inputValue < 1600 && inputValue > 1400)
    inputValue = 1500;
  float mappedValue = (inputValue - 1500) / 2.0;
  return mappedValue;
}
// 步态变化函数 用于前后移动模仿人滑动前行 通过遥控器控制
int trot(int inputValue)
{
  if (inputValue < 1000)
    inputValue = 1000;
  if (inputValue > 2000)
    inputValue = 2000;
  if (inputValue > 1400 && inputValue < 1550)
    inputValue = 1500;
  int output = ((inputValue - 1500) / 10);
  return output;
}

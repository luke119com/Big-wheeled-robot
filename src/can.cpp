#include "can.h"

MIT MITCtrlParam;
// 左腿关节电机MIT控制
MIT LeftFronMITCtrlParam;
MIT LeftRearMITCtrlParam;

// 右腿关节电机MIT控制
MIT RightFronMITCtrlParam;
MIT RightRearMITCtrlParam;
float t;
uint32_t prev_ts;
uint16_t printCount = 0;
int motorIndex = 0;                            // 当前启动的电机索引
unsigned long lastSendTime = 0; // 记录上次发送时间
float Am_kp = 0;

// 定义腿部标识和对应的控制参数
struct LegCommand
{
  uint8_t id;         // CAN ID
  MIT &control_param; // 对应的控制参数
};


void CAN_Control()
{
  unsigned long currentTime = millis(); // 获取当前时间

  recCANMessage(); // CAN接收函数
  t += 0.0001f;

  MITCtrlParam.pos = 0;
  MITCtrlParam.vel = 0;
  MITCtrlParam.kp = 0;
  MITCtrlParam.kd = 0;
  MITCtrlParam.tor = 0;
  // 左腿关节电机1 控制参数  pos正  MIT协议为左后腿
  LeftFronMITCtrlParam.pos = 1 * motorLeftFront; // 28 - motorLeftRear motorLeftFront
  LeftFronMITCtrlParam.vel = 0;
  LeftFronMITCtrlParam.kp =17+Am_kp;
  LeftFronMITCtrlParam.kd = 1.5;
  LeftFronMITCtrlParam.tor = 0;
  // 左腿关节电机2 控制参数 pos负 控制升高降低  MIT协议为左前腿
  LeftRearMITCtrlParam.pos = 1 * motorLeftRear; //-23 + motorLeftFront
  LeftRearMITCtrlParam.vel = 0;
  LeftRearMITCtrlParam.kp =17+Am_kp;
  LeftRearMITCtrlParam.kd =1.5;
  LeftRearMITCtrlParam.tor = 0;

  // 右腿关节电机1 控制参数
  RightFronMITCtrlParam.pos = motorRightFront ;
  RightFronMITCtrlParam.vel = 0;
  RightFronMITCtrlParam.kp = 20+Am_kp;
  RightFronMITCtrlParam.kd =1.7;
  RightFronMITCtrlParam.tor = 0;
  // 右腿关节电机2 控制参数
  RightRearMITCtrlParam.pos = motorRightRear;//motorRightRear
  RightRearMITCtrlParam.vel = 0;
  RightRearMITCtrlParam.kp = 20+Am_kp;
  RightRearMITCtrlParam.kd =1.7;
  RightRearMITCtrlParam.tor = 0;

  uint32_t current_ts = micros();
  if (current_ts - prev_ts >= 1000) // 1s 1000000
  {                                 // 1ms
    printCount++;
    if (printCount >= 200)
    {
      printCount = 0;
      // Serial.printf("total Send num:%d, rec Num:%d\n", sendNum, recNum);
      if (motorIndex < 4 ) //发送使能指令
      {
        Serial.println("...");
        startMotor(motorIndex);           // 启动当前索引的电机
        motorIndex++;                     // 准备启动下一个电机
      }
      sendNum = 0;
      recNum = 0;
    }
    //打印关节电机电角度 1 2 3 4
    // Serial.printf("%.2f,%.2f,%.2f,%.2f\n", devicesState[0].pos, devicesState[1].pos, devicesState[2].pos,devicesState[3].pos);
    // Serial.print(motorLeftRear);
    // Serial.print(",");
    // Serial.println(  motorRightFront );
    prev_ts = current_ts;   
  }

  if (currentTime - lastSendTime >= SEND_INTERVAL)
  {
    // 发送命令
    sendMITCommand(0x02, LeftRearMITCtrlParam);
    sendMITCommand(0x01, RightRearMITCtrlParam);
    sendMITCommand(0x03, RightFronMITCtrlParam);
    sendMITCommand(0x04, LeftFronMITCtrlParam);
    lastSendTime = currentTime; // 更新最后发送时间
  }
}


// 内部启动指定电机
void startMotor(int motorIndex)
{
  switch (motorIndex)
  {
  case 0:
    enable(0x01); // 启用设备
    Serial.println("Motor 1 started!");
    break;
  case 1:
    enable(0x02); // 启用设备
    Serial.println("Motor 2 started!");
    break;
  case 2:
    enable(0x03); // 启用设备
    Serial.println("Motor 3 started!");
    break;
  case 3:
    enable(0x04); // 启用设备
    Serial.println("Motor 4 started!");
    break;
  default:
    break;
  }
}




void posInit()
{

  setZero(0x01);
  setZero(0x02);
  setZero(0x03);
  setZero(0x04); 

}

void recordMotorStartPos()
{
  motorPos1 = devicesState[0].pos;
  motorPos2 = devicesState[1].pos;
  motorPos3 = devicesState[2].pos;
  motorPos4 = devicesState[3].pos;
}
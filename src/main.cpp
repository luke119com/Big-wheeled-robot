#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "bipedal_data.h"
#include "CAN_comm.h"
#include "config.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ppm.h"
#include "Motor.h"
#include "robot.h"
#include "can.h"
#include "pid.h"
#include "WS2812.h"
#include "PS2.h"

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数
#define PIN        0
#define NUMPIXELS 62

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
MPU6050 mpu6050 = MPU6050(Wire);//实例化MPU6050
hw_timer_t *timer = NULL; // Timer for accurate pulse width measurement
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
int cnt;

void IMUTask(void *pvParameters);
void testdataprint(); //调试打印函数
void Open_thread_function();//启动线程


void setup()
{
  Wire.begin(1, 2, 400000UL);//初始化IIC
  Serial.begin(115200);//初始化调试串口
  mpu6050.begin(); //初始化MPU陀螺仪
  Open_thread_function();//启动线程
  CANInit(); // 初始化CAN
  // ppm_init(); //遥控器读取中断初始化
  ps2Init();
  motorInit();
  // CAN_Control();//使能关节电机
  delay(1000);
  enableMotor();
  // posInit();//重新设置零位
  // pixels.begin();//初始化RGB
  delay(1000);
}


void loop()
{
  // ws2812Test(pixels);
  serialReceiveUserCommand();                                // 串口数据输入处理 用于调试pid用
  PIDValues pid = interpolatePID(ZeparamremoteValue);         //PID线性拟合函数
  wheel_control();                                            // 轮子 霍尔电机PID控制函数
  CAN_Control();                                              // CAN 关节电机控制函数
  // remote_switch();                                            // 遥控开关
  PS2_switch();
  jump_control();                                          // 机器人跳跃控制
  inverseKinematics();                                        // 运动学逆解
  robot_control();                                            // 机器人行为控制
  sendMotorTargets(up_start * wheel_motor1_target, up_start * wheel_motor2_target); // 发送控制轮毂电机的目标值
  // storeFilteredPPMData();                                     // 对ppm数据进行滤波处理 避免数据大幅度跳动
  // mapPPMToRobotControl();                                     // 处理遥控器数据 将其映射为机器人行为控制
  mapPs2ToRobotControl();
  // testdataprint();                                         // 测试数据打印 可以打印输出相关调节信息
  // ledFlush(pixels);                                        //RGB
} 

// 测试数据打印函数 用于打印调试相关数据
void testdataprint()
{
  if (cnt++ > 5000)
  {
    cnt = 0;
    Serial.print(gyroZ);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.print(gyroY);
    Serial.print("\t");
    Serial.println(ZeparamremoteValue);
  }
}
//启动线程
void Open_thread_function() 
{
    // 陀螺仪读取任务进程
    xTaskCreatePinnedToCore(
        IMUTask,   // 任务函数
        "IMUTask", // 任务名称
        4096,      // 堆栈大小
        NULL,      // 传递的参数
        1,         // 任务优先级
        NULL,      // 任务句柄
        1          // 运行在核心 0
    );
}

// 陀螺仪数据读取
void IMUTask(void *pvParameters) 
{
  gyroY = mpu6050.getGyroY();  // 第一次读出来作为初值
  roll = mpu6050.getAngleX();
  while (true) 
  {
    mpu6050.update();
    pitch = mpu6050.getAngleY();
    yaw = mpu6050.getAngleZ();
    gyroX = mpu6050.getGyroX();
    gyroZ = mpu6050.getGyroZ();
    // 这里是设置一定死区 避免数据的波动
    if (gyroZ > -8 && gyroZ < 8) gyroZ = 0;
    roll = lowPassFilter(mpu6050.getAngleX(), roll, 0.05);
    gyroY = lowPassFilter(mpu6050.getGyroY(), gyroY, 0.005);

  }
}

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
#include "device_tuning.h"
#include "wifi_manager.h"

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define PIN 0
#define NUMPIXELS 62

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
MPU6050 mpu6050 = MPU6050(Wire);
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
int cnt;

void IMUTask(void *pvParameters);
void WiFiTask(void *pvParameters);
void testdataprint();
void Open_thread_function();

void setup()
{
  Wire.begin(1, 2, 400000UL);
  Serial.begin(115200);

  wifiManagerPortal.begin();
  xTaskCreatePinnedToCore(
      WiFiTask,
      "WiFiTask",
      8192,
      NULL,
      1,
      NULL,
      1);

  mpu6050.begin();
  Open_thread_function();
  CANInit();
  // ppm_init();
  ps2Init();
  motorInit();
  // CAN_Control();
  delay(1000);
  enableMotor();
  // posInit();
  // pixels.begin();
  delay(1000);
}

void loop()
{
  // ws2812Test(pixels);
  serialReceiveUserCommand();
  PIDValues pid = interpolatePID(ZeparamremoteValue);
  updateBalanceOffsetByCurrentHeight();
  wheel_control();
  CAN_Control();
  // remote_switch();
  PS2_switch();
  // jump_control();
  inverseKinematics();
  robot_control();
  sendMotorTargets(up_start * wheel_motor1_target, up_start * wheel_motor2_target);
  // storeFilteredPPMData();
  // mapPPMToRobotControl();
  mapPs2ToRobotControl();
  // testdataprint();
  // ledFlush(pixels);
}

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

void Open_thread_function()
{
  xTaskCreatePinnedToCore(
      IMUTask,
      "IMUTask",
      4096,
      NULL,
      1,
      NULL,
      1);
}

void IMUTask(void *pvParameters)
{
  accX = mpu6050.getAccX();
  accY = mpu6050.getAccY();
  accZ = mpu6050.getAccZ();
  imuTemp = mpu6050.getTemp();
  gyroY = mpu6050.getGyroY();
  roll = mpu6050.getAngleX();
  while (true)
  {
    mpu6050.update();
    accX = mpu6050.getAccX();
    accY = mpu6050.getAccY();
    accZ = mpu6050.getAccZ();
    imuTemp = mpu6050.getTemp();
    pitch = mpu6050.getAngleY();
    yaw = mpu6050.getAngleZ();
    gyroX = mpu6050.getGyroX();
    gyroZ = mpu6050.getGyroZ();
    if (gyroZ > -8 && gyroZ < 8)
    {
      gyroZ = 0;
    }
    roll = lowPassFilter(mpu6050.getAngleX(), roll, 0.05);
    gyroY = lowPassFilter(mpu6050.getGyroY(), gyroY, 0.005);
    vTaskDelay(1);
  }
}

void WiFiTask(void *pvParameters)
{
  while (true)
  {
    wifiManagerPortal.loop();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

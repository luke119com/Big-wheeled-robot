#include "pid.h"
#include <math.h>

namespace {
constexpr float kWheelOutputLimit = 8.0f;
constexpr float kWheelOutputSlew = 60.0f;
constexpr int kLowHeightSpeedLimit = 14;
constexpr int kHighHeightSpeedLimit = 8;
}

float vel_kp = -0;
float balance_kp = -0;
float balance_kd = 0;
float balance_ki = -0.0f;
int speed_limit = 5;
float motor1_target_vel = 0, motor2_target_vel = 0;
float wheel_motor1_target = 0, wheel_motor2_target = 0;
float clampToRange(float value, float minVal, float maxVal);

void wheel_control()
{
  static uint32_t last_us = 0;
  static float i_term = 0.0f;
  static float last_u1 = 0.0f;
  static float last_u2 = 0.0f;

  uint32_t now_us = micros();
  float dt = (last_us == 0) ? 0.005f : (now_us - last_us) * 1e-6f;
  last_us = now_us;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.02f) dt = 0.02f;

  motor1_target_vel = (-vel_kp * ((forwardBackward / 2) - (motor1_vel + motor2_vel) / 2));
  motor2_target_vel = (-vel_kp * ((forwardBackward / 2) - (motor1_vel + motor2_vel) / 2));

  float pitch_error1 = motor1_target_vel - pitch - balance_offset - remoteBalanceOffset;
  float pitch_error2 = motor2_target_vel - pitch - balance_offset - remoteBalanceOffset;

  const float pitch_deadband = 0.15f;
  if (fabsf(pitch_error1) < pitch_deadband) pitch_error1 = 0.0f;
  if (fabsf(pitch_error2) < pitch_deadband) pitch_error2 = 0.0f;

  float gyroY_use = gyroY;
  const float gyro_deadband = 0.6f;
  if (fabsf(gyroY_use) < gyro_deadband) gyroY_use = 0.0f;

  bool allow_i = (fabsf(forwardBackward) < 0.1f) && (fabsf(steering) < 0.1f) && (fabsf(gyroY_use) < 6.0f);
  if (allow_i) {
    i_term += pitch_error1 * dt;
    const float i_limit = 6.0f;
    if (i_term > i_limit) i_term = i_limit;
    if (i_term < -i_limit) i_term = -i_limit;
  } else {
    i_term *= 0.98f;
  }

  float u1 = balance_kp * pitch_error1 + balance_kd * gyroY_use + balance_ki * i_term;
  float u2 = balance_kp * pitch_error2 + balance_kd * gyroY_use + balance_ki * i_term;

  u1 = clampToRange(u1, -kWheelOutputLimit, kWheelOutputLimit);
  u2 = clampToRange(u2, -kWheelOutputLimit, kWheelOutputLimit);

  float max_delta = kWheelOutputSlew * dt;
  u1 = clampToRange(u1, last_u1 - max_delta, last_u1 + max_delta);
  u2 = clampToRange(u2, last_u2 - max_delta, last_u2 + max_delta);
  last_u1 = u1;
  last_u2 = u2;

  wheel_motor1_target = -u1 - 0.15f * steering;
  wheel_motor2_target = -u2 + 0.15f * steering;

  wheel_motor1_target = clampToRange(wheel_motor1_target, -speed_limit, speed_limit);
  wheel_motor2_target = clampToRange(wheel_motor2_target, -speed_limit, speed_limit);
}

PIDValues interpolatePID(int y_height)
{
  PIDValues result = {vel_kp, balance_kp, balance_kd, robot_kp};

  if (Shake_shoulder == 0)
  {
    float y0 = 0;
    float y1 = 80;
    float y2 = 150;
    PIDValues pid0 = {-0.55f, -0.183f, 0.055f, 4.0f};
    PIDValues pid1 = {-0.55f, -0.170f, 0.04f, 4.7f};
    PIDValues pid2 = {-0.54f, -0.158f, 0.042f, 6.2f};
    float ki0 = -0.003f, ki1 = -0.004f, ki2 = -0.0055f;

    if (y_height <= y1)
    {
      speed_limit = kLowHeightSpeedLimit;
      float t = (y_height - y0) / (y1 - y0);
      vel_kp = pid0.linear_vel_kp + t * (pid1.linear_vel_kp - pid0.linear_vel_kp);
      balance_kp = pid0.linear_balance_kp + t * (pid1.linear_balance_kp - pid0.linear_balance_kp);
      balance_kd = pid0.linear_balance_kd + t * (pid1.linear_balance_kd - pid0.linear_balance_kd);
      robot_kp = pid0.linear_robot_kp + t * (pid1.linear_robot_kp - pid0.linear_robot_kp);
      balance_ki = ki0 + t * (ki1 - ki0);
    }
    else
    {
      speed_limit = kHighHeightSpeedLimit;
      float t = (y_height - y1) / (y2 - y1);
      vel_kp = pid1.linear_vel_kp + t * (pid2.linear_vel_kp - pid1.linear_vel_kp);
      balance_kp = pid1.linear_balance_kp + t * (pid2.linear_balance_kp - pid1.linear_balance_kp);
      balance_kd = pid1.linear_balance_kd + t * (pid2.linear_balance_kd - pid1.linear_balance_kd);
      robot_kp = pid1.linear_robot_kp + t * (pid2.linear_robot_kp - pid1.linear_robot_kp);
      balance_ki = ki1 + t * (ki2 - ki1);
    }

    result.linear_vel_kp = vel_kp;
    result.linear_balance_kp = balance_kp;
    result.linear_balance_kd = balance_kd;
    result.linear_robot_kp = robot_kp;
  }

  return result;
}

String serialReceiveUserCommand()
{
  static String received_chars;
  String command = "";
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    received_chars += inChar;
    if (inChar == '\n')
    {
      command = received_chars;
      int commaPosition = command.indexOf(',');
      int newlinePosition = command.indexOf('\n');
      if (commaPosition != -1 && newlinePosition != -1)
      {
        String firstParam = command.substring(0, commaPosition);
        vel_kp = firstParam.toDouble();
        Serial.print("vel_kp:");
        Serial.println(vel_kp);

        String secondParamStr = command.substring(commaPosition + 1, newlinePosition);
        int secondCommaPosition = secondParamStr.indexOf(',');
        if (secondCommaPosition != -1)
        {
          balance_kp = secondParamStr.substring(0, secondCommaPosition).toDouble();
          balance_kd = secondParamStr.substring(secondCommaPosition + 1).toDouble();
        }
        else
        {
          robot_kp = secondParamStr.toDouble();
        }
        Serial.print("balance_kp:");
        Serial.println(balance_kp, 3);
        Serial.print("balance_kd:");
        Serial.println(balance_kd, 3);
        Serial.print("robot_kp:");
        Serial.println(robot_kp, 3);
      }
      received_chars = "";
    }
  }
  return command;
}

float clampToRange(float value, float minVal, float maxVal)
{
  if (value < minVal)
    return minVal;
  if (value > maxVal)
    return maxVal;
  return value;
}

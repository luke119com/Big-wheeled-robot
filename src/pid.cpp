#include "pid.h"
#include "device_tuning.h"

#include <math.h>

float vel_kp = -0;
float balance_kp = -0;
float balance_kd = 0;
float balance_ki = -0.0f;
int speed_limit = 5;
float motor1_target_vel = 0;
float motor2_target_vel = 0;
float wheel_motor1_target = 0;
float wheel_motor2_target = 0;

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
  if (dt < 0.001f)
  {
    dt = 0.001f;
  }
  if (dt > 0.02f)
  {
    dt = 0.02f;
  }

  motor1_target_vel = (-vel_kp * ((forwardBackward / 2) - (motor1_vel + motor2_vel) / 2));
  motor2_target_vel = (-vel_kp * ((forwardBackward / 2) - (motor1_vel + motor2_vel) / 2));

  float pitch_error1 = motor1_target_vel - pitch - balance_offset - remoteBalanceOffset;
  float pitch_error2 = motor2_target_vel - pitch - balance_offset - remoteBalanceOffset;

  const float pitch_deadband = 0.15f;
  if (fabsf(pitch_error1) < pitch_deadband)
  {
    pitch_error1 = 0.0f;
  }
  if (fabsf(pitch_error2) < pitch_deadband)
  {
    pitch_error2 = 0.0f;
  }

  float gyroY_use = gyroY;
  const float gyro_deadband = 0.6f;
  if (fabsf(gyroY_use) < gyro_deadband)
  {
    gyroY_use = 0.0f;
  }

  bool allow_i = (fabsf(forwardBackward) < 0.1f) && (fabsf(steering) < 0.1f) && (fabsf(gyroY_use) < 6.0f);
  if (allow_i)
  {
    i_term += pitch_error1 * dt;
    const float i_limit = 6.0f;
    if (i_term > i_limit)
    {
      i_term = i_limit;
    }
    if (i_term < -i_limit)
    {
      i_term = -i_limit;
    }
  }
  else
  {
    i_term *= 0.98f;
  }

  float u1 = balance_kp * pitch_error1 + balance_kd * gyroY_use + balance_ki * i_term;
  float u2 = balance_kp * pitch_error2 + balance_kd * gyroY_use + balance_ki * i_term;

  u1 = clampToRange(u1, -5, 5);
  u2 = clampToRange(u2, -5, 5);

  const float slew = 40.0f;
  float max_delta = slew * dt;
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
  if (Shake_shoulder == 0)
  {
    const HeightProfile profile = getHeightProfileForOffset(y_height);
    vel_kp = profile.velKp;
    balance_kp = profile.balanceKp;
    balance_kd = profile.balanceKd;
    balance_ki = profile.balanceKi;
    robot_kp = profile.robotKp;
    speed_limit = profile.speedLimit;
  }

  PIDValues result = {vel_kp, balance_kp, balance_kd, robot_kp};
  return result;
}

static bool parseFloatList(const String &input, float *values, int expectedCount)
{
  int start = 0;
  for (int i = 0; i < expectedCount; ++i)
  {
    int commaPos = input.indexOf(',', start);
    String token = (commaPos == -1) ? input.substring(start) : input.substring(start, commaPos);
    token.trim();
    if (token.length() == 0)
    {
      return false;
    }
    values[i] = token.toFloat();
    if (commaPos == -1)
    {
      return i == expectedCount - 1;
    }
    start = commaPos + 1;
  }
  return input.indexOf(',', start) == -1;
}

static void printFootPoseStatus()
{
  Serial.print("ik_mode:");
  Serial.println(serialFootPoseMode ? "on" : "off");
  Serial.print("ik_target_left:");
  Serial.print(serialLeftTargetX, 2);
  Serial.print(",");
  Serial.println(serialLeftTargetY, 2);
  Serial.print("ik_target_right:");
  Serial.print(serialRightTargetX, 2);
  Serial.print(",");
  Serial.println(serialRightTargetY, 2);
  Serial.print("ik_current_left:");
  Serial.print(x1, 2);
  Serial.print(",");
  Serial.println(Y1, 2);
  Serial.print("ik_current_right:");
  Serial.print(x2, 2);
  Serial.print(",");
  Serial.println(y2, 2);
}

static bool handleFootPoseCommand(String command)
{
  command.trim();
  if (command.length() == 0)
  {
    return true;
  }

  int commaPos = command.indexOf(',');
  String keyword = (commaPos == -1) ? command : command.substring(0, commaPos);
  String payload = (commaPos == -1) ? "" : command.substring(commaPos + 1);
  keyword.trim();
  payload.trim();
  keyword.toLowerCase();

  if (keyword == "ik" || keyword == "ikset")
  {
    float values[4];
    if (!parseFloatList(payload, values, 4))
    {
      Serial.println("ik_cmd_error: use ik,leftX,leftY,rightX,rightY");
      return true;
    }

    if (!validateSerialFootPoseTargets(values[0], values[1], values[2], values[3]))
    {
      Serial.println("ik_target_error: target out of reachable workspace");
      return true;
    }

    setSerialFootPoseTargets(values[0], values[1], values[2], values[3]);
    if (keyword == "ik")
    {
      setSerialFootPoseMode(true);
      robot_control();
      inverseKinematics();
      Serial.println("ik_move: enabled");
    }
    else
    {
      Serial.println("ik_set: stored");
    }
    printFootPoseStatus();
    return true;
  }

  if (keyword == "ikleft" || keyword == "ikright")
  {
    float values[2];
    if (!parseFloatList(payload, values, 2))
    {
      Serial.println("ik_cmd_error: use ikleft,x,y or ikright,x,y");
      return true;
    }

    float leftTargetX = serialLeftTargetX;
    float leftTargetY = serialLeftTargetY;
    float rightTargetX = serialRightTargetX;
    float rightTargetY = serialRightTargetY;
    if (keyword == "ikleft")
    {
      leftTargetX = values[0];
      leftTargetY = values[1];
    }
    else
    {
      rightTargetX = values[0];
      rightTargetY = values[1];
    }

    if (!validateSerialFootPoseTargets(leftTargetX, leftTargetY, rightTargetX, rightTargetY))
    {
      Serial.println("ik_target_error: target out of reachable workspace");
      return true;
    }

    setSerialFootPoseTargets(leftTargetX, leftTargetY, rightTargetX, rightTargetY);
    setSerialFootPoseMode(true);
    robot_control();
    inverseKinematics();
    Serial.println("ik_move: enabled");
    printFootPoseStatus();
    return true;
  }

  if (keyword == "ikmove")
  {
    if (!validateSerialFootPoseTargets(serialLeftTargetX, serialLeftTargetY, serialRightTargetX, serialRightTargetY))
    {
      Serial.println("ik_target_error: stored target out of reachable workspace");
      return true;
    }
    setSerialFootPoseMode(true);
    robot_control();
    inverseKinematics();
    Serial.println("ik_move: enabled");
    printFootPoseStatus();
    return true;
  }

  if (keyword == "ikstop")
  {
    setSerialFootPoseMode(false);
    Serial.println("ik_move: disabled");
    printFootPoseStatus();
    return true;
  }

  if (keyword == "ikstatus")
  {
    printFootPoseStatus();
    return true;
  }

  if (keyword == "ikhelp")
  {
    Serial.println("ik,leftX,leftY,rightX,rightY");
    Serial.println("ikset,leftX,leftY,rightX,rightY");
    Serial.println("ikleft,x,y");
    Serial.println("ikright,x,y");
    Serial.println("ikmove / ikstop / ikstatus");
    return true;
  }

  return false;
}

String serialReceiveUserCommand()
{
  static String received_chars;
  String command = "";
  while (Serial.available())
  {
    char inChar = static_cast<char>(Serial.read());
    received_chars += inChar;
    if (inChar == '\n')
    {
      command = received_chars;
      command.trim();
      if (handleFootPoseCommand(command))
      {
        received_chars = "";
        return command;
      }
      int commaPosition = command.indexOf(',');
      int newlinePosition = command.length();
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
  {
    return minVal;
  }
  if (value > maxVal)
  {
    return maxVal;
  }
  return value;
}

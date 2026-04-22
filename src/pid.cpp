#include "pid.h"
#include <math.h>

// PID参数
float vel_kp = -0;//速度环
float balance_kp = -0;//直立环Kp
float balance_kd = 0;//直立环K
float balance_ki = -0.0f;//直立环Ki(自动微调)
int speed_limit = 5; //轮毂电机速度限制
float motor1_target_vel = 0, motor2_target_vel = 0;
float wheel_motor1_target = 0, wheel_motor2_target = 0; // 电机目标值
float clampToRange(float value, float minVal, float maxVal);
// 底部轮子霍尔电机PID控制函数
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

  // Wheel speed loop
  motor1_target_vel = (-vel_kp * ((forwardBackward/2 )  - (motor1_vel  + motor2_vel) / 2));
  motor2_target_vel = (-vel_kp * ((forwardBackward/2 )  - (motor1_vel  + motor2_vel) / 2));

  float pitch_error1 = motor1_target_vel - pitch - balance_offset - remoteBalanceOffset;
  float pitch_error2 = motor2_target_vel - pitch - balance_offset - remoteBalanceOffset;

  const float pitch_deadband = 0.15f; // deg
  if (fabsf(pitch_error1) < pitch_deadband) pitch_error1 = 0.0f;
  if (fabsf(pitch_error2) < pitch_deadband) pitch_error2 = 0.0f;

  float gyroY_use = gyroY;
  const float gyro_deadband = 0.6f; // deg/s
  if (fabsf(gyroY_use) < gyro_deadband) gyroY_use = 0.0f;

  // I-term only when nearly still to auto-trim balance
  bool allow_i = (fabsf(forwardBackward) < 0.1f) && (fabsf(steering) < 0.1f) && (fabsf(gyroY_use) < 6.0f);
  if (allow_i) {
    i_term += pitch_error1 * dt;
    const float i_limit = 6.0f; // deg*s
    if (i_term > i_limit) i_term = i_limit;
    if (i_term < -i_limit) i_term = -i_limit;
  } else {
    i_term *= 0.98f;
  }

  float u1 = balance_kp * pitch_error1 + balance_kd * gyroY_use + balance_ki * i_term;
  float u2 = balance_kp * pitch_error2 + balance_kd * gyroY_use + balance_ki * i_term;

  u1 = clampToRange(u1, -5, 5);
  u2 = clampToRange(u2, -5, 5);

  // Output slew limit to reduce oscillation
  const float slew = 40.0f; // units per second
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

// //PID线性拟合函数
// PIDValues  interpolatePID(int y_height) {
//   if(Shake_shoulder == 0)
//   {
//       // 已知数据点
//       float y0 = 0, y1 = 80, y2 = 150;
//       PIDValues pid0 = {-0.55,-0.175, 0.060, 3.2};
//       PIDValues pid1 = {-0.55,-0.168,0.057, 4.5};
//       PIDValues pid2 = {-0.54,-0.155, 0.055, 6};
//       PIDValues result;
//       if (y_height <= y1) 
//       {
//           speed_limit = 5;
//           float t = (y_height - y0) / (y1 - y0);
//           vel_kp = pid0.linear_vel_kp + t * (pid1.linear_vel_kp - pid0.linear_vel_kp);
//           balance_kp = pid0.linear_balance_kp + t * (pid1.linear_balance_kp - pid0.linear_balance_kp);
//           balance_kd = pid0.linear_balance_kd + t * (pid1.linear_balance_kd - pid0.linear_balance_kd);
//           robot_kp = pid0.linear_robot_kp + t * (pid1.linear_robot_kp - pid0.linear_robot_kp);
//       } 
//       else 
//       {
//           speed_limit = 3;
//           float t = (y_height - y1) / (y2 - y1);
//           vel_kp = pid1.linear_vel_kp + t * (pid2.linear_vel_kp - pid1.linear_vel_kp);
//           balance_kp = pid1.linear_balance_kp + t * (pid2.linear_balance_kp - pid1.linear_balance_kp);
//           balance_kd = pid1.linear_balance_kd + t * (pid2.linear_balance_kd - pid1.linear_balance_kd);
//           robot_kp = pid1.linear_robot_kp + t * (pid2.linear_robot_kp - pid1.linear_robot_kp);
//       }
//       return result;
//   }
// }



//PID线性拟合函数
PIDValues interpolatePID(int y_height)
{
  if(Shake_shoulder == 0)
  {
      // 已知数据点
      const float y0 = 0.0f, y1 = 80.0f, y2 = 150.0f;
      const PIDValues pid0 = {-0.55f, -0.183f, 0.055f, 4.0f};
      const PIDValues pid1 = {-0.55f, -0.170f, 0.040f, 4.7f};
      const PIDValues pid2 = {-0.54f, -0.158f, 0.042f, 6.2f};
      const float ki0 = -0.003f, ki1 = -0.004f, ki2 = -0.0055f;
      PIDValues result = {vel_kp, balance_kp, balance_kd, robot_kp};
      float height = static_cast<float>(y_height);
      if (height < y0)
      {
          height = y0;
      }
      if (height > y2)
      {
          height = y2;
      }

      if (height <= y1) 
      {
          speed_limit = 5;
          float t = (height - y0) / (y1 - y0);
          vel_kp = pid0.linear_vel_kp + t * (pid1.linear_vel_kp - pid0.linear_vel_kp);
          balance_kp = pid0.linear_balance_kp + t * (pid1.linear_balance_kp - pid0.linear_balance_kp);
          balance_kd = pid0.linear_balance_kd + t * (pid1.linear_balance_kd - pid0.linear_balance_kd);
          robot_kp = pid0.linear_robot_kp + t * (pid1.linear_robot_kp - pid0.linear_robot_kp);
          balance_ki = ki0 + t * (ki1 - ki0);
      } 
      else 
      {
          speed_limit = 3;
          float t = (height - y1) / (y2 - y1);
          vel_kp = pid1.linear_vel_kp + t * (pid2.linear_vel_kp - pid1.linear_vel_kp);
          balance_kp = pid1.linear_balance_kp + t * (pid2.linear_balance_kp - pid1.linear_balance_kp);
          balance_kd = pid1.linear_balance_kd + t * (pid2.linear_balance_kd - pid1.linear_balance_kd);
          robot_kp = pid1.linear_robot_kp + t * (pid2.linear_robot_kp - pid1.linear_robot_kp);
          balance_ki = ki1 + t * (ki2 - ki1);
      }
      return result;
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

//PID串口调参函数
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
      command.trim();
      if (handleFootPoseCommand(command))
      {
        received_chars = "";
        return command;
      }
      int commaPosition = command.indexOf(',');
      int newlinePosition = command.length();
      if (commaPosition != -1 && newlinePosition != -1) //给的第一个值
      {
        String firstParam = command.substring(0, commaPosition);
        vel_kp = firstParam.toDouble(); //速度环Kp
        Serial.print("vel_kp:");
        Serial.println(vel_kp);

        String secondParamStr = command.substring(commaPosition + 1, newlinePosition);
        int secondCommaPosition = secondParamStr.indexOf(',');
        if (secondCommaPosition != -1) //如果给三个值
        {
          balance_kp = secondParamStr.substring(0, secondCommaPosition).toDouble();//直立环Kp
          balance_kd = secondParamStr.substring(secondCommaPosition + 1).toDouble();//直立环Kd
        }
        else //如果只给两个值
        {
          robot_kp = secondParamStr.toDouble();
        }
        Serial.print("balance_kp:");
        Serial.println(balance_kp,3);
        Serial.print("balance_kd:");
        Serial.println(balance_kd,3);
        Serial.print("robot_kp:");
        Serial.println(robot_kp,3);
      }
      received_chars = "";
    }
  }
  return command;
}

// 限幅函数
float clampToRange(float value, float minVal, float maxVal)
{
  if (value < minVal)
    return minVal;
  if (value > maxVal)
    return maxVal;
  return value;
}


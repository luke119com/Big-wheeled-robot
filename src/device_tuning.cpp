#include "device_tuning.h"

#include "robot.h"

namespace
{
constexpr float kMinLegHeight = 130.0f;
constexpr float kMaxLegHeight = 380.0f;
constexpr float kMinBalancePoint = -15.0f;
constexpr float kMaxBalancePoint = 15.0f;
constexpr float kMinVelKp = -2.0f;
constexpr float kMaxVelKp = 2.0f;
constexpr float kMinBalanceKp = -5.0f;
constexpr float kMaxBalanceKp = 5.0f;
constexpr float kMinBalanceKd = -2.0f;
constexpr float kMaxBalanceKd = 2.0f;
constexpr float kMinBalanceKi = -1.0f;
constexpr float kMaxBalanceKi = 1.0f;
constexpr float kMinRobotKp = -20.0f;
constexpr float kMaxRobotKp = 20.0f;
constexpr int kMinSpeedLimit = 1;
constexpr int kMaxSpeedLimit = 20;

float clampFloat(float value, float minValue, float maxValue)
{
  if (value < minValue)
  {
    return minValue;
  }

  if (value > maxValue)
  {
    return maxValue;
  }

  return value;
}

int clampInt(int value, int minValue, int maxValue)
{
  if (value < minValue)
  {
    return minValue;
  }

  if (value > maxValue)
  {
    return maxValue;
  }

  return value;
}

DeviceTuningConfig buildDefaultConfig()
{
  DeviceTuningConfig config = {};
  config.leftHeight = 300.0f;
  config.rightHeight = 300.0f;

  for (int i = 0; i < kHeightProfileCount; ++i)
  {
    const float t = static_cast<float>(i) / static_cast<float>(kHeightProfileCount - 1);
    config.profiles[i].offset = i * kHeightProfileStep;
    config.profiles[i].balancePoint = -0.5f + 0.5f * t;
    config.profiles[i].velKp = -0.55f + 0.01f * t;
    config.profiles[i].balanceKp = -0.183f + 0.025f * t;
    config.profiles[i].balanceKd = 0.055f - 0.013f * t;
    config.profiles[i].balanceKi = -0.003f - 0.0025f * t;
    config.profiles[i].robotKp = 4.0f + 2.2f * t;
    config.profiles[i].speedLimit = (i <= 8) ? 5 : 3;
  }

  return config;
}

DeviceTuningConfig g_config = buildDefaultConfig();

void normalizeProfile(HeightProfile &profile, int index)
{
  profile.offset = index * kHeightProfileStep;
  profile.balancePoint = clampFloat(profile.balancePoint, kMinBalancePoint, kMaxBalancePoint);
  profile.velKp = clampFloat(profile.velKp, kMinVelKp, kMaxVelKp);
  profile.balanceKp = clampFloat(profile.balanceKp, kMinBalanceKp, kMaxBalanceKp);
  profile.balanceKd = clampFloat(profile.balanceKd, kMinBalanceKd, kMaxBalanceKd);
  profile.balanceKi = clampFloat(profile.balanceKi, kMinBalanceKi, kMaxBalanceKi);
  profile.robotKp = clampFloat(profile.robotKp, kMinRobotKp, kMaxRobotKp);
  profile.speedLimit = clampInt(profile.speedLimit, kMinSpeedLimit, kMaxSpeedLimit);
}

void normalizeConfig(DeviceTuningConfig &config)
{
  config.leftHeight = clampFloat(config.leftHeight, kMinLegHeight, kMaxLegHeight);
  config.rightHeight = clampFloat(config.rightHeight, kMinLegHeight, kMaxLegHeight);

  for (int i = 0; i < kHeightProfileCount; ++i)
  {
    normalizeProfile(config.profiles[i], i);
  }
}
}

int clampHeightOffset(int offset)
{
  return clampInt(offset, 0, (kHeightProfileCount - 1) * kHeightProfileStep);
}

int getHeightProfileIndexForOffset(int offset)
{
  const int clampedOffset = clampHeightOffset(offset);
  return clampInt((clampedOffset + kHeightProfileStep / 2) / kHeightProfileStep, 0, kHeightProfileCount - 1);
}

DeviceTuningConfig getDeviceTuningConfig()
{
  DeviceTuningConfig config = g_config;
  config.leftHeight = leftY;
  config.rightHeight = rightY;
  return config;
}

void applyDeviceTuningConfig(const DeviceTuningConfig &config)
{
  g_config = config;
  normalizeConfig(g_config);

  leftY = g_config.leftHeight;
  rightY = g_config.rightHeight;

  if (!serialFootPoseMode)
  {
    Y1 = leftY;
    y2 = rightY;
  }

  updateBalanceOffsetByCurrentHeight();
}

HeightProfile getHeightProfileByIndex(int index)
{
  const int safeIndex = clampInt(index, 0, kHeightProfileCount - 1);
  return g_config.profiles[safeIndex];
}

HeightProfile getHeightProfileForOffset(int offset)
{
  return getHeightProfileByIndex(getHeightProfileIndexForOffset(offset));
}

HeightProfile getCurrentHeightProfile()
{
  return getHeightProfileForOffset(ZeparamremoteValue);
}

float getCurrentHeightForPitchControl()
{
  float leftHeight = leftY;
  float rightHeight = rightY;
  float heightOffset = static_cast<float>(clampHeightOffset(ZeparamremoteValue));

  if (serialFootPoseMode)
  {
    leftHeight = serialLeftTargetY;
    rightHeight = serialRightTargetY;
    heightOffset = 0.0f;
  }

  const float averageHeight = (leftHeight + rightHeight) * 0.5f + heightOffset;
  return clampFloat(averageHeight, kMinLegHeight, kMaxLegHeight);
}

float getCurrentPitchTarget()
{
  return getCurrentHeightProfile().balancePoint;
}

void updateBalanceOffsetByCurrentHeight()
{
  balance_offset = getCurrentPitchTarget();
}

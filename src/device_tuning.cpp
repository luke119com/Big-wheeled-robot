#include "device_tuning.h"

#include "robot.h"

namespace
{
constexpr float kMinLegHeight = 130.0f;
constexpr float kMaxLegHeight = 380.0f;
constexpr float kMinPitchTarget = -15.0f;
constexpr float kMaxPitchTarget = 15.0f;

DeviceTuningConfig g_config = {
    0.0f,
    300.0f,
    300.0f,
    {
        {130.0f, 0.0f},
        {255.0f, 0.0f},
        {380.0f, 0.0f},
    },
};

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

void normalizePitchMap(HeightPitchPoint *points, int count)
{
  for (int i = 0; i < count; ++i)
  {
    points[i].height = clampFloat(points[i].height, kMinLegHeight, kMaxLegHeight);
    points[i].pitch = clampFloat(points[i].pitch, kMinPitchTarget, kMaxPitchTarget);
  }

  for (int i = 0; i < count - 1; ++i)
  {
    for (int j = i + 1; j < count; ++j)
    {
      if (points[j].height < points[i].height)
      {
        const HeightPitchPoint temp = points[i];
        points[i] = points[j];
        points[j] = temp;
      }
    }
  }
}

float interpolatePitchMap(const HeightPitchPoint *points, int count, float height)
{
  if (count <= 0)
  {
    return 0.0f;
  }

  if (height <= points[0].height)
  {
    return points[0].pitch;
  }

  for (int i = 1; i < count; ++i)
  {
    if (height <= points[i].height)
    {
      const float span = points[i].height - points[i - 1].height;
      if (span <= 0.001f)
      {
        return points[i].pitch;
      }

      const float t = (height - points[i - 1].height) / span;
      return points[i - 1].pitch + t * (points[i].pitch - points[i - 1].pitch);
    }
  }

  return points[count - 1].pitch;
}
}

DeviceTuningConfig getDeviceTuningConfig()
{
  DeviceTuningConfig config = g_config;
  config.initPitch = init_pitch;
  config.leftHeight = leftY;
  config.rightHeight = rightY;
  return config;
}

void applyDeviceTuningConfig(const DeviceTuningConfig &config)
{
  g_config = config;
  g_config.initPitch = clampFloat(g_config.initPitch, kMinPitchTarget, kMaxPitchTarget);
  g_config.leftHeight = clampFloat(g_config.leftHeight, kMinLegHeight, kMaxLegHeight);
  g_config.rightHeight = clampFloat(g_config.rightHeight, kMinLegHeight, kMaxLegHeight);
  normalizePitchMap(g_config.pitchMap, 3);

  init_pitch = g_config.initPitch;
  leftY = g_config.leftHeight;
  rightY = g_config.rightHeight;

  if (!serialFootPoseMode)
  {
    Y1 = leftY;
    y2 = rightY;
  }

  updateBalanceOffsetByCurrentHeight();
}

float getPitchMapValueForHeight(float height)
{
  return interpolatePitchMap(g_config.pitchMap, 3, height);
}

float getCurrentHeightForPitchControl()
{
  float leftHeight = leftY;
  float rightHeight = rightY;
  float heightOffset = static_cast<float>(ZeparamremoteValue);

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
  return init_pitch + getPitchMapValueForHeight(getCurrentHeightForPitchControl());
}

void updateBalanceOffsetByCurrentHeight()
{
  balance_offset = getCurrentPitchTarget();
}

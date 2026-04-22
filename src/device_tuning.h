#ifndef DEVICE_TUNING_H
#define DEVICE_TUNING_H

struct HeightPitchPoint
{
  float height;
  float pitch;
};

struct DeviceTuningConfig
{
  float initPitch;
  float leftHeight;
  float rightHeight;
  HeightPitchPoint pitchMap[3];
};

DeviceTuningConfig getDeviceTuningConfig();
void applyDeviceTuningConfig(const DeviceTuningConfig &config);
float getPitchMapValueForHeight(float height);
float getCurrentHeightForPitchControl();
float getCurrentPitchTarget();
void updateBalanceOffsetByCurrentHeight();

#endif

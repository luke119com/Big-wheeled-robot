#ifndef DEVICE_TUNING_H
#define DEVICE_TUNING_H

constexpr int kHeightProfileCount = 16;
constexpr int kHeightProfileStep = 10;

struct HeightProfile
{
  int offset;
  float balancePoint;
  float velKp;
  float balanceKp;
  float balanceKd;
  float balanceKi;
  float robotKp;
  int speedLimit;
};

struct DeviceTuningConfig
{
  float leftHeight;
  float rightHeight;
  HeightProfile profiles[kHeightProfileCount];
};

DeviceTuningConfig getDeviceTuningConfig();
void applyDeviceTuningConfig(const DeviceTuningConfig &config);

int clampHeightOffset(int offset);
int getHeightProfileIndexForOffset(int offset);
float getCurrentHeightForPitchControl();
float getCurrentPitchTarget();
HeightProfile getHeightProfileByIndex(int index);
HeightProfile getHeightProfileForOffset(int offset);
HeightProfile getCurrentHeightProfile();
void updateBalanceOffsetByCurrentHeight();

#endif

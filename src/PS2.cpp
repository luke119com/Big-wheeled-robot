#include "PS2.h"

int error = -1;
byte type = 0;
byte vibrate = 0;
int tryNum = 1;
uint32_t R2_LONG_MS = 1500;
uint32_t r2_press_ts = 0;
bool r2_prev = false;
bool r2_long_fired = false;
uint32_t R1_LONG_MS = 1500;
uint32_t r1_press_ts = 0;
bool r1_prev = false;
bool r1_long_fired = false;
PS2X ps2x;

void ps2Init()
{
  while (error != 0) {
    delay(1000);
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print("#try config ");
    Serial.println(tryNum);
    tryNum++;
  }

  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.println(" Unknown Controller type found ");
      break;
    case 1:
      Serial.println(" DualShock Controller found ");
      break;
    case 2:
      Serial.println(" GuitarHero Controller found ");
      break;
    case 3:
      Serial.println(" Wireless Sony DualShock Controller found ");
      break;
  }
}

float normAxis(int intput, int dead, int center)
{
  int vel = intput - center;
  if (abs(vel) <= dead) return 0;
  float mag = (abs(vel) - dead) / (127.0f - dead);
  if (mag < 0) mag = 0;
  return (vel >= 0 ? -1 : 1) * mag;
}

unsigned long lastPadTime = 0;
unsigned long lastBalanceTime = 0;
unsigned long lastShakeTime = 0;

void mapPs2ToRobotControl()
{
  unsigned long now = millis();

  if (ps2x.Button(PSB_PAD_UP) && now - lastPadTime > 200) {
    ZeparamremoteValue = min(ZeparamremoteValue + 10, 150);
    lastPadTime = now;
    if (nowLed != KEEP_STEADY) nowLed = LED_UP;
  }

  if (ps2x.Button(PSB_PAD_DOWN) && now - lastPadTime > 200) {
    ZeparamremoteValue = max(ZeparamremoteValue - 10, 0);
    lastPadTime = now;
    if (nowLed != KEEP_STEADY) nowLed = LED_DOWN;
  }

  if (jump_flag == 0 && EH_rollflag == 0) {
    if (ps2x.Button(PSB_PAD_RIGHT) && now - lastShakeTime > 200) {
      Shake_shoulder_vakue = min(Shake_shoulder_vakue + 3, 24);
      lastShakeTime = now;
    }
    if (ps2x.Button(PSB_PAD_LEFT) && now - lastShakeTime > 200) {
      Shake_shoulder_vakue = max(Shake_shoulder_vakue - 3, -24);
      lastShakeTime = now;
    }
  }

  if (ps2x.Button(PSB_L1) && now - lastBalanceTime > 100) {
    remoteBalanceOffset = remoteBalanceOffset + 0.2;
    lastBalanceTime = now;
  }

  if (ps2x.Button(PSB_L2) && now - lastBalanceTime > 100) {
    remoteBalanceOffset = remoteBalanceOffset - 0.2;
    lastBalanceTime = now;
  }

  forwardBackward = normAxis(ps2x.Analog(PSS_RY), 5, 128) * 4;
  steering = -normAxis(ps2x.Analog(PSS_LX), 5, 128) * 6;
}

void PS2_switch()
{
  ps2x.read_gamepad(false, vibrate);
  uint32_t now = millis();
  bool r2 = ps2x.Button(PSB_R2);
  bool r1 = ps2x.Button(PSB_R1);

  if (r2 && EH_rollflag == 0 && Shake_shoulder_vakue == 0) {
    if (!r2_prev) {
      r2_press_ts = now;
      r2_long_fired = false;
    }
    if (!r2_long_fired && (now - r2_press_ts >= R2_LONG_MS)) {
      r2_long_fired = true;
      jump_flag = 1;
    }
  } else {
    jump_flag = 0;
  }
  r2_prev = r2;

  if (r1 && jump_flag == 0) {
    if (!r1_prev) {
      r1_press_ts = now;
      r1_long_fired = false;
    }

    if (!r1_long_fired && (now - r1_press_ts >= R1_LONG_MS)) {
      r1_long_fired = true;
      if (EH_rollflag == 0) {
        EH_rollflag = 1;
        nowLed = KEEP_STEADY;
      } else {
        EH_rollflag = 0;
        nowLed = EXIT;
      }
    }
  }
  r1_prev = r1;

  if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
    up_start = 1;
  }
  if (ps2x.ButtonPressed(PSB_CROSS)) {
    up_start = 0;
  }

  if (ps2x.ButtonPressed(PSB_SQUARE)) {
    toggleEndEffectorSineTest();
    Serial.println(endEffectorSineTestEnabled ? "square sine path on" : "square sine path off");
  }

  if (ps2x.ButtonPressed(PSB_CIRCLE)) {
    Shake_shoulder_vakue = 0;
  }
}

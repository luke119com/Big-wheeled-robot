#include"PS2.h"

int error = -1;
byte type = 0;
byte vibrate = 0;
int tryNum = 1;
bool ps2Connected = false;
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
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  Serial.print("#try config ");
  Serial.println(tryNum);
  tryNum++;

  ps2Connected = (error == 0);
  if (!ps2Connected)
  {
    Serial.println("PS2 controller not detected. System will continue and retry in loop.");
    return;
  }

  type = ps2x.readType();
  switch(type) {
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

bool ps2IsConnected()
{
  return ps2Connected;
}


float normAxis(int intput,int dead,int center)
{
  int vel = intput-center;
  if(abs(vel)<=dead) return 0;
  float mag = (abs(vel) - dead) / (127.0f - dead);
  if (mag < 0) mag = 0;
  return (vel>=0?-1:1)*mag;

}

unsigned long lastPadTime = 0;
unsigned long lastBalanceTime = 0;
unsigned long lastShakeTime = 0;
unsigned long lastPs2PollTime = 0;
unsigned long lastPs2ReconnectAttempt = 0;

void mapPs2ToRobotControl()
{ 
  unsigned long now = millis();

  if (serialFootPoseMode || !ps2Connected)
  {
    forwardBackward = 0;
    steering = 0;
    return;
  }

  // if (ps2x.ButtonPressed(PSB_PAD_UP)) {
  //   ZeparamremoteValue = min(ZeparamremoteValue + 10, 150);
  //   if(nowLed!=KEEP_STEADY){
  //   nowLed = LED_UP;
  //   }
  // } 

  // if (ps2x.ButtonPressed(PSB_PAD_DOWN)) {
  //   ZeparamremoteValue = max(ZeparamremoteValue - 10, 0);
  //   if(nowLed!=KEEP_STEADY){
  //   nowLed = LED_DOWN;
  //   }
  // }  

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

  if(jump_flag==0&&EH_rollflag ==0){
    if(ps2x.Button(PSB_PAD_RIGHT)&&now - lastShakeTime>200){
      Shake_shoulder_vakue = min(Shake_shoulder_vakue + 3, 24);
      lastShakeTime=now;
    }
  // if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) Shake_shoulder_vakue = min(Shake_shoulder_vakue + 3, 24);
    if(ps2x.Button(PSB_PAD_LEFT)&&now - lastShakeTime>200){
      Shake_shoulder_vakue = max(Shake_shoulder_vakue - 3, -24);
      lastShakeTime=now;
    }    
  // if (ps2x.ButtonPressed(PSB_PAD_LEFT)) Shake_shoulder_vakue = max(Shake_shoulder_vakue - 3, -24);
  }

  // if (ps2x.ButtonPressed(PSB_L1)) remoteBalanceOffset = remoteBalanceOffset + 0.2;
  // if (ps2x.ButtonPressed(PSB_L2)) remoteBalanceOffset = remoteBalanceOffset - 0.2;

  if(ps2x.Button(PSB_L1)&&now-lastBalanceTime>100){
    remoteBalanceOffset = remoteBalanceOffset + 0.2;
    lastBalanceTime=now;
  } 
  
  if(ps2x.Button(PSB_L2)&&now-lastBalanceTime>100) 
  {
    remoteBalanceOffset = remoteBalanceOffset - 0.2;
    lastBalanceTime=now;  
  }

  forwardBackward = normAxis(ps2x.Analog(PSS_RY),5,128)*4;
  steering = -normAxis(ps2x.Analog(PSS_LX),5,128)*6;
  // Serial.println(forwardBackward);
  // delay(1);
}

void PS2_switch()
{
  uint32_t now = millis();

  if (!ps2Connected)
  {
    if (now - lastPs2ReconnectAttempt >= 3000)
    {
      lastPs2ReconnectAttempt = now;
      ps2Init();
    }
    return;
  }

  if (now - lastPs2PollTime < 4) {
    return;
  }
  lastPs2PollTime = now;

  if (!ps2x.read_gamepad(false, vibrate)) {
    return;
  }

  bool r2 = ps2x.Button(PSB_R2);
  bool r1 = ps2x.Button(PSB_R1);

  if(r2&&EH_rollflag ==0&&Shake_shoulder_vakue==0)
  {
  if(!r2_prev){
    r2_press_ts = now;
    r2_long_fired = false;
  }
  if (!r2_long_fired && (now - r2_press_ts >= R2_LONG_MS)) {
    r2_long_fired = true;        // 长按触发一次
    // Serial.println(" 起跳");
    jump_flag=1;
  }
  }else{
    // Serial.println(" 不起跳");
    jump_flag=0;
  }
  r2_prev = r2;

  if(r1&&jump_flag==0){
  if(!r1_prev){
    r1_press_ts = now;
    r1_long_fired = false;
  }

  if (!r1_long_fired && (now - r1_press_ts >= R1_LONG_MS)) {
    r1_long_fired = true;        // 长按触发一次
    if(EH_rollflag==0){
      // Serial.println(" 自稳开启且灯亮");
      EH_rollflag=1;
      nowLed =KEEP_STEADY;
    }else{
      EH_rollflag=0;
      nowLed =EXIT;
      // Serial.println(" 自稳关闭且关灯");
    }
  }
  }
  r1_prev = r1;

  if(ps2x.ButtonPressed(PSB_TRIANGLE))//使能轮毂
  {
    up_start = 1;
    // Serial.println("轮毂启动");
  }
  if(ps2x.ButtonPressed(PSB_CROSS))//失能轮毂
  {
    up_start = 0;
    // Serial.println(" 轮毂关闭");
  }

  if (ps2x.ButtonPressed(PSB_SQUARE)) {
    if(ledSwitch==false){
      ledSwitch=true;
      // Serial.println(" 开灯");
    }else{
      ledSwitch=false;
      // Serial.println(" 关灯");
    } 
  }

  if(ps2x.ButtonPressed(PSB_CIRCLE)){
    Shake_shoulder_vakue=0;
    // Serial.println("高度回正");
  }

}

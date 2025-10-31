#ifndef WS2812_H
#define WS2812_H
#include <Adafruit_NeoPixel.h>

#define DATA_PIN 0
enum LedMode {LED_UP,LED_DOWN,KEEP_STEADY,EXIT};
extern volatile LedMode nowLed; 
extern bool ledSwitch;

void ws2812Test(Adafruit_NeoPixel pixels);
void setLedMode(LedMode nowled );
void ledFlush(Adafruit_NeoPixel &pixels);
#endif
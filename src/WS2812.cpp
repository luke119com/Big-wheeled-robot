#include <Arduino.h>
#include "WS2812.h"

bool ledSwitch = false;
volatile LedMode nowLed;


void ws2812Test(Adafruit_NeoPixel pixels)
{
  // pixels.clear();
  // for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

  //   // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
  //   // Here we're using a moderately bright green color:
  //   pixels.setPixelColor(i, pixels.Color(0, 150, 0));

  //   pixels.show();   // Send the updated pixel colors to the hardware.
  //   delay(500); // Pause before next pass through loop
  // }
  pixels.fill(pixels.Color(255, 0, 0));
  pixels.show();
  //  delay(10);
}

void setLedMode(LedMode nowled ) 
{
  nowLed = nowled;
}

void ledFlush(Adafruit_NeoPixel &pixels) {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < 40) return;   
  last = now;

 
  static int prevMode = -1;
  static int row = 0, last_row = -1;             
  static int downRow = 4, lastDownRow = -1;      

  if (!ledSwitch) { pixels.clear(); pixels.show(); return; }


  if (prevMode != nowLed) {
    row = 0; last_row = -1;
    downRow = 4; lastDownRow = -1;
    prevMode = nowLed;
  }

  switch (nowLed) {
    case LED_DOWN: {
      if (last_row >= 0) {
        for (int s = 0; s < 8; ++s) pixels.setPixelColor(s * 5 + last_row, 0);
      }
      if (row < 5) {
        for (int s = 0; s < 8; ++s) pixels.setPixelColor(s * 5 + row, pixels.Color(0,150,0));
        last_row = row;
        row++;                       
        if (row >= 5) {              
          nowLed = EXIT;
        }
      }
    } break;

    case LED_UP: {
      
      if (lastDownRow >= 0) {
        for (int s = 0; s < 8; ++s) pixels.setPixelColor(s * 5 + lastDownRow, 0);
      }
      if (downRow >= 0) {
        for (int s = 0; s < 8; ++s) pixels.setPixelColor(s * 5 + downRow, pixels.Color(0,150,0));
        lastDownRow = downRow;
        downRow--;                    
        if (downRow < 0) {            
          nowLed = EXIT;
        }
      }
    } break;

    case KEEP_STEADY:
      pixels.fill(pixels.Color(255, 0, 0));  
      break;

    case EXIT:
      pixels.fill(pixels.Color(0, 0, 0));  
      break;

    default: break;
  }

    for (int i = 40; i < 62; i++) {
    pixels.setPixelColor(i, pixels.Color(255, 255, 255));  // 白色
  }
  pixels.show();
}



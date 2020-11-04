// 190630

#include "Wire.h"
#include <Adafruit_LiquidCrystal.h>
Adafruit_LiquidCrystal lcd(0);

//  Xbee  Mega
//  2     19 rx1
//  3     18 tx1

int   RTCMcount = 0;

//====================================================================
void setup() {


  lcd.begin(20, 4);
  lcd.clear();
  lcd.setCursor(0, 3);
  lcd.print("Xbee Monitor 201101");
  lcd.setCursor(0, 0);

  Serial.begin(115200);
  Serial1.begin(38400);   // xBee
  delay(500); 
  Serial.print("Monitor");
  
  }
//====================================================================
void loop() {
  byte  xchr;
  
  while (Serial1.available() > 0) {
    xchr = Serial1.read();
    Serial.print(" ");
    if (xchr < 0x10)
      Serial.print("0");
    Serial.print(xchr, HEX);
    if (xchr == 0xd3) {
      RTCMcount++;
      lcd.setCursor(0, 0);
      lcd.print("RTCM msg: ");
      lcd.print((String)RTCMcount);
      Serial.println();
      Serial.print((String)RTCMcount);
      }   // d3

    }   // while avail
  }

// 190630

#include "Wire.h"
#include <Adafruit_LiquidCrystal.h>
Adafruit_LiquidCrystal lcd(0);

//  Xbee  Mega
//  2     19 rx1
//  3     18 tx1

int   RTCMcount = 0;
byte  buffer[1024];       // max message len
//====================================================================
void hexout(byte xchr) {
  Serial.print(" ");
  if (xchr < 0x10)
    Serial.print("0");
    Serial.print(xchr, HEX);
  }
//====================================================================
void setup() {


  lcd.begin(20, 4);
  lcd.clear();
  lcd.setCursor(0, 3);
  lcd.print("Xbee Monitor 201106");
  lcd.setCursor(0, 0);

  Serial.begin(115200);
  //  Serial1.begin(38400);   // xBee
  //Serial1.begin(19200);   // xBee
  Serial1.begin(9600);   // xBee
  delay(500);
  Serial.println("Monitor");

}
//====================================================================
void loop() {
  int   avail;
  int   ibuff;
  int   nbytes;
  int   pbytes;
  int   rbytes;
  byte  xchr;
  byte  pktlen[2];

  while (Serial1.available() > 0) {
    xchr = Serial1.read();
    if (xchr == 0xd3) {
      if (Serial1.available() >= 2);                // wait for data
        Serial1.readBytes(pktlen, 2);         // bytes 1 & 2 length
      if (pktlen[0] != 0)                     // all RTCM messages are <= 80
        return;
      nbytes = pktlen[1] + 3; // to read:  length - 3 header bytes + 6 OH
      pbytes = nbytes + 3;    // total packet length
      ibuff = 0;
      while (nbytes > 0) {
        avail = Serial1.available();
        if (avail > nbytes) {
          Serial1.readBytes(&buffer[ibuff], nbytes);
          continue;
          }
        if (avail > 0) {
          Serial1.readBytes(&buffer[ibuff], avail);
          nbytes -= avail;
          ibuff += avail;
          }
        }
//      if (Serial1.available() >= nbytes)
//        rbytes = Serial1.readBytes(buffer, nbytes);
      RTCMcount++;
      lcd.setCursor(0, 0);
      lcd.print("RTCM msg: ");
      lcd.print((String)RTCMcount);
      Serial.println();
      Serial.print("RTCM msg: ");
      Serial.print((String)RTCMcount);
      Serial.print(" ");
      Serial.println(rbytes);
      hexout(0xd3);
      hexout(pktlen[0]);
      hexout(pktlen[1]);
      for (int i = 0; i < pbytes-3; i++) {
        xchr = buffer[i];
        hexout(xchr);
        if (((i+4) % 16) == 0)
          Serial.println();
      }
    }
  }   // while avail
}

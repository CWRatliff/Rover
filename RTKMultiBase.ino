/*
  Send UBX binary commands to enable RTCM sentences on Ublox ZED-F9P module
  By: Nathan Seidle
  SparkFun Electronics
  Date: September 7th, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example does all steps to configure and enable a ZED-F9P as a base station:
    Emulate survey-in using NTRIP for corrections
    Enable six RTCM messages
    Begin outputting RTCM bytes
*/
#define LSMP    10    // arbitrary sample size
#include <Wire.h>     //Needed for I2C to GPS
#include "SparkFun_Ublox_Arduino_Library.h"
SFE_UBLOX_GPS myGPS;
#include <LiquidCrystal.h>
LiquidCrystal lcd(10, 11, 12, 4, 5, 6, 7);
//Adafruit_LiquidCrystal lcd(0);
int backLight = 13;
const int mode1 = 52;
const int mode2 = 50;
const int mode3 = 48;
const int mode4 = 46;
const int mode5 = 44;
int mode = 0;
// mode 3 - xmit RTCM when FIXED
// mode 4 - xmit RTCM if delta sum < 5,5
// mode 5 - xmit RTCM if delta sum < 10,10

char  str[50];
int   bcount = 0;		  // RTCM line counter
int   RTCMcount = 0;
int   carrier;        // fix status, 0=no, 1=float, 2=fix
long  fixlat = 342394683;
long  fixlon = -1190687653;
//==========================================================
void setup() {
  long  lat;
  long  lng;
  long  alt;
  int   ill = 0;        // lat/lon array index
  int   i;
  int   fix = 0;            // fix quality 3 = 3D
  long  alat[LSMP];     // arrays of previous lat/lon
  long  alng[LSMP];     // to check for convergence
  int   dlat;           // sum of differences
  int   dlng;
  int   diff;

  byte  payload[1000];    // TMODE3 packet size
  ubxPacket pack;
  boolean response = false;

  pinMode(mode1, INPUT);
  pinMode(mode2, INPUT);
  pinMode(mode3, INPUT);
  pinMode(mode4, INPUT);
  pinMode(mode5, INPUT);
  digitalWrite(mode1, HIGH);
  digitalWrite(mode2, HIGH);
  digitalWrite(mode3, HIGH);
  digitalWrite(mode4, HIGH);
  digitalWrite(mode5, HIGH);
  if (digitalRead(mode1) == LOW)
    mode = 1;
  else if (digitalRead(mode2) == LOW)
    mode = 2;
  else if (digitalRead(mode2) == LOW)
    mode = 2;
  else if (digitalRead(mode3) == LOW)
    mode = 3;
  else if (digitalRead(mode4) == LOW)
    mode = 4;
  else if (digitalRead(mode5) == LOW)
    mode = 5;

  Serial.print("mode: ");
  Serial.println(mode);

  pack.payload = &payload[0];
  Serial.begin(115200);
//  Serial1.begin(38400);          // xbee
  Serial1.begin(19200);          // xbee
  //  while (!Serial); //Wait for user to open terminal
  Serial.println("NTRIP base station 200816");

  Wire.begin();

  //lcd.begin(Wire);
  pinMode(backLight, OUTPUT);
  //  lcd.begin(16, 4);
  lcd.begin(20, 4);
  lcd.clear();
  lcd.setCursor(0, 3);
  lcd.print("MultiBase ");
  lcd.print("200917");

  delay(500);

  myGPS.begin(Wire);
  if (myGPS.isConnected() == false) {
    Serial.println(F("Ublox GPS not detected. Freezing."));
    lcd.setCursor(0, 1);
    lcd.print(F("No GPS detected"));
    while (1);
  }

  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  lcd.setCursor(0, 0);
  lcd.print("GPS Detected");

  // clean the slate
  myGPS.disableSurveyMode();
  response &= myGPS.disableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C);
  response &= myGPS.disableRTCMmessage(UBX_RTCM_1074, COM_PORT_I2C);
  response &= myGPS.disableRTCMmessage(UBX_RTCM_1084, COM_PORT_I2C);
  response &= myGPS.disableRTCMmessage(UBX_RTCM_1094, COM_PORT_I2C);
  response &= myGPS.disableRTCMmessage(UBX_RTCM_1124, COM_PORT_I2C);
  response &= myGPS.disableRTCMmessage(UBX_RTCM_1230, COM_PORT_I2C);

  while (fix < 3) {
    fix = myGPS.getFixType();
    Serial.print("fix status: ");
    Serial.println(fix);
    delay(1000);
  }
  lcd.setCursor(0, 0);
  lcd.print("Fix at 3D or better");

  // prefill lat/lon array
  for (i = 0; i < LSMP; i++) {
    lat = myGPS.getLatitude();
    lng = myGPS.getLongitude();
    alat[i] = lat;
    alng[i] = lng;
  }
  Serial.println("type x to accept fix, or f for prefixed");
  while (1) {
    lat = myGPS.getLatitude();
    lng = myGPS.getLongitude();

    dlat = 0;
    dlng = 0;
    for (i = 0; i < LSMP; i++) {
      diff = (alat[i] - lat);           // workaround: something funny about arduino abs()
      dlat = (diff  > 0) ? (dlat + diff) : (dlat - diff);
      diff = (alng[i] - lng);
      dlng = (diff > 0) ? (dlng + diff) : (dlng - diff);
    }
    alat[ill] = lat;
    alng[ill] = lng;
    ill++;
    if (ill >= LSMP)
      ill = 0;

    sprintf(str, "lat/lon: %ld/%ld del: %d,%d", lat, lng, dlat, dlng );
    Serial.println(str);
    if (Serial.available()) {
      byte incomming = Serial.read();
      if (incomming == 'x') {
        Serial.println("Survey terminated");
        break;
      }
      if (incomming == 'f') {
        lat = fixlat;
        lng = fixlon;
        Serial.println("Canned location set as fixed");
        break;
      }
    }

    if (dlat < 10 && dlng < 10 && mode == 5)     // arbitrary criterion for sum of differences
      break;
    if (dlat < 5 && dlng < 5 && mode == 4)     // arbitrary criterion for sum of differences
      break;
    carrier = myGPS.getCarrierSolutionType();
    if (carrier == 2 && mode == 3) {
      Serial.println("Carrier indicated FIXED");
      break;
    }
    lcd.setCursor(0, 1);
    lcd.print("deltas: ");
    lcd.print(dlat);
    lcd.print("/");
    lcd.print(dlng);
    lcd.print("    ");
    delay(1000);
  }
  lcd.setCursor(0, 1);
  lcd.print("                      ");
  lcd.setCursor(0, 0);
  lcd.print("Precise fix"           );

  if (mode != 3) {
    alt = myGPS.getAltitude() / 10;  // alt in cm's
  
    // N.B. beware deprication to VALSET
    pack.cls = UBX_CLASS_CFG;
    pack.id = UBX_CFG_TMODE3;
    pack.len = 40;
    pack.startingSpot = 0;
    for (int i = 0; i < 40; i++)
      payload[i] = 0;
    payload[2] = 2;       // FIXED mode
    payload[3] = 1;       // pos in lat/lon/alt
    payload[4] = lat & 0xff;
    payload[5] = lat >> 8;
    payload[6] = lat >> 16;
    payload[7] = lat >> 24;
    payload[8] = lng & 0xff;
    payload[9] = lng >> 8;
    payload[10] = lng >> 16;
    payload[11] = lng >> 24;
    payload[12] = alt & 0xff;
    payload[13] = alt >> 8;
    payload[14] = alt >> 16;
    payload[15] = alt >> 24;
    pack.payload = &payload[0];
  
  //  myGPS.enableDebugging();
    response = myGPS.sendCommand(&pack);
    myGPS.disableDebugging();
  }
  if (response == true)
    Serial.println(F("Survey valid!"));
  Serial.println("TMODE3 set");
  //   while (1);


  // -------------------------------------------------------------------------------------
  i = 3;                    // try three times
  while (i-- > 0) {
    response = true;
    response &= myGPS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C, 1);
    response &= myGPS.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_I2C, 1);
    response &= myGPS.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_I2C, 1);
    response &= myGPS.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_I2C, 1);
    response &= myGPS.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_I2C, 1);
    response &= myGPS.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_I2C, 10);

    if (response == true) {
      Serial.println(F("RTCM messages enabled"));
      break;
    }
    else {
      Serial.println(F("RTCM failed to enable."));
      lcd.setCursor(0, 2);
      lcd.print("RTCM not enabled");
      delay(1000);
      if (i == 0)
        while (1); //Freeze
    }
  }

  Serial.println(F("Base survey complete! RTCM now broadcasting."));
  myGPS.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3);
  lcd.clear();
  lcd.print(F("Xmit RTCM messages"));
}

void loop() {
  myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

  delay(250); //Don't pound too hard on the I2C bus
  carrier = myGPS.getCarrierSolutionType();
}

//==========================================================
void SFE_UBLOX_GPS::processRTCM(uint8_t incoming)
{
  //Let's just pretty-print the HEX values for now
  /*  if (bcount % 16 == 0) Serial.println();
    Serial.print(" ");
    if (incoming < 0x10) Serial.print("0");
    Serial.print(incoming, HEX);
    bcount++;
  */
  Serial1.write(incoming);      // send RTCM byte to rover

  if (incoming == 0xd3) {
    RTCMcount++;
    if (RTCMcount % 10 == 0) {
      lcd.setCursor(0, 1);
      lcd.print("RTCM msg: ");
      lcd.print((String)RTCMcount);
      Serial.print("RTCM msg: ");
      Serial.println((String)RTCMcount);
      lcd.setCursor(0, 2);
      lcd.print("Carrier: ");
      lcd.print((String)carrier);
      Serial.print("Carrier: ");
      Serial.println((String)carrier);


    }
  }
}

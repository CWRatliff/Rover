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
//#include <SerLCD.h>
#include "SparkFun_Ublox_Arduino_Library.h"
SFE_UBLOX_GPS myGPS;
//SerLCD lcd;
#include <LiquidCrystal.h>
LiquidCrystal lcd(10, 11, 12, 4, 5, 6, 7);
//Adafruit_LiquidCrystal lcd(0);
int backLight = 13;

char  str[50];
int   bcount = 0;		  // RTCM line counter
int   RTCMcount = 0;
int   carrier;        // fix status, 0=no, 1=float, 2=fix
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

  pack.payload = &payload[0];
  Serial.begin(115200);
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
  lcd.print("NTRIPsurvey ");
  lcd.print("200816");

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
  lcd.print("Fix better than 3D");

  // prefill lat/lon array
  for (i = 0; i < LSMP; i++) {
    lat = myGPS.getLatitude();
    lng = myGPS.getLongitude();
    alat[i] = lat;
    alng[i] = lng;
  }
  while (1) {
    lat = myGPS.getLatitude();
    lng = myGPS.getLongitude();

    dlat = 0;
    dlng = 0;
    for (i = 0; i < LSMP; i++) {
      diff = (alat[i] - lat);           // something funny abour arduino abs()
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
    if (dlat < 10 && dlng < 10)     // arbitrary criterion for sum of differences
      break;
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

  myGPS.enableDebugging();
  response = myGPS.sendCommand(pack);
  myGPS.disableDebugging();
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

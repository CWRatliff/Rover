/*
  Send UBX binary commands to enable RTCM sentences on Ublox ZED-F9P module
  By: Nathan Seidle
  SparkFun Electronics
  Date: September 7th, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example does all steps to configure and enable a ZED-F9P as a base station:
    Begin Survey-In
    Once we've achieved 2m accuracy and 300s have passed, survey is complete
    Enable six RTCM messages
    Begin outputting RTCM bytes
*/
#include <Wire.h> //Needed for I2C to GPS
#include <SerLCD.h>
#include "SparkFun_Ublox_Arduino_Library.h"
SFE_UBLOX_GPS myGPS;
SerLCD lcd;
int bcount = 0;		// RTCM message counter
//==========================================================
void setup() {
  Serial.begin(115200);
  Serial1.begin(19200);          // xbee
//  while (!Serial); //Wait for user to open terminal
  Serial.println("Ublox GPS I2C Test");

  Wire.begin();

  lcd.begin(Wire);
  lcd.clear();
//  lcd.setBacklight(0, 0, 0);
  lcd.setCursor(0, 3);
  lcd.print("RTKsurvey ");
//  lcd.setCursor(0, 1);
  lcd.print("200627");

  delay(500);

  myGPS.begin(Wire);
  if (myGPS.isConnected() == false) {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    lcd.setCursor(0, 1);
    lcd.print(F("No GPS detected"));
    while (1);
	}

  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  lcd.setCursor(0, 0);
  lcd.print("GPS Detected");

  //Check if Survey is in Progress before initiating one
  boolean response;
  response = myGPS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (request can take a long time)
  if (response == false) {
    Serial.println(F("Failed to get Survey In status"));
    while (1); //Freeze
	  }

  if (myGPS.svin.active == true) {
    Serial.print(F("Survey already in progress."));
    lcd.setCursor(0, 0);
    lcd.print(F("Survey started"));
	}
  else {
    //Start survey
    response = myGPS.enableSurveyMode(300, 2.000); //Enable Survey in, 300 seconds, 2.0m
    if (response == false) {
      Serial.println(F("Survey start failed"));
      lcd.setCursor(0, 1);
      lcd.print(F("Survey start failed"));
      while (1);
	  }
    Serial.println(F("Survey started. This will run until 300s has passed and less than 2m accuracy is achieved."));
	}

  while (Serial.available()) Serial.read(); //Clear buffer

//  lcd.clear();
  lcd.print(F("Survey in progress"));

  //Begin waiting for survey to complete
  while (myGPS.svin.valid == false) {
    if (Serial.available()) {
      byte incoming = Serial.read();
      if (incoming == 'x') {
        //Stop survey mode
        response = myGPS.disableSurveyMode(); //Disable survey
        Serial.println(F("Survey stopped"));
        break;
		}
	 }

    response = myGPS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (req can take a long time)
    if (response == true) {
      Serial.print(F("Press x to end survey - "));
      Serial.print(F("Time elapsed: "));
      Serial.print((String)myGPS.svin.observationTime);

      lcd.setCursor(0, 1);
      lcd.print(F("Elapsed: "));
      lcd.print((String)myGPS.svin.observationTime);
      lcd.print("    ");

      Serial.print(F(" Accuracy: "));
      Serial.print((String)myGPS.svin.meanAccuracy);
      Serial.println();

      lcd.setCursor(0, 2);
      lcd.print(F("Accuracy: "));
      lcd.print((String)myGPS.svin.meanAccuracy);
      lcd.print("   ");
	  }
    else {
      Serial.println(F("SVIN request failed"));
	  }

    delay(1000);
	}
  Serial.println(F("Survey valid!"));

// ------------------------------------------------------------------------------------- 
  response = true;
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_I2C, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_I2C, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_I2C, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_I2C, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_I2C, 10);

  if (response == true) {
    Serial.println(F("RTCM messages enabled"));
	}
  else {
    Serial.println(F("RTCM failed to enable. Sure you have an ZED-F9P?"));
    while (1); //Freeze
	}

  Serial.println(F("Base survey complete! RTCM now broadcasting."));
  myGPS.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3);
  lcd.clear();
//  lcd.setBacklight(0, 255, 0);
  lcd.print(F("Xmit RTCM messages"));
  
  }

void loop() {
  myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.
  
  delay(250); //Don't pound too hard on the I2C bus
  }

//==========================================================
void SFE_UBLOX_GPS::processRTCM(uint8_t incoming)
{
  //Let's just pretty-print the HEX values for now
  if (bcount % 16 == 0) Serial.println();
  Serial.print(" ");
  if (incoming < 0x10) Serial.print("0");
  Serial.print(incoming, HEX);
  bcount++;

  Serial1.write(incoming);
}

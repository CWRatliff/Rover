//191019 - revised lat/long arithmetic to avoid rounding loss of accuracy
//200216 - defined lat/long biases for ease of location change
//200608 - continue instead of break in xbee input
//200612 - revised RTCM processing
//200615 - working on heading responsiveness
//200630 - removed RTCM stream to separate xbee/arduino

#define FALSE 0
#define TRUE 1
#define LCRYS
//#define ADA
//#define SPARK
//#define LTBIAS  342333333     // VdM 34d 14m
//#define LNBIAS  1190666666    // 119d 4m
#define LTBIAS 342166666   // Aven Navidad 34d 13m
#define LNBIAS 1190000000   // 119d 0m

#include <Wire.h>
#include <math.h>

#include <LiquidCrystal.h>
LiquidCrystal lcd(10, 11, 12, 4, 5, 6, 7);

#include "SparkFun_Ublox_Arduino_Library.h"

SFE_UBLOX_GPS myGPS;

unsigned long gpsepoch;
char    str[25];
byte    gps;
int     hdg;
int     oldhdg = 0;
long    latitude;
long    longitude;
int     msgcnt = 0;

//=======================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("GPS Mobile Rover");

  Wire.begin();
  lcd.begin(20, 4);

  lcd.clear();
  lcd.setCursor(0, 3);
  lcd.print("GPSrover ");
  lcd.print("200807");

  myGPS.begin();
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
  gpsepoch = millis();
}
//============================================================
void loop() {
  char  xchr;


  if ((millis() - gpsepoch) < 1000)
    return;
  latitude = myGPS.getLatitude();
  longitude = myGPS.getLongitude();

  //  lat and long are multiplied by 1e7 to avoid double precision
  //  fixed point lat = (deg)*e7 + (min)*e7/60 + (sec)*e7/3600
  //  reorder to delay double till last operation

  long llatsec = (latitude - LTBIAS) * 36;
  double latsec = (double)llatsec / 100000.0;

  long llongsec = (longitude + LNBIAS) * 36;  // W.Lon will be minus
  double lonsec = (double)llongsec / 100000.0;

  lonsec = fabs(lonsec);
  latsec = fabs(latsec);

  char latstr[15];
  dtostrf(latsec, 7, 4, latstr);
  sprintf(str, "lat: %s ", latstr);
  lcd.setCursor(0, 0);
  lcd.print(str);

  char lonstr[15];
  dtostrf(lonsec, 6, 4, lonstr);
  sprintf(str, "lon: %s ", lonstr);
  lcd.setCursor(0, 1);
  lcd.print(str);

  long altitude = myGPS.getAltitudeMSL();
  sprintf(str, "alt:%ld ", altitude);
  lcd.setCursor(0, 2);
  lcd.print(str);

  /*    long accuracy = myGPS.getPositionAccuracy();
      sprintf(str, "V%ld ", accuracy);
      Serial.println(str);
      gpsphase = 5;
      return;
      }*/
  //  long accuracy = myGPS.getPositionAccuracy();
  long accuracy = myGPS.getHorizontalAccuracy() / 10;
  if (accuracy > 0) {
    sprintf(str, "acc: %ld ", accuracy);
    lcd.setCursor(11, 2);
    lcd.print(str);
  }

  gpsepoch = millis();
}

//191019 - revised lat/long arithmetic to avoid rounding loss of accuracy
//200216 - defined lat/long biases for ease of location change
//200608 - continue instead of break in xbee input
//200612 - revised RTCM processing
//200615 - working on heading responsiveness
//200630 - removed RTCM stream to separate xbee/arduino
//201211 - commented out IMU auto calibrate
//201227 - used lib yaw func, declination done here

#define FALSE 0
#define TRUE 1
#define LTBIAS  342333333     // VdM 34d 14m
#define LNBIAS  1190666666    // 119d 4m
//#define LTBIAS 342166666   // Aven Navidad 34d 13m
//#define LNBIAS 1190000000   // 119d 0m
#define DECLINATION 12        // for Camarillo

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <math.h>
#include "SparkFun_Ublox_Arduino_Library.h"

SFE_UBLOX_GPS myGPS;
BNO080 myIMU;

unsigned long gpsepoch;
char    str[25];
byte    gps;
int     hdg;
int     oldhdg = 0;
int     gpsphase = 0;
long    latitude;
long    longitude;
int     msgcnt = 0;
//============================================================
// BNO080
void CheckIMU() {
  double yaw;

  if (myIMU.dataAvailable() == true) {
    yaw = myIMU.getYaw() * 180 / M_PI;

    hdg = -yaw;     // chg from CCW to CW
    hdg += DECLINATION;
                    // 90deg between x, y is by rotation of BNO chip
    Serial.print("raw yaw: ");
    Serial.println(yaw);
                    
    if (hdg < 0)
      hdg += 360;
    if (hdg != oldhdg) {
      sprintf(str, "{O%d}", hdg);
      Serial2.write(str);
      Serial.println(str);
      oldhdg = hdg;
    }
  }
}

//=======================================================================
void setup() {
  boolean imu;
  boolean gps;
  Serial.begin(115200);
  Serial.println("RaspardHdgSpeedup 201211");
  Serial1.begin(9600);        // XBee
  Serial2.begin(9600);        // RPi USB/ttl


  Wire.begin();
  Wire.setClock(400000);
  for (int i = 0; i < 3; i++) {
    imu = myIMU.begin();
    if (imu)
      break;
    }
  if (imu == FALSE)
    Serial2.write("{T3}");

//  myIMU.calibrateAll();           // 5 min auto calibrate
  myIMU.enableRotationVector(50); //Send data update every 50ms

  for (int i = 0; i < 3; i++) {
    gps = myGPS.begin();
    if (gps)
      break;
    }
  if (gps == FALSE)
    Serial2.write("{T4}");
    
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX
//  myGPS.setNavigationFrequency(10); // 10 Hz
//  myGPS.setNavigationFrequency(4); // 4 Hz, 10 Hz seems to reduce accuracy
//  myGPS.setNavigationFrequency(2); // 2 Hz, 4 & 10 Hz seems to reduce accuracy
  myGPS.setNavigationFrequency(1); // 2 Hz, 4 & 10 Hz seems to reduce accuracy
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
  gpsepoch = millis();
}
//============================================================
void loop() {
  char  xchr;

  // read Xbee input and upload to RPi
  while (Serial1.available()) {
    xchr = Serial1.read();
    Serial2.write(xchr);
    Serial.print(xchr);
    if (xchr == '}') {
      Serial.println();
      continue;  //break;
      }
    } // endwhile

  // read any msg from RPi and send via XBee
  while (Serial2.available()) {
    xchr = Serial2.read();
    Serial1.write(xchr);
    Serial.print(xchr);
    if (xchr == '}') {
      Serial.println();
      continue; //break;
      }
    }
    
  CheckIMU();

  if ((millis() - gpsepoch) < 1000)
    return;

  if (gpsphase == 0) {
    latitude = myGPS.getLatitude();
    gpsphase = 1;
    return;
    }
  if (gpsphase == 1) {
    longitude = myGPS.getLongitude();
    gpsphase = 2;
    return;
    }
  if (gpsphase == 2) {

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
    sprintf(str, "{LT%s}", latstr);
    Serial2.write(str);
    Serial.println(str);

    char lonstr[15];
    dtostrf(lonsec, 6, 4, lonstr);
    sprintf(str, "{LN%s}", lonstr);
    Serial2.write(str);
    Serial.println(str);
    
    gpsphase = 3;
    return;
  }

  if (gpsphase == 3) {
    long altitude = myGPS.getAltitudeMSL();
    sprintf(str, "{LH%ld}", altitude);
    Serial.println(str);
    gpsphase = 4;
    return;
    }
  if (gpsphase == 4) {
/*    long accuracy = myGPS.getVerticalAccuracy();
    sprintf(str, "{LV%ld}", accuracy);
    Serial.println(str);
    gpsphase = 5;
    return;
    }
  // else if gpsphase == 5
*/
    long accuracy = myGPS.getHorizontalAccuracy() / 10;
    if (accuracy > 0) {
      sprintf(str, "{LA%ld}", accuracy);
      Serial2.write(str);
      Serial.println(str);
      }
    }
  gpsepoch = millis();
  gpsphase = 0;
  }

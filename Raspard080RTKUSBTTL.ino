//190823 - added '.'s to msgs - seems to be SPI timing need
//191019 - revised lat/long arithmetic to avoid rounding loss of accuracy
//200216 - defined lat/long biases for ease of location change
//200608 - continue instead of break in xbee input

#define FALSE 0
#define TRUE 1
#define LTBIAS  342333333     // VdM 34d 14m
#define LNBIAS  1190666666    // 119d 4m
//#define LTBIAS 342166666   // Aven Navidad 34d 13m
//#define LNBIAS 1190000000   // 119d 0m

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <math.h>
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS

SFE_UBLOX_GPS myGPS;
BNO080 myIMU;

unsigned long imuepoch;
unsigned long gpsepoch;
char    str[25];
byte    gps;
int     oldhdg = 0;

//==========================================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Raspard080RTKUSBttl");
  Serial1.begin(9600);        // XBee
  Serial2.begin(9600);        // RPi USB/ttl
  Serial3.begin(38400);       // RTK RCTM

  Wire.begin();
  myIMU.begin();
  myIMU.enableRotationVector(50); //Send data update every 50ms
  myGPS.begin();
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR

  }
//============================================================
void loop() {
  char* p;
  char  xchr;
  float qx;
  float qy;
  float qz;
  float qw;
  float quatRadianAccuracy;
  double sinr_cosp;
  double cosr_cosp;
  double roll;
  double sinp;
  double pitch;
  double siny_cosp;
  double cosy_cosp;  
  double yaw;
  double trash;
  int hdg;
  
  // read Xbee input and upload to RPi
  while (Serial1.available()) {
    xchr = Serial1.read();
    if (xchr & 0x80) {
      if ((xchr & 0xb0) == 0xb0) {
        gps = (xchr & 0x0f) << 4;
        continue;
        }
      if ((xchr & 0xd0) == 0xd0) {
        gps |= (xchr & 0x0f);
        Serial3.write(gps);
        continue;
        }
      }
    Serial2.write(xchr);
    Serial.print(xchr);
    if (xchr == '}') {
      Serial.println();
      break;
      }
    } // endwhile

  // read any msg from RPi and send via XBee
  while (Serial2.available()) {
    xchr = Serial2.read();
    Serial1.write(xchr);
    Serial.print(xchr);
    if (xchr == '}') {
      Serial.println();
      break;
      }
    }
      
//  if ((millis() - imuepoch) > 250) {
//    imuepoch = millis();
    
    if (myIMU.dataAvailable() == true) {
      qx = myIMU.getQuatI();
      qy = myIMU.getQuatJ();
      qz = myIMU.getQuatK();
      qw = myIMU.getQuatReal();
  //    quatRadianAccuracy = myIMU.getQuatRadianAccuracy();
  /*
      Serial.print(qx, 3);
      Serial.print(F(","));
      Serial.print(qy, 3);
      Serial.print(F(","));
      Serial.print(qz, 3);
      Serial.print(F(","));
      Serial.print(qw, 3);
      Serial.print(F(","));
  //    Serial.print(quatRadianAccuracy, 2);
      Serial.println();
  */  
      // convert quaternian to Euler angles
      // roll (x-axis rotation)
      sinr_cosp = +2.0 * (qw * qx + qy * qz);
      cosr_cosp = +1.0 - 2.0 * (qx * qx + qy * qy);
      roll = atan2(sinr_cosp, cosr_cosp);
    
      // pitch (y-axis rotation)
      sinp = +2.0 * (qw * qy - qz * qx);
      if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
      else
        pitch = asin(sinp);
    
      // yaw (z-axis rotation)
      siny_cosp = +2.0 * (qw * qz + qx * qy);
      cosy_cosp  = +1.0 - 2.0 * (qy * qy + qz * qz);  
      yaw = atan2(siny_cosp, cosy_cosp) * 180/M_PI;

      hdg = -yaw;
      if (hdg < 0)
        hdg += 360;
      Serial.print("Yaw = ");
      Serial.print(yaw);
      Serial.print(" heading = ");
      Serial.println(hdg);
      // compose 'O'rientation msg for Pi
//      hdg = 450 - yaw;                  // cvt math CCW x-axis to azimuth
//      hdg = hdg % 360;
      }

    if (hdg != oldhdg) {
      sprintf(str, "{O%d}", hdg);
      Serial2.write(str);
      Serial.println(str);
      oldhdg = hdg;
      }
//    }
/*
      Serial.print("yaw:");
      Serial.print(yaw);
      Serial.print(" pitch: ");
      Serial.print(pitch*180.0/M_PI, 2);
      Serial.print(" roll: ");
      Serial.print(roll*180.0/M_PI, 2);
      Serial.println(str);
*/
    if ((millis() - gpsepoch) < 1000)
      return;
    gpsepoch = millis();

    long latitude = myGPS.getLatitude();
    long longitude = myGPS.getLongitude();

      
    //  lat and long are multiplied by 1e7 to avoid double precision
    //  fixed point lat = (deg)*e7 + (min)*e7/60 + (sec)*e7/3600
    //  reorder to delay double till last operation

    long llatsec = (latitude - LTBIAS) * 36;
    double latsec = (double)llatsec / 100000.0;

    long llongsec = (longitude + LNBIAS) * 36;  // long will be minus
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

    long accuracy = myGPS.getPositionAccuracy();
    if (accuracy > 0) {
      sprintf(str, "{LA%ld}", accuracy);
      Serial2.write(str);
      Serial.println(str);
      }
    }

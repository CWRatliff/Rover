/*


  Note: Long/lat are large numbers because they are * 10^7. To convert lat/long
  to something google maps understands simply divide the numbers by 10,000,000. We 
  do this so that we don't have to use floating point numbers.

  Leave NMEA parsing behind. Now you can simply ask the module for the datums you want!

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106
*/
#include <stdlib.h>
#include <Wire.h> //Needed for I2C to GPS

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;
#define DELAY 2000

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.
double latdel;
double londel;
char  ibuffer[50];
int   bndx = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun Ublox Example");

  Wire.begin();
  Serial1.begin(9600);

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
}

void loop()
{
  double trash;
  char    buff[30];
  int     cin;
  char*    cend;
  
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > DELAY) {
    lastTime = millis(); //Update the timer
    
    long latitude = myGPS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = myGPS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (deg)"));

    long altitude = myGPS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    byte SIV = myGPS.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);

    long acc = myGPS.getPositionAccuracy();
    Serial.print(" acc: ");
    Serial.print(acc);

    double latdeg = (double)latitude / 10000000.0;
    double latmin = modf(latdeg, &trash);
    double latsec = modf(latmin * 60, &trash);
    double londeg = (double)longitude / 10000000.0;
    double lonmin = modf(londeg, &trash);
    double lonsec = modf(lonmin * 60, &trash);
    lonsec = fabs(lonsec);

    while (Serial1.available()) {
      cin = Serial1.read();
      if (cin == '{')
        bndx = 0;
      ibuffer[bndx++] = cin;
      ibuffer[bndx] = '\0';

      if (cin == '}') { 
        if (ibuffer[1] == 'L') {
          if (ibuffer[2] == 'a')
            latdel = strtod(&ibuffer[3], &cend);
          else
            londel = strtod(&ibuffer[3], &cend);
          }
        }
      }

    double latcorr = latsec - latdel;
    double loncorr = lonsec - londel;

    Serial.println();

    Serial.print("Raw lat/lon: ");
    Serial.print(latsec, 4);
    Serial.print("/");
    Serial.print(lonsec, 4);
    Serial.print(" Corr lat/lon: ");
    Serial.print(latcorr);
    Serial.print("/");
    Serial.print(loncorr);
    }
}

//191019 - revised lat/long arithmetic to avoid rounding loss of accuracy
//200216 - defined lat/long biases for ease of location change
//200608 - continue instead of break in xbee input
//200612 - revised RTCM processing
//200615 - working on heading responsiveness
//200630 - removed RTCM stream to separate xbee/arduino
//201211 - commented out IMU auto calibrate
//201227 - used lib yaw func, declination done here
//210109 - using gyro - quats only, no mag
//210918 - added lidar sensor
//210924 - switched to ada servo controller due to servo interferrenceground hits
//210930 tilt bias 245->255 to lessen

#define FALSE 0
#define TRUE 1
#define LTBIAS  342333333     // VdM 34d 14m
#define LNBIAS  1190666666    // 119d 4m
//#define LTBIAS 342166666   // Aven Navidad 34d 13m
//#define LNBIAS 1190000000   // 119d 0m
#define DECLINATION 12        // for Camarillo
#define  MINSTR    3000       // min lidar strength
#define MAXDIST   20.0        // max lidar range
#define OFFSET    3.0         // halfwidth of freeway needed

#include <math.h>
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include "SparkFun_Ublox_Arduino_Library.h"
#include <Adafruit_PWMServoDriver.h>

SFE_UBLOX_GPS myGPS;
BNO080 myIMU;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
int pan = 0;
int tilt = 1;


int       strength;
int       distance;
boolean   receiveComplete = false;

unsigned long gpsepoch;   // gps interval
unsigned long epoch;      // lidar interval
char    str[25];
int     hdg;
int     oldhdg = 0;
int     gpsphase = 0;
long    latitude;
long    longitude;
int     aim = -20;        // pan angle
int     delta = 1;        // servo backlash
int     elevation = 6;    // tilt angle (on rover)
int     pitch;
int     pbias;
int     swath;
//int   bias = 94;			// pan servo adjustment
//int   tbias = 96;			// tilt servo adjustment
// bias = 300				// test article
// tbias = 305
int     bias = 280;       // pan servo adjustment
int     tbias = 265;      // tilt servo adjustment
boolean obstacle = false; // obstacle marked?
boolean lidarflag = false;    // to scan or not to scan
float   blipd[42];      // 41 + delta, L to R
int		  blips[42];
char    cmdbuff[25];
int     cmdndx = 0;

//============================================================
// BNO080

void CheckIMU() {
  double yaw;

  if (myIMU.dataAvailable() == true) {
    yaw = myIMU.getYaw() * 180 / M_PI;

    hdg = -yaw;     // chg from CCW to CW
    hdg += DECLINATION;
    //    Serial.print("raw yaw: ");
    //    Serial.println(yaw);

    if (hdg < 0)
      hdg += 360;
    if (hdg != oldhdg) {
      sprintf(str, "{O%d}", hdg);
      Serial2.write(str);
      Serial.println(str);
      oldhdg = hdg;
    }
    pitch = -((myIMU.getRoll() * 180 / M_PI) - pbias);     // rover mounting 90deg about z axis
  }
}
// ==========================================================
//  ISR for TSmini

void getTFminiData(int* distance, int* strength, boolean* complete) {
  static char i = 0;          // continuity
  char j = 0;
  int checksum = 0;
  static int rx[9];

  while (Serial3.available()) {      // read all bytes available
    rx[i] = Serial3.read();
    if (rx[0] != 0x59)
      i = 0;
    else if (i == 1 && rx[1] != 0x59)
      i = 0;
    else if (i == 8) {
      for (j = 0; j < 8; j++)
        checksum += rx[j];
      if (rx[8] == (checksum % 256)) {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
        *complete = true;
      }
      i = 0;
    }
    else
      i++;
  }
}
//==================================================================
// analyze blip line for obstacles & xmit
// N.B. some remaining redundancy with lidar() wrt signal qualification

int lineanal() {
  int  starta = 0;
  int lasta = 0;
  int stesta;
  int ltesta;
  float odist;

  float minimumd;
  float mindist = MAXDIST;
  //  int dis;
  long sigstrength;
  int swath = 0;
  int testswath;
  long ttlstrength;
  long cumstr;
  float  alpha;      // angle to blip radians
  float dist;
  boolean tracking = false;

  dumpstr();
  xmit("Z2", (double)pitch, elevation, 0);
  for (int aim = -20; aim <= 20; aim++) { // see servo rotation note above
    dist = blipd[aim + 20];
    sigstrength = blips[aim + 20];

    //    if (strength > MINSTR && offset < OFFSET) {
    if (sigstrength > 10000)
      sigstrength = 10000;
    if (sigstrength > MINSTR && dist > 5.0) {

      if (!tracking) {					// maybe start new track
        if (dist < mindist) {			// closer than last track?
          tracking = true;				// yes, start a new tracking
          stesta = aim;
          testswath = 0;
          mindist = dist;
          ttlstrength = sigstrength;
        }
      }
      else {							// continue tracking
        testswath++;
        ltesta = aim;
        mindist = min(dist, mindist);
        ttlstrength += sigstrength;
      }
    }  // if strong signal

    else if (tracking) {				// was tracking, but lost strength or offset
      tracking = false;
      if (testswath >= 3) {
        swath = testswath;				// new best guess
        starta = stesta;
        lasta = ltesta;
        odist = mindist;
        cumstr = ttlstrength;
      }
    }
  } // end of -20 to 20 sweep

  if (swath >= 3) {						// any wide enough obstruction?
    int side = (lasta + starta) / 2;    // less than 0 -> Port
    if (side < 0)
      xmit("S1P", odist, lasta, swath);
    else // (side >= 0)
      xmit("S1S", odist, starta, swath);
    sigstrength = cumstr / (swath + 1);
    //	xmit("S1", odist, starta, lasta);
//    xmit("Z1", (double)sigstrength, swath);
    obstacle = true;					// {S1...} obstacle message sent

  } // if swath
}
//================================================================
void dumpstr() {
  char sline[43];
  int blip;

  for (int i = 0; i < 42; i++) {
    blip = blips[i];
    if (blipd[i] <= 3)
      sline[i] = ' ';
    else if (blip > 6000)
      sline[i] = '@';
    else if (blip > 5000)
      sline[i] = '#';
    else if (blip > 4000)
      sline[i] = 'X';
    else if (blip > 3750)
      sline[i] = 'x';
    else if (blip > 3500)
      sline[i] = '+';
    else if (blip > 3250)
      sline[i] = '-';
    else if (blip > 3000)
      sline[i] = '.';
    else
      sline[i] = ' ';
    sline[42] = '\0';
  }
  Serial2.write("{Z3 |");
  Serial2.write(sline);
  Serial2.write("|}");
}
//=============================================================
// save blip strength & distance in line arrays

void lidar() {
  float dist = distance / 30.48;
  float temp;


  if (strength > 500 && dist < 20.0) {  // got a healthy blip
    // save blip in array
    int i = aim + 20 - delta;
    if (i >= 0 && i < 42) {             // avoid neg subscript
      temp = strength * dist / 10.0;    // normalize signal strength to 10 ft
      blips[i] = temp;
      blipd[i] = dist; // exp. determined
    }
  }
}
//=============================================================
// send message to rasb pi

void xmit(char *preamble, float value, int angle, int width) {
  char dist[15];
  char  str[40];
  dtostrf(value, 4, 1, dist);
  sprintf(str, "{%s%s,%d,%d}", preamble, dist, angle, width);
  Serial2.write(str);
  Serial.println(str);
  return;
}
//=======================================================================
// degrees to PWM (very approx)

int deg2pwm(int deg) {
  return (deg * 2);
}
//=======================================================================
void setup() {
  boolean imu;
  boolean gps;
  Serial.begin(115200);
  Serial.println("RaspardSensor 211011");
  Serial1.begin(9600);        // XBee
  Serial2.begin(9600);        // RPi USB/ttl
  Serial3.begin(115200);      // TFmini

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
  //  myIMU.enableRotationVector(50); //Send data update every 50ms
  myIMU.enableGyroIntegratedRotationVector(50); //Send data update every 50ms
  while (myIMU.dataAvailable() == false) {}
  pbias = myIMU.getRoll() * 180 / M_PI;     // rover mounting 90deg about z axis
  Serial.print("pbias: ");
  Serial.println(pitch);

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
  myGPS.setAutoPVT(TRUE);
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
  gpsepoch = millis();

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);
  //pan.attach(9);
  //pan.write(aim + bias);
  //tilt.attach(10);
  //tilt.write(elevation + tbias);
  pwm.setPWM(pan, 0, deg2pwm(-20) + bias);
  pwm.setPWM(tilt, 0, deg2pwm(0) + tbias);
  epoch = millis();

  delay(25);
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
    if (xchr == '{')
      cmdndx = 0;
    cmdbuff[cmdndx++] = xchr;
    if (xchr == '}') {
      cmdbuff[cmdndx] = '\0';
      Serial.println(cmdbuff);
      if (cmdbuff[1] == 'a') {
        if (cmdbuff[2] == 'S')    // {aStby} or {aStop}
          lidarflag = false;
        if (cmdbuff[2] == 'W')    // {aWP##}
          lidarflag = true;
      }
      continue; //break;
    }
  }

  CheckIMU();

  // lidar scan for obstructions
  if (lidarflag) {
    if ((millis() - epoch) > 50) {

      if (receiveComplete) {
        receiveComplete = false;
        lidar();
        aim += delta;
        if (aim > 20) {				// end of right sweep
          lineanal();
          for (int i = 0; i < 42; i++) {
            blipd[i] = 0;
            blips[i] = 0;
          }
          aim = 20;
          delta = -1;
          elevation += 2;
          if (elevation > 4)
            //          if (elevation > 6)
            elevation = 0;
        }
        if (aim < -20) {			// end of left sweep
          lineanal();
          for (int i = 0; i < 42; i++) {
            blipd[i] = 0;
            blips[i] = 0;
          }
          aim = -20;
          delta = 1;
          elevation += 2;
          if (elevation > 4)
            //          if (elevation > 6)
            elevation = 0;
        }
        //pan.write(aim + bias);
        //tilt.write(elevation + tbias);
        pwm.setPWM(pan, 0, deg2pwm(aim) + bias);
        pwm.setPWM(tilt, 0, deg2pwm(elevation) + tbias - deg2pwm(pitch));
      } // if receive
      epoch = millis();
    } // if epoch
  } // if lidarflag

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
    // temp fix since autoPVT doesnt include HorizAcc
    // however, PVT has mm level hAcc, but not in Sparkfun lib
    //    long accuracy = myGPS.getHorizontalAccuracy() / 10;
    long accuracy = myGPS.gethAcc();
    if (accuracy > 0) {
      sprintf(str, "{LA%ld}", accuracy);
      Serial2.write(str);
      Serial.println(str);
    }
  }
  gpsepoch = millis();
  gpsphase = 0;
}
//===================================================================
// ISR for Serial3 TFmini

void serialEvent3() {
  getTFminiData(&distance, &strength, &receiveComplete);
}

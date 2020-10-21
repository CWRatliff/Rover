//190621 - added compass command
//190701 - keypad initial code
//190714 - debounce for ZERO, STOP, COMPASS
//190726 - keypad instead of xbox
//200406 - diagnostic cmds

#include <Keypad.h>

const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {49, 47, 45, 43}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {48, 46, 44, 42}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

#include "Wire.h"
#include "Adafruit_LiquidCrystal.h"
//#include <LiquidCrystal.h>

#define FALSE       0
#define TRUE        (!FALSE)
#define BUFF        256

Adafruit_LiquidCrystal lcd(0);

//  Xbee  Mega                  // hardware UART on Mega Arduino
//  2     19 rx1
//  3     18 tx1

#define DIAGNOSTIC  'T'     // command a rover diagnostic
#define DIRECTION   'D'     // one digit basic standby commands
#define EXECUTE     'E'     // execute autopilot maneover
#define FUNCTION    'F'     // multi digit input

char  inpt;

int   diagflag = FALSE;
int   exeflag = FALSE;
int   lbflag = FALSE;
int   lbcount = 0;
char  lb1;

char  ibuffer[BUFF];              // Xbee software input buffer
int   itail = 0;
int   ihead = 0;
int   piflag = FALSE;             // input from Raspberry Pi

int   epoch;                      // last xmission time

//===============================================================
// print text numbers to lcd
char* lcdout(char *s) {
  char  c;
  while (isDigit(*s) || *s == '-' || *s == '.' || *s == ' ') {
    c = *s++;
    lcd.print(c);
    }
  return (s);
  }
//==================================================================
// print text to lcd
char* lcdtext(char *s) {
  char c;
  while (*s) {
    c = *s++;
    lcd.print(c);
    }
  return (s);
  }
//===================================================================  
void xmit(char code) {
  Serial1.print("{");
  Serial1.print(code);
  Serial1.print("}");
  epoch = millis();
  } 
//===================================================================  
void xmitnum(char code, char num) {
  Serial1.print("{");
  Serial1.print(code);
  Serial1.print(num);
  Serial1.print("}");
  epoch = millis();
  } 
//===================================================================  
void xmit2num(char code, char num, char num2) {
  Serial1.print("{");
  Serial1.print(code);
  Serial1.print(num);
  Serial1.print(num2);
  Serial1.print("}");
  epoch = millis();
  } 
//====================================================================
void setup(){
 
  Serial.begin(115200);

  lcd.begin(20, 4);
  lcd.clear();
  Serial1.begin(9600);        // Xbee on hardware port
  delay(100);
   xmit2num(FUNCTION, '9', '9');             // ping, were alive
   Serial.println("XbeeKpad 200610");

    lcd.setCursor(0, 0);
    lcd.print("Spd: ");
    lcd.setCursor(0, 1);
    lcd.print("Str: ");
    lcd.setCursor(10, 1);
    lcd.print("DTG: ");
    lcd.setCursor(0, 2);
    lcd.print("Hdg: ");
    lcd.setCursor(10, 2);
    lcd.print("CTG: ");
    lcd.setCursor(0, 3);
    lcd.print("L/L: ");

}
//=========================================================================
void loop() {

  char* str;
  int   spd;
  int   ang;


   //-----------------------------------------Xbee input from bot
   while (Serial1.available()) {
    inpt = Serial1.read();
    if (inpt == '{') {
      ihead = 0;
      continue;
      }
    if (inpt == '}') {
      ibuffer[ihead] = '\0';
      piflag = TRUE;
      break;
      }
    ibuffer[ihead++] = inpt;
    }
  if (piflag) {
    Serial.println(ibuffer);
    str = &ibuffer[1];

    if (ibuffer[0] == 'v') {
      lcd.setCursor(5, 0);
      lcdout(str);                        // print speed
      }
    if (ibuffer[0] == 's') {
      lcd.setCursor(5, 1);
      lcdout(str);                        // print steering
      }
    if (ibuffer[0] == 'h') {
      lcd.setCursor(5, 2);
      lcdout(str);                        // print heading
      }
    if (ibuffer[0] == 'd') {
      lcd.setCursor(15, 1);
      lcdout(str);                        // print distance to wpt
      }
    if (ibuffer[0] == 'c') {
      lcd.setCursor(15, 2);
      lcdout(str);                        // print course to wpt
      }
    if (ibuffer[0] == 'a') {              // auto/stby status
      lcd.setCursor(16, 0);
      lcdtext(str);
      }
    if (ibuffer[0] == 'l') {
      str = &ibuffer[2];
      if (ibuffer[1] == 't') {
        lcd.setCursor(5, 3);
        lcdout(str);
        }
      if (ibuffer[1] == 'n') {
        lcd.setCursor(12, 3);
        lcdout(str);
        }
      }

    piflag = FALSE;
    }

    //-----------------------------------------------keypad inputs
    char kpad = keypad.getKey();
    if (kpad) {
      Serial.print(kpad);

      if (kpad == '*') {
        diagflag = FALSE;
        exeflag = TRUE;
        lbflag = FALSE;
        }
        
      else if (kpad == '#') {
        diagflag = FALSE;
        lbflag = TRUE;
        lbcount = 0;
        exeflag = FALSE;
        Serial.print("hatch found");
        }

      else if (kpad == 'D') {
        diagflag = TRUE;
        exeflag = FALSE;
        lbflag = FALSE;
        }
      if (kpad >= '0' && kpad <= '9') {        
        if (exeflag && kpad >= '0' && kpad <= '9') {
          xmitnum(EXECUTE, kpad);
          exeflag = FALSE;
          }
          
        else if (diagflag && kpad >= '0' && kpad <= '9') {
          xmitnum(DIAGNOSTIC, kpad);
          diagflag = FALSE;
          }
          
        else if (lbflag && kpad >= '0' && kpad <= '9') {
          if (lbcount == 0) {
            lb1 = kpad;
            lbcount++;
            }
          else if (lbcount == 1) {
            xmit2num(FUNCTION, lb1, kpad);
            lbflag = FALSE;
            }
          }
  
        else {
          Serial.print("bare key");
          xmitnum(DIRECTION, kpad);
          }
        }

//    if ((millis() - epoch) > 1000) {}
//      xmit('*');                          // were still alive
//    epoch = millis();
      
  delay(50);  
    }
  }

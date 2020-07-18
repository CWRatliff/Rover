
//============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("ArdRTCMbin");
  Serial1.begin(19200);        // XBee
  Serial3.begin(38400);       // RTK RCTM

  }
//============================================================
void loop() {
  char  xchr;

  // read Xbee input and upload to RPi
  while (Serial1.available()) {
    xchr = Serial1.read();
    Serial3.write(xchr);

    } // endwhile
  }

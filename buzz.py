#07_04_temp_final.py

import RPi.GPIO as GPIO
import time
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.OUT)

while (True):
    GPIO.output(21, GPIO.HIGH)
    time.sleep(.001)
    GPIO.output(21, GPIO.LOW)
    time.sleep(.001)


 

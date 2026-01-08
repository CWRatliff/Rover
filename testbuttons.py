'''
Run on controller box to test buttons
'''


import RPi.GPIO as GPIO
import time

GREEN = 21
BLACK = 5
RED = 13
BLUE = 7
YELLOW = 10

butngreen = False
butngreen2 = False
greenepoch = time.time()
butnred = False
butnred2 = False
redepoch = time.time()
butnblack = False
blackepoch = time.time()
butnblue = False
butnyellow = False

GPIO.setmode(GPIO.BCM)
GPIO.setup(GREEN, GPIO.IN, pull_up_down=GPIO.PUD_UP) # green
GPIO.setup(BLACK, GPIO.IN, pull_up_down=GPIO.PUD_UP) # black
GPIO.setup(RED, GPIO.IN, pull_up_down=GPIO.PUD_UP) # red
GPIO.setup(BLUE, GPIO.IN, pull_up_down=GPIO.PUD_UP) # blue
GPIO.setup(YELLOW, GPIO.IN, pull_up_down=GPIO.PUD_UP) # yellow

# check tactile buttons
while True:
    if (GPIO.input(GREEN) == False):              # button grounds out GPIO
        if not butngreen:                      # if not stale tap
            if ((time.time() - greenepoch) < .6): # if less than .6 sec
                if butngreen2:                 # if double tap in progress
                    print("green - 35 deg rightturn")
                else:
                    print("green - 5 deg right turn")
                    butngreen2 = True          # double tap started
            else:                              # else 1st tap
                print("green - 1 deg right turn") 
        butngreen = False                      # button released
            
    if (GPIO.input(BLACK) == False):
        if not butnblack:
            if ((time.time() - blackepoch) < .6): # if less than .6 sec
                print("black - all stop!")
            else:
                print("black - zero steering")
            butnblack = True
            blackepoch = time.time()           # reset
    else:
        butnblack = False
            
    if (GPIO.input(RED) == False):
        if not butnred:
            if ((time.time() - redepoch) < .6): # if less than .6 sec
                if butnred2:
                    print("red - right 35 deg steering")
                else:
                    print("red - right 5 deg steering")
                    butnred2 = True
            else:
                print("red - left 1 deg steering")
                butnred2 = False
            butnred = True
            redepoch = time.time()
    else:
        butnred = False

    if (GPIO.input(BLUE) == False):
        if not butnblue:
            print("blue - increase forward speen")
            butnblue = True
    else:
        butnblue = False
            
    if (GPIO.input(YELLOW) == False):
        if not butnyellow:
            print("yellow - decrease speed")
            butnyellow = True
    else:
        butnyellow = False
            
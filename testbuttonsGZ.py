'''
Run on controller box to test buttons
'''


import gpiozero as gz
import time

GREEN = 21
BLACK = 5
RED = 13
BLUE = 7
YELLOW = 10

butngreen2 = False
greenepoch = time.time()
butnred2 = False
redepoch = time.time()
blackepoch = time.time()

btngreen = gz.Button(GREEN)
btnblack = gz.Button(BLACK)
btnred = gz.Button(RED)
btnblue = gz.Button(BLUE)
btnyellow = gz.Button(YELLOW)

while True:
    # check tactile buttons
    if (btngreen.is_pressed):              # button grounds out GPIO
        if not butngreen:                      # if not stale tap
            if ((time.time() - greenepoch) < .6): # if less than .6 sec
                if butngreen2:                 # if double tap in progress
                    print("green - 35 deg rightturn") # 35 deg turn
                else:
                    print("green - 5 degree")  # 5 deg steering
                    butngreen2 = True          # double tap started
            else:                              # else 1st tap
                print("green - 1 degree")      # 1 deg steering
                butngreen2 = False             # first tap
            butngreen = True                   # register the pressed state
            greenepoch = time.time()           # start timer
    else:
        butngreen = False                      # button released
            
    if (btnblack.is_pressed):
        if not butnblack:
            if ((time.time() - blackepoch) < .6): # if less than .6 sec
                print('black - all stop')      # all stop!
            else:
                print('black - zero steering') # zero steering
            butnblack = True
            blackepoch = time.time()           # reset
    else:
        butnblack = False
            
    if (btnred.is_pressed):
        if not butnred:
            if ((time.time() - redepoch) < .6): # if less than .6 sec
                if butnred2:
                    print('red - 35 degree')    # 35 deg steering
                else:
                    print('red - 5 deg')        # 5 deg steering
                    butnred2 = True
            else:
                print('red - 1 deg')            # 1 deg steering
                butnred2 = False
            butnred = True
            redepoch = time.time()
    else:
        butnred = False
            
    if (btnblue.is_pressed):
        if not butnblue:
            print('blue')
                butnblue = True
        else:
            butnblue = False
            
        if (btnyellow.is_pressed):
            if not butnyellow:
                print('yellow')
                butnyellow = True
        else:
            butnyellow = False
            
           
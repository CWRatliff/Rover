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

btngreen2 = False
greenepoch = time.time()
btnred2 = False
redepoch = time.time()
blackepoch = time.time()

btngreen = gz.Button(GREEN)
btnblack = gz.Button(BLACK)
btnred = gz.Button(RED)
btnblue = gz.Button(BLUE)
btnyellow = gz.Button(YELLOW)

while True:
    # check tactile buttons
    if (btngreen.is_pressed):                   # button grounds out GPIO
        if not btngreen:                      # if not stale tap
            if ((time.time() - greenepoch) < .6): # if less than .6 sec
                if btngreen2:                 # if double tap in progress
                    print("green - 35 deg rightturn") # 35 deg turn
                else:
                    print("green - 5 degree")  # 5 deg steering
                    btngreen2 = True          # double tap started
            else:                              # else 1st tap
                print("green - 1 degree")      # 1 deg steering
                btngreen2 = False             # first tap
            btngreen = True                   # register the pressed state
            greenepoch = time.time()           # start timer
    else:
        btngreen = False                      # button released
            
    if (btnblack.is_pressed):
        if not btnblack:
            if ((time.time() - blackepoch) < .6): # if less than .6 sec
                print('black - all stop')      # all stop!
            else:
                print('black - zero steering') # zero steering
            btnblack = True
            blackepoch = time.time()           # reset
    else:
        butnblack = False
            
    if (btnred.is_pressed):
        if not btnred:
            if ((time.time() - redepoch) < .6): # if less than .6 sec
                if btnred2:
                    print('red - 35 degree')    # 35 deg steering
                else:
                    print('red - 5 deg')        # 5 deg steering
                    btnred2 = True
            else:
                print('red - 1 deg')            # 1 deg steering
                btnred2 = False
            btnred = True
            redepoch = time.time()
    else:
        btnred = False
            
    if (btnblue.is_pressed):
        if not btnblue:
            print('blue')
            btnblue = True
        else:
            btnblue = False
            
        if (btnyellow.is_pressed):
            if not btnyellow:
                print('yellow')
                btnyellow = True
        else:
            btnyellow = False
            
           
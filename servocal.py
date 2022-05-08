from whichrover import *
import serial

import motor_driver_ada

log = open("logfile.txt", 'w')
robot = motor_driver_ada.motor_driver_ada(log)
corner = input("corner ")
angle = Rlfbias
while True:
    print("angle = ", angle)
    robot.set_angle(corner, angle)
    robot.diag()
    tweak = input("plus-minus ")
    if (tweak == 'x'):
        break
    if (tweak == '+'):
        angle = angle + 1
    if (tweak == '-'):
        angle = angle - 1
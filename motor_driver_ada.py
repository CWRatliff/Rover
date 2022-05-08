#rover motor driver class - 4 servo motors for steering, 6 DC motors for locomotion
#190721 steering limits into this module
#200305 changed to new adafruit servo class
#200404 used 'D' hubs and individual biases
#200405 corrected actual motor to port mapping
#N.B. RC 0x81 & 0x82 may be exchanged on 'Spot 2'
#200421 incorporated roboclaw methods directly
#200605 swapped 0x81 & 0x82, back to original roboclaw
'''        
        self.lfbias = 48        # experimentally determined for 'Spot 2'
        self.lrbias = 44
        self.rrbias = 69
        self.rfbias = 40
'''

from whichrover import *

from adafruit_servokit import ServoKit
kit = ServoKit(channels = 16)

import serial
import math
from roboclaw import Roboclaw

class motor_driver_ada:

    def __init__(self, log):
        self.lfbias = Rlfbias        # experimentally determined for 'Spot 2'
        self.lrbias = Rlrbias
        self.rrbias = Rrrbias
        self.rfbias = Rrfbias
        self.pan_bias = 83
        self.left_limit = -36
        self.right_limit = 36
        self.d1 = Rd1         #C/L to corner wheels
        self.d2 = Rd2          #mid axle to fwd axle
        self.d3 = Rd3          #mid axle to rear axle
        self.d4 = Rd4        #C/L to mid wheels
        self.speedfactor = 35   # 8000 counts at 100%
        self.rr_motor = kit.servo[0]
        self.rf_motor = kit.servo[1]
        self.lf_motor = kit.servo[2]
        self.lr_motor = kit.servo[3]
        self.pan = kit.servo[15]
        self.tilt = kit.servo[14]

#pan_bias = 0
        self.rr_motor.actuation_range = 120
        self.rf_motor.actuation_range = 120
        self.lf_motor.actuation_range = 120
        self.lr_motor.actuation_range = 120
        self.rr_motor.set_pulse_width_range(700, 2300)
        self.rf_motor.set_pulse_width_range(700, 2300)
        self.lf_motor.set_pulse_width_range(700, 2300)
        self.lr_motor.set_pulse_width_range(700, 2300)
        self.log = log
        
        self.rc = Roboclaw("/dev/ttyS0",115200)
        i = self.rc.Open()
        self.crc = 0
        self.port = serial.Serial("/dev/ttyS0", baudrate=115200, timeout=0.1)

        self.lf_motor.angle = self.lfbias
        self.rf_motor.angle = self.rfbias
        self.lr_motor.angle = self.lrbias
        self.rr_motor.angle = self.rrbias
        self.stop_all()
        ver = self.rc.ReadVersion(0x80)
        print(ver[0],ver[1])
        ver = self.rc.ReadVersion(0x81)
        print(ver[0],ver[1])
        ver = self.rc.ReadVersion(0x82)
        print(ver[0],ver[1])

    def diag(self):
        print("servo rr ="+str(self.rr_motor.angle))
        print("servo rf ="+str(self.rf_motor.angle))
        print("servo lf ="+str(self.lf_motor.angle))
        print("servo lr ="+str(self.lr_motor.angle))
#       self.turn_motor(0x80, vel, voc, 1)

    def set_motor(self, address, v, av, m12):
        vx = int(v * av)
        if (m12 == 1):
            self.rc.SpeedM1(address, vx)
        else:
            self.rc.SpeedM2(address, vx)

    def set_angle(self, corner, angle):
        if corner == 0:
            self.rr_motor.angle = angle
        elif corner == 1:
            self.rf_motor.angle = angle
        elif corner == 2:
            self.lf_motor.angle = angle
        else:
            self.lr_motor.angle = angle
    '''
        match corner:
            case 0:
                self.rr_motor.angle = angle
            case 1:
            case 2:
            case 3:
    '''

    '''
    def turn_motor(self, address, v, av1, av2):
        v1 = int(v * av1)
        v2 = int(v * av2)
        if v >= 0:
            self.rc.ForwardM1(address, v1)
            self.rc.ForwardM2(address, v2)
#             self.M1Forward(address, v1)
#             self.M2Forward(address, v2)
        else:
            self.rc.BackwardM1(address, abs(v1))
            self.rc.BackwardM2(address, abs(v2))
#             self.M1Backward(address, abs(v1))
#             self.M2Backward(address, abs(v2))
#       print("m1, m2 = "+str(v1)+", "+str(v2))
    '''
    
    def stop_all(self):
        self.set_motor(0X80, 0, 0, 1)
        self.set_motor(0X81, 0, 0, 1)
        self.set_motor(0X82, 0, 0, 1)
        self.set_motor(0X80, 0, 0, 2)
        self.set_motor(0X81, 0, 0, 2)
        self.set_motor(0X82, 0, 0, 2)

    def motor_speed(self):
        speed1 = self.rc.ReadSpeedM1(0x80)
        speed2 = self.rc.ReadSpeedM2(0x80)
        self.log.write("motor speed = %d, %d" % (speed1[1], speed2[1]))
        print("motor speed = %d, %d" % (speed1[1], speed2[1]))
        speed1 = self.rc.ReadSpeedM1(0x81)
        speed2 = self.rc.ReadSpeedM2(0x81)
        self.log.write("motor speed = %d, %d" % (speed1[1], speed2[1]))
        print("motor speed = %d, %d" % (speed1[1], speed2[1]))
        speed1 = self.rc.ReadSpeedM1(0x82)
        speed2 = self.rc.ReadSpeedM2(0x82)
        self.log.write("motor speed = %d, %d" % (speed1[1], speed2[1]))
        print("motor speed = %d, %d" % (speed1[1], speed2[1]))
#         self.battery_voltage()
        err = self.rc.ReadError(0x80)[1]
        if err:
            print("status of 0x80", err)
            self.log.write("0x80 error: %d" % err)
        err = self.rc.ReadError(0x81)[1]
        if err:
            print("status of 0x81", err)
            self.log.write("0x81 error: %d" % err)
        err = self.rc.ReadError(0x82)[1]
        if err:
            print("status of 0x82", err)
            self.log.write("0x82 error: %d" % err)
            
    def battery_voltage(self):
        volts = self.rc.ReadMainBatteryVoltage(0x80)[1]/10.0
        print("Ada Voltage = ",volts)
        self.log.write("Voltage: %5.1f\n" % volts)
        return (volts)

# based on speed & steer, command all motors
    def motor(self, speed, steer):
#        self.log.write("Motor speed, steer "+str(speed)+", "+str(steer)+'\n')
        if (steer < self.left_limit):
            steer = self.left_limit
        if (steer > self.right_limit):
            steer = self.right_limit
#        vel = speed * 1.26
        vel = self.speedfactor * speed
        voc = 0
        vic = 0
        #roboclaw speed limit 0 to 127
        # see BOT-2/18 (181201)
        # math rechecked 200329
        if steer != 0:                                  #if steering angle not zero, compute angles, wheel speed
            angle = math.radians(abs(steer))
            ric = self.d3 / math.sin(angle)             #turn radius - inner corner
            rm = ric * math.cos(angle) + self.d1        #body central radius
            roc = math.sqrt((rm+self.d1)**2 + self.d3**2) #outer corner
            rmo = rm + self.d4                          #middle outer
            rmi = rm - self.d4                          #middle inner
            phi = math.degrees(math.asin(self.d3 / roc))
            if steer < 0:
                phi = -phi
            voc = roc / rmo                             #velocity corners & middle inner
            vic = ric / rmo
            vim = rmi / rmo

# SERVO MOTORS ARE COUNTER CLOCKWISE
# left turn
        if steer < 0:
            self.lf_motor.angle = self.lfbias - steer
            self.rf_motor.angle = self.rfbias - phi
            self.lr_motor.angle = self.lrbias + steer
            self.rr_motor.angle = self.rrbias + phi
#            self.turn_motor(0x80, vel, voc, 1)          #RC 1 - rf, rm
#            self.turn_motor(0x81, vel, voc, vic)        #RC 2 - lm, lf
#            self.turn_motor(0x82, vel, vim, vic)        #RC 3 - rr, lr
            self.set_motor(0x80, vel, voc, 1)           #RC 1 - rf, rm
            self.set_motor(0x81, vel, voc, 1)           #RC 2 - lm, lf
            self.set_motor(0x82, vel, vim, 1)           #RC 3 - rr, lr
            self.set_motor(0x80, vel,   1, 2)           #RC 1 - rf, rm
            self.set_motor(0x81, vel, vic, 2)           #RC 2 - lm, lf
            self.set_motor(0x82, vel, vic, 2)           #RC 3 - rr, lr
#             cstr = "v, vout, vin %f %f %f\n" % (vel, voc, vic)
#             self.log.write(cstr)

#right turn
        elif steer > 0:
            self.lf_motor.angle = self.lfbias - phi
            self.rf_motor.angle = self.rfbias - steer
            self.lr_motor.angle = self.lrbias + phi
            self.rr_motor.angle = self.rrbias + steer
#            self.turn_motor(0x80, vel, vic, vim)
#            self.turn_motor(0x81, vel, vic, voc)
#            self.turn_motor(0x82, vel, 1, voc)
            self.set_motor(0x80, vel, vic, 1)
            self.set_motor(0x81, vel, vic, 1)
            self.set_motor(0x82, vel,   1, 1)
            self.set_motor(0x80, vel, vim, 2)
            self.set_motor(0x81, vel, voc, 2)
            self.set_motor(0x82, vel, voc, 2)
            #            print("80 vic, vim ",vic,vim)
#            print("81 vic, voc ",vic,voc)
#            print("82 vom, voc ", 1, voc)
#             cstr = "v, vout, vin %f %f %f\n" % (vel, voc, vic)
#             self.log.write(cstr)

#straight ahead
        else:
            self.lf_motor.angle = self.lfbias
            self.rf_motor.angle = self.rfbias
            self.lr_motor.angle = self.lrbias
            self.rr_motor.angle = self.rrbias
            self.set_motor(0x80, vel, 1, 1)
            self.set_motor(0x81, vel, 1, 1)
            self.set_motor(0x82, vel, 1, 1)
            self.set_motor(0x80, vel, 1, 2)
            self.set_motor(0x81, vel, 1, 2)
            self.set_motor(0x82, vel, 1, 2)
#       print("v, vout, vin "+str(vel)+", "+str(voc)+", "+str(vic))
#       self.diag()
#             cstr = "v, vout, vin %f %f %f\n" % (vel, voc, vic)
#             self.log.write(cstr)

    def sensor_pan(self, angle):
        self.pan.angle = self.pan_bias + angle
        
    def depower(self):
        self.lf_motor.angle = None
        self.rf_motor.angle = None
        self.lr_motor.angle = None
        self.rr_motor.angle = None
        self.pan.angle = None
        

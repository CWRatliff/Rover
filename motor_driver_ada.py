#rover motor driver class - 4 servo motors for steering, 6 DC motors for locomotion
#220510 using WhichRover for constants
#220723 kit object made "self"

from whichrover import *
from adafruit_servokit import ServoKit
# import serial
import math
from roboclaw import Roboclaw

class motor_driver_ada:

    def __init__(self, log):
        self.left_limit = -36
        self.right_limit = 36

        self.kit = ServoKit(channels = 16)

        self.rr_motor = self.kit.servo[0]
        self.rf_motor = self.kit.servo[1]
        self.lf_motor = self.kit.servo[2]
        self.lr_motor = self.kit.servo[3]
        self.pan = self.kit.servo[15]
        self.tilt = self.kit.servo[14]

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
#        self.port = serial.Serial("/dev/ttyS0", baudrate=115200, timeout=0.1)

        self.rr_motor.angle = Rrrbias
        self.rf_motor.angle = Rrfbias
        self.lf_motor.angle = Rlfbias
        self.lr_motor.angle = Rlrbias
        self.diag()
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

    '''
    def set_motor(self, address, v, av, m12):

        vx = int(v * av)
        if (m12 == 1):
            self.rc.SpeedM1(address, vx)     # cmd 35
        else:
            self.rc.SpeedM2(address, vx)     # cmd 36
        self.log.write("Motor cmd addr:%d, spd:%d, dir:%d/n" % (address, vx, m12))
    '''

    def set_motor(self, address, v, av, m12):
        vx = int(v * av * 1.26)
        if (m12 == 1):
            if vx >= 0:
                self.rc.ForwardM1(address, vx)
            else:
                self.rc.BackwardM1(address, abs(vx))
        else:
            if vx >= 0:
                self.rc.ForwardM2(address, vx)
            else:
                self.rc.BackwardM2(address, abs(vx))

    def set_angle(self, corner, angle):         # calibration method
        if corner == 0:
            self.rr_motor.angle = angle
        elif corner == 1:
            self.rf_motor.angle = angle
        elif corner == 2:
            self.lf_motor.angle = angle
        else:
            self.lr_motor.angle = angle

    def get_angle(self, corner):
        if corner == 0:
            return self.rr_motor.angle
        elif corner == 1:
            return self.rf_motor.angle
        elif corner == 2:
            return self.lf_motor.angle
        else:
            return self.lr_motor.angle

    def stop_all(self):                 # all stop
        self.set_motor(0X80, 0, 0, 1)
        self.set_motor(0X81, 0, 0, 1)
        self.set_motor(0X82, 0, 0, 1)
        self.set_motor(0X80, 0, 0, 2)
        self.set_motor(0X81, 0, 0, 2)
        self.set_motor(0X82, 0, 0, 2)

    def motor_diag(self):
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
        return (volts)

# based on speed & steer, command all motors
    def motor(self, speed, steer):
        self.log.write("Motor speed, steer "+str(speed)+", "+str(steer)+'\n')
        if (steer < self.left_limit):
            steer = self.left_limit
        if (steer > self.right_limit):
            steer = self.right_limit
#        vel = speed(in pcts) * motor speed 

        vel = speed

#        vel = Rspeedfactor * speed
        voc = 0.0
        vic = 0.0
        #roboclaw speed limit 0 to 127
        # see BOT-2/18 (181201)
        # math rechecked 200329
        if steer != 0:                                  #if steering angle not zero, compute angles, wheel speed
            angle = math.radians(abs(steer))
            ric = Rd3 / math.sin(angle)             #turn radius - inner corner
            rm = ric * math.cos(angle) + Rd1        #body central radius
            roc = math.sqrt((rm+Rd1)**2 + Rd3**2) #outer corner
            rmo = rm + Rd4                          #middle outer
            rmi = rm - Rd4                          #middle inner
            phi = math.degrees(math.asin(Rd3 / roc))
            if steer < 0:
                phi = -phi
            voc = roc / rmo                             #velocity corners & middle inner
            vic = ric / rmo
            vim = rmi / rmo

# SERVO MOTORS ARE COUNTER CLOCKWISE
# left turn
        if steer < 0:
            self.rr_motor.angle = Rrrbias + phi
            self.rf_motor.angle = Rrfbias - phi
            self.lf_motor.angle = Rlfbias - steer
            self.lr_motor.angle = Rlrbias + steer
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
            self.rr_motor.angle = Rrrbias + steer
            self.rf_motor.angle = Rrfbias - steer
            self.lf_motor.angle = Rlfbias - phi
            self.lr_motor.angle = Rlrbias + phi
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
            self.rr_motor.angle = Rrrbias
            self.rf_motor.angle = Rrfbias
            self.lf_motor.angle = Rlfbias
            self.lr_motor.angle = Rlrbias
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
        self.pan.angle = Rpanbias + angle
        
    def depower(self):
        self.rr_motor.angle = None
        self.rf_motor.angle = None
        self.lf_motor.angle = None
        self.lr_motor.angle = None
        self.pan.angle = None
        

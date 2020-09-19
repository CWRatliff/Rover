#interface via Xbee radios
# using SPI instead of I2C
#190621 - added compass following
#190715 - strengthened xbee input validation
#190720 - improved compass following
#190802 - U turns
#190816 - GPS, waypoints
#190826 - routes, compass corrections
#190925 - EKF added
#191023 - major code review
#200314 - I/F with Arduino via USB-tty serial port
#200412 - restructure flag and auto blocks for faster EKF
#200813 - wpts upgrade, code improvements esp. route & U'ies
#200819 - switch to vector ops, better waypoint convergence
#200919 - Kalman time linked to gps input

'''
+---------+----------+----------+  +---------+----------+----------+
| L 1deg  | Fwd      | R 1deg   |  | L 90deg | Auto     | R 90deg  |
|        1|         2|         3|  |        1|         2|         3|
+---------+----------+----------+  +---------+----------+----------+
| L 5deg  | 0 steer  | R 5deg   |  | Mag     |          | Mag      |
|        4|         5|         6|  | L 1deg 4|         5| R 1deg  6|
+---------+----------+----------+  +---------+----------+----------+
| L 35deg | Rev      | R 35deg  |  | L 180   |          | R 180    |
|        7|         8|         9|  |        7|         8|         9|
+---------+----------+----------+  +---------+----------+----------+
|         | Stop     |          |  |  *      | Stby     |          |
|         |         0|          |  |         |         0|          |
+---------+----------+----------+  +---------+----------+----------+

Sent codes:
a - status
c - course to wpt
d - distance to wpt
h - heading
l - lat/long
s - steering angle
v - speed

Received codes
D - one digit commands
E - 'star' commnds
F - 'pound' commands
L - gps lat/lon/accuracy
O - compass heading
T - 'D' commands, diagnostics
'''

import serial
import time
import math
import motor_driver_ada
import cEKF

steer = 0                               # current steering angle clockwise
speed = 0                               # current speed plus->forward
azimuth = 0                             # desired course
epoch = time.time()
gpsEpoch = epoch
hdg = 0                                 # true compass heading
travel = 0                              # odometer

oldsteer = 500
oldspeed = 500
oldhdg = 500
compass_adjustment = 12                 # Camarillo declination
compass_bias = 5                        # rover allignment

ilatsec = 0.0                           # input from GPS hardware
ilonsec = 0.0
flatsec = 0.0                           # Kalman filtered lat/lon
flonsec = 0.0

# all vectors in US Survey feet, AV - 34N14 by 119W04 based, RV - relative
filterRV = [0, 0]                       # Kalman filtered loc
posAV = [0, 0]                          # gps position
startAV = [0, 0]                        # waypoint track start
trackRV = [0, 0]                        # waypoint path from initial position to destination

latitude = math.radians(34.24)          # Camarillo
latfeet = 6079.99/60                    # Kyle's converter
lonfeet = -latfeet * math.cos(latitude)
accgps = 0.0                            # grps accuracy in ft
segstart = time.time()                  # speed segment start (seconds)
#spdfactor = .0088                       # convert speed percentage to ft/sec ref BOT:3/17
spdfactor = .0122                       # for 43 RPM
#spdfactor = .017                        # for 60 RPM

left = False
left_limit = -36
right_limit = 36

auto = False
flag = False
rteflag = False
mainwptflag = False

rtseg = 0
routes = [[0,0],                    #0
[28, 27, 0],                        #1
[28, 27, 26, 0],                    #2
[28, 30, 29, 31, 27, 28, 0],        #3
[0]]
          
wptdist = 0.0

waypts=[[0,1],[1,2],[2,3],[3,4],[4,5],[5,6],[6,7],[7,8],[8,9],[9,10],
[22.678, 9.399],                    #10 open area near main gate
[20.808, 7.730, "speed bump"],      #11 mid speed bump
[20.641, 7.396, "T seam"],          #12 center parking 'T' seam
[20.945, 6.375, "gar door"],        #13
[20.987, 6.066, "driveway center"], #14
[20.830, 5.945, "gravel"],          #15
[20.200, 5.556, "woodpile"],        #16
[19.372, 6.355, "stairs pivot"],    #17
[19.071, 6.840, "shed #4"],         #18
[18.393, 6.283, "longe center"],    #19
[18.181, 7.900, "stall ctr"],       #20
[21.174, 6.110, "E dway start"],    #21
[22.227, 6.883, "horse gravel"],    #22
[22.846, 7.390, "trash"],           #23
[22.599, 7.159, "EF east entry"],   #24

[20.804, 7.949, "ref corner - F"],  #25
[20.984, 7.713, "hose bib - F"],    #26
[21.491, 7.646, "rose bush - F"],   #27
[22.039, 7.401, "boat corner - F"], #28
[22.461, 8.176, "EF middle - F"],   #29
[22.319, 7.696, "office gap"],      #30
[22.003, 7.820, "EF rose gap"],     #31
[11,12]]

version = "Rover 1.0 200919\n"
print(version)
tme = time.localtime(time.time())
print (tme)
log = open("logfile.txt", 'a')
log.write(version)
#log.write(tme)
robot = motor_driver_ada.motor_driver_ada(log)
robot.battery_voltage()
Kfilter = cEKF.Kalman_filter()
port = "/dev/ttyUSB0"
tty = serial.Serial(port, 9600)
tty.flushInput()

#====================================================
def vdot(U, V):
    return (U[0]*V[0] + U[1]*V[1])
def vmag(V):
    return math.sqrt(V[0]*V[0] + V[1]*V[1])
def vsmult(V, scalar):
    return [V[0]*scalar, V[1]*scalar]
def vsub(headV, tailV):
    return [headV[0]-tailV[0], headV[1]-tailV[1]]

# get compass course from direction vector
def vcourse(V):
    return (450 - math.degrees(math.atan2(V[1],V[0])) % 360)
# cvt lat/lon seconds to U.S survey feet
def vlatlon(latsec, lonsec):
    return [lonsec*lonfeet, latsec*latfeet]
def vprint(txt, V):
    str = "%s: [%6.1f, %6.1f]" % (txt, V[0], V[1])
    print(str)
    logit(str)
    
#================tty = serial.Serial(port, 9600)==============
def readusb():
    try:
        dd = tty.read(1).decode("utf-8")
        return(dd)
    except UnicodeDecodeError:
        print("Woops")
        return (0)
#==================================================================
def odometer(spd):
    global travel
    global segstart
    
    now = time.time()
    delT = now - segstart
    travel += delT * spdfactor * abs(spd)
    segstart = now
#==================================================================
def max_turn(angle):
    global steer
    global speed
    global left_limit
    global right_limit
    
    dt = 1
    if (angle < 0):
        if steer > (left_limit + 1):
            while (steer > left_limit):
                steer -= dt
                robot.motor(speed, steer)
                time.sleep(0.05)
#        robot.motor(speed, steer)
    else:
        if steer < (right_limit + 1):
            while (steer < right_limit):
                steer += dt
                robot.motor(speed, steer)
                time.sleep(0.05)
#        robot.motor(speed, steer)
    return
#===================================================================
def new_waypoint(nwpt):
    global posAV
    global startAV
    global trackRV
    global azimuth
    global wptdist
    global auto
    global wptflag
    global epoch
    
    startAV = posAV
    vprint("track start", startAV)
    destAV = vlatlon(waypts[nwpt][0], waypts[nwpt][1])
    logit("wpt: %d %7.2f, %7.2f" % (nwpt, destAV[0], destAV[1]))
    trackRV = vsub(destAV, startAV)
    vprint("track", trackRV)
    azimuth = vcourse(trackRV);
    logit("az set to %d" % azimuth)
    wptdist = vmag(trackRV)
    auto = True
    wptflag = True
    sendit("{aWp" + str(nwpt) + "}")
    Kfilter.Kalman_start(time.time(), posAV[0], posAV[1], \
        (math.radians(450-hdg) % 360), \
        speed * spdfactor)
    epoch = time.time()
    return

#===================================================================
def simple_commands(schr):
    global auto
    global azimuth
    global speed
    global steer
    
    if schr == '0':                     # 0 - stop 
        speed = 0
        odometer(0)
        robot.motor(speed, steer)

    elif schr == '1':                   # 1 - Left
        if (auto):
            azimuth -= 1
            logit("az set to %d" % azimuth)
        else:
            steer -= 1
            robot.motor(speed, steer)
            
    elif schr == '2':                   # 2 - Forward
        if speed <= 90:
            odometer(speed)
            speed += 10
            robot.motor(speed, steer)

    elif schr == '3':                   # 3 - Right
        if (auto):
            azimuth += 1
            logit("az set to %d" % azimuth)
        else:
            steer += 1
            robot.motor(speed, steer)

    elif schr == '4':                   # 4 - Left 5 deg
        if (auto):
            azimuth -= 5
            logit("az set to %d" % azimuth)
        else:
            steer -= 5
            robot.motor(speed, steer)
            
    elif schr == '5':                   # 5 - Steer zero
        dt = 1
        if steer > 0:
            while (steer > 0):
                steer -= dt
                robot.motor(speed, steer)
#               time.sleep(0.05)
        elif steer < 0:
            while (steer < 0):
                steer += dt
                robot.motor(speed, steer)
#               time.sleep(0.05)
        steer = 0
        robot.motor(speed, steer)
        if (auto):
            azimuth = hdg
            logit("az set to %d" % azimuth)

    elif schr == '6':                   # 6 - Left 5 deg
        if (auto):
            azimuth += 5
            logit("az set to %d" % azimuth)
        else:
            steer += 5
            robot.motor(speed, steer)
            
    elif schr == '7':                   # 7 - HAW steer left limit
        if (auto):
            azimuth += left_limit
            logit("az set to %d" % azimuth)
        else:
            max_turn(left_limit)
 
    elif schr == '8':                   # 8 -  Reverse
        if speed >= -90:
            odometer(speed)
            speed -= 10
            robot.motor(speed, steer)

    elif schr == '9':                   # 9 - GEE steer right limit
        if (auto):
            azimuth += right_limit
            logit("az set to %d" % azimuth)
        else:
            max_turn(right_limit)

    return
#===================end of D commands
def star_commands(schr):
    global auto
    global azimuth
    global auto
    global rteflag
    global wptflag
    global compass_adjustment
    global left
    
    if (schr == '0'):                   #standby
        auto = False
        wptflag = False
        rteflag = False
        azimuth = hdg
        sendit("{aStby}")
        logit("Standby")
    elif (auto and schr == '1'):      #left 90 deg
        azimuth -= 90
        azimuth %= 360
        logit("az set to %d" % azimuth)
    elif (schr == '2'):               #autopilot on
        auto = True
        azimuth = hdg
        sendit("{aAuto}")
        logit("Auto-pilot")
    elif (auto and schr == '3'):      #right 90 deg
        azimuth += 90
        azimuth %= 360
        logit("az set to %d" % azimuth)
    elif (schr == '4'):               #adj compass
        compass_adjustment -= 1
        logit("Compass bias "+str(compass_adjustment))
    elif (schr == '6'):               #adj compass
        compass_adjustment += 1
        logit("Compass bias "+str(compass_adjustment))
    elif (auto and schr == '7'):      #left 180 deg
        left = True
        max_turn(left_limit)
        azimuth -= 180
        azimuth %= 360
        logit("az set to %d" % azimuth)
    elif (auto and schr == '9'):      #right 180 deg
        left = False
        max_turn(right_limit)
        azimuth += 180
        azimuth %= 360
        logit("az set to %d" % azimuth)
    return
#================================================================
def diag_commands(schr):
    if (schr == '0'):
        logit("diagnostic #1 =======================")
        robot.motor_speed()
        log.flush()
    return

#=================================================================
def logit(xcstr):
    print(xcstr)
    log.write(xcstr + '\n')
    return
#=================================================================
def sendit(xcstr):
    tty.write(xcstr.encode("utf-8"))
    return
#=================================================================
#=================================================================

sendit("{aStby}")
logit("Standby")
sendit("{d----}")
sendit("{c----}")
cbuff = ""

try:
    while True:             #######  main loop    #######
        
        while (tty.inWaiting() > 0): # read characters from slave
            d = readusb()
            if (d == 0):
                continue
            if (d == '{'):
                cbuff = ""
                flag = False
            cbuff += d
            if (d == '}'):
                flag = True
                break
            #endwhile read

#========================================================================
        if (flag):                          # flag means we got a command
            flag = False
            msglen = len(cbuff)
            if (msglen < 3 or cbuff[0] != '{'):
                print("bad msg: ", cbuff)
                cbuff = ""
                continue
            xchr = cbuff[1]
            if (xchr != 'O'):             #ignore compass input (too many)
                tt = time.localtime()
                ts = time.strftime("%H:%M:%S ", tt)
                logit("msg: " + ts + cbuff)
               
            if (xchr >= 'A') and (xchr <= 'Z'):

#======================================================================
# single digit keypad commands
                if (xchr == 'D'):
                    xchr = cbuff[2]
                    simple_commands(xchr)
#======================================================================
# Keypad commands preceded by a star
                if xchr == 'E':
                    xchr = cbuff[2]
                    star_commands(xchr)
#======================================================================
#Keypad commands preceeded by a #
                elif xchr == 'F':                   #goto waypoint
                    try:
                        wpt = int(cbuff[2:msglen-1])
                        print("wpt= ")
                        print(wpt)
                        if wpt == 0:                # end of waypoint / route
                            wptflag = False
                            rteflag = False
                            auto = False
                            sendit("{aStby}")
                            logit("Standby")
                            sendit("{d----}")
                            sendit("{c---}")
                            odometer(speed)
                            speed = 0
                            robot.motor(speed, steer)
                        elif (wpt > 0 and wpt < 4):   # start of route
                            route = wpt
                            rteflag= True
                            rtseg = 0
                            wptflag = True
                            wpt = routes[route][rtseg]
                            new_waypoint(wpt)
                        elif (wpt >= 10 and wpt <= 31): #start of waypoint
                            new_waypoint(wpt)
                        else:
                            pass
                        
                    except ValueError:
                        print("bad data" + cbuff)
#======================================================================
                elif xchr == 'L':                   #lat/long input from GPS h/w
                    xchr = cbuff[2]
                    try:
                        x = float(cbuff[3:msglen-1])
                        if (xchr == 'T'):
                            ilatsec = x
                            posAV = vlatlon(ilatsec, ilonsec)
                        elif xchr == 'N':
                            ilonsec = x
                            posAV = vlatlon(ilatsec, ilonsec)
                            gpsEpoch = time.time()
                        elif xchr == 'A':
                            accgps = x * .00328084   #cvt mm to feet
                            if (accgps < 50):
                                cstr = "{lt%6.2f}" % accgps    #send to controller
                                sendit(cstr)
                                logit(cstr)

                        else:
                            pass

                    except ValueError:
                        print("bad data" + cbuff)
                    finally:
                        pass

#============================================================================= 
                elif xchr == 'O':                   #O - orientation esp hdg from arduino
                    hdg = int(cbuff[2:msglen-1])
                    hdg = (hdg + compass_adjustment + compass_bias)%360
#===========================================================================
                elif xchr == 'T':                   #'D' key + number button Diagnostic
                    xchr = cbuff[2]
                    diag_commands(xchr)
#=========================================================================                    
                else:
                    pass
                #
            flag = False
            cbuff = ""
            # endif flag        module = self._original_import(*args, **kw)

#======================================================================
        if (auto):
                     
            if (time.time() > (epoch + 1)):     #once per sec
                epoch = time.time()

                if wptflag:
                    v = speed * spdfactor
                    phi = math.radians((450-hdg) % 360)
                    ct = time.time()
                    logit("time: " + str(ct))
                    logit("raw L/L: " + str(ilatsec) + "/" + str(ilonsec))
                    logit("raw hdg: %6.1f" % hdg)
                    logit("raw speed: %5.3f" % v)
                    xEst = Kfilter.Kalman_step(gpsEpoch, posAV[0], \
                            posAV[1], phi, v)
#                    xEst = Kfilter.Kalman_step(time.time(), posAV[0], \
#                            posAV[1], phi, v)
                    flonsec = xEst[0, 0] / lonfeet
                    flatsec = xEst[1, 0] / latfeet
                    fhdg = (450 - math.degrees(xEst[2, 0])) % 360
                    logit("filtered L/L: %7.4f/%7.4f" % (flatsec, flonsec))
                    logit("Filtered hdg: %6.1f" % fhdg)
                    logit("Filtered speed: %6.3f" %xEst[3, 0])
                    workAV = vlatlon(flatsec, flonsec)   # see BOT 3:41 for diagram
                    filterRV = vsub(workAV, startAV)
                    vprint("Kalman pos vec", filterRV)
                    aimRV = vsub(trackRV, filterRV)
                    dtg = vmag(aimRV)
                    udotv = vdot(trackRV, filterRV)
                    if (udotv > 0):
                        trk = udotv / wptdist
                        progRV = vsmult(trackRV, trk / wptdist)
                        vprint("progress vec", progRV)
                        xtrackRV = vsub(progRV, filterRV)
                        vprint("xtrack vec", xtrackRV)
                        xtrk = vmag(xtrackRV)
                        
                        prog = vmag(filterRV)/wptdist     # progress along track
                        aim = (1.0 - prog) / 2 + prog     # aim at half the remaining dist on trackV
                        workRV = vsmult(trackRV, aim)
                        aimRV = vsub(workRV, filterRV)    # vector from filteredV to aimV                     
                    else:
                        xtrk = 0
                    azimuth = vcourse(aimRV)

                    vprint("wpt target vector", aimRV)
                    logit("az set to %d" % azimuth)

                    cstr = "{d%5.1f} " % dtg
                    sendit(cstr)
                    logit(cstr)
                    
                    cstr = "{c%3.0f} " % azimuth
                    sendit(cstr)
                    logit(cstr)

                    if (xtrk < 1000 and xtrk > -1000):
                        cstr = "{ln%5.1f} " % xtrk   #send to controller
                        sendit(cstr)
                        logit(cstr)
 
                    if (accgps < 1000 and accgps > -1000):
                        cstr = "{lt%5.1f} " % accgps    #send to controller
                        sendit(cstr)
                        logit(cstr)

                    #closing on waypoint
                    if (dtg < max(2.0, accgps) or vdot(aimRV, trackRV) <= 0):
                        logit("close to waypoint")
                        if rteflag:
                            rtseg += 1
                            wpt = routes[route][rtseg]
                            if (wpt == 0):
                                sendit("{aStby}")
                                logit("Standby")
                                wptflg = False
                                rteflag = False
                                odometer(speed)
                                speed = 0
                                auto = False
                            else:
                                new_waypoint(wpt)

                        else:
                            sendit("{aStby}")
                            logit("Standby")
                            wptflag =  False
                            odometer(speed)
                            speed = 0
                            auto = False
                        #endif dtg ===================
                    #endif wptflag ===================
                #endif epoch timer ===================
                                
            steer = int(azimuth - hdg)
            if (steer < -180):
                steer = steer + 360
            elif (steer > 180):
                steer = steer - 360
            if (abs(steer) == 180):
                if left:
                    steer = -180
                else:
                    steer = 180
            robot.motor(speed, steer)
            #endif auto ===========================
                
        if (hdg != oldhdg):
            cstr = "{h%3d}" % hdg
            sendit(cstr)
            oldhdg = hdg
            logit(cstr)
        if (speed != oldspeed):
            cstr = "{v%4d}" % speed
            sendit(cstr)
            oldspeed = speed
            logit(cstr)
        if (steer != oldsteer):
            cstr = "{s%4d}" % steer
            sendit(cstr)
            oldsteer = steer
            logit(cstr)

        # endwhile main loop ========================
    #endtry ======================

finally:
    robot.motor(0,0)
    robot.battery_voltage()
    print("odometer: %7.1f" % travel)
    logit("odometer: %7.1f" % travel)
    log.close()
    cstr = "{aStop}"
    tty.write(cstr.encode("utf-8"))
    print("Stopped")
#    robot.deinit()
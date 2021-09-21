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
#200920 - Dead Reconning when no recent GPS
#200922 - routes track from wpt to wpt (except at start)
#201009 - dodging obstacles
#201223 - use gps to adjusty IMU heading bias
#201226 - work on waypoint homeing
#201227 - if xtrk > 3, use filtered hdg to recompute bias
#210103 - monitor rapid yaw, rebias if detected
#210110 - revise bias resets (using BNO080 gyro, no-mag)
#210730 - A* trip planning
#210918 - Lidar sensor / object dodging

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
E - '*' commnds
F - '#' commands
L - gps lat/lon/accuracy
O - compass heading
T - 'D' commands, diagnostics
'''

import serial
import datetime
import time
import math
import motor_driver_ada
import cEKF
import astar

steer = 0                               # current steering angle clockwise
speed = 0                               # current speed plus->forward
approach_factor = .75                     # after waypoint slowdown
resume_speed = speed
reducedflag = False
azimuth = 0                             # desired course
epoch = time.time()
starttime = epoch
gpsEpoch = epoch
oldEpoch = epoch
hdg = 0                                 # true compass heading
fhdg = 0                                # Kalman filtered heading
yaw = 0                                 # latest IMU yaw (True north)reading
travel = 0                              # odometer
cogBase = 0
pan = 0

oldsteer = 500
oldspeed = 500
oldhdg = 500
#declination = 12                        # Camarillo declination
compass_bias = 98                        # for canopy table (using gyro/quat, no-mag

# all vectors in US Survey feet, AV - 34N14 by 119W04 based, RV - relative
aimRV = [0.0, 0.0]                      # aim point
cogAV = [0.0, 0.0]                      # cogBase starting point
destAV = [0.0, 0.0]                     # waypoint destination
estAV = [0.0, 0.0]                      # est pos either posAV or posAV+DR
filterRV = [0.0, 0.0]                   # Kalman filtered loc
pathRV = [0.0, 0.0]                     # from present pos to wpt end
posAV = [0.0, 0.0]                      # gps position
startAV = [0.0, 0.0]                    # waypoint track start
trackRV = [0.0, 0.0]                    # waypoint path from initial position to destination
workAV = [0.0, 0.0]                     # Kalman filtered AV

ilatsec = 0.0
ilonsec = 0.0
gotolat = 0.0
gotolon = 0.0
latitude = math.radians(34.24)          # Camarillo
latfeet = 6079.99/60                    # Kyle's converter                    startAV = posAV

lonfeet = -latfeet * math.cos(latitude)
accgps = 0.0                            # grps accuracy in ft
segstart = time.time()                  # speed segment start (seconds)
#spdfactor = .0088                       # convert speed percentage to ft/sec ref BOT:3/17
#spdfactor = .0122                       # for 43 RPM
#spdfactor = .017                        # for 60 RPM
spdfactor = .025                        # for 84 RPM

left = False
left_limit = -36
right_limit = 36

auto = False
flag = False
#rteflag = False
wptflag = False

rtseg = 0
route = [0]

routes = [[0,0],                    #0
    [28, 27, 0],                        #1
    [28, 30, 29, 32, 33, 34, 35, 36, 37, 38, 39, 42, 43, 27, 28, 0],  #2
    [28, 30, 29, 31, 27, 28, 0],        #3 - E.F. meander
    [14, 15, 16, 22, 18, 22, 16, 15, 14, 13, 0], #4 - to hut #4 and back
    [0]]
          
wptdist = 0.0

# (x, y) ((East, North)) in U.S. Survey feet offsets from 34-14N -119-4W
waypts=[[0,1],[1,2],[2,3],[3,4],[4,5],[5,6],[6,7],[7,8],[8,9],[9,10],
    [ -787.36,  2298.03],     #10 
    [ -787.00,  1977.00],     #11 Arena pepper
    [ -758.44,  1910.67],     #12 Arena spur
    [ -599.97,  2236.32],     #13 workshop F 
    [ -578.94,  2247.67],     #14 driveway center F
    [ -490.20,  2097.75],     #15 gravel rev 210430
    [ -471.80,  2053.82],     #16 fig tree fork F
    [ -698.89,  1856.89],     #17 Arena throat
    [ -592.93,  1931.82],     #18 shed #3/#4 F
    [ -526.33,  1863.82],     #19 longe center
    [ -661.79,  1842.34],     #20 stall ctr
    [ -511.84,  2145.63],     #21 E dway start
    [ -548.45,  1951.78],     #22 hut row bend F
    [ -619.07,  2315.06],     #23 trash
    [ -482.00,  1902.22],     #24 Arena ease exit
    [ -665.89,  2108.14],     #25 ref corner - F
    [ -464.67,  1928.44],     #26 Arena turnoff
    [ -640.51,  2179.75],     #27 rose bush - F 210102 modified to avoid roses
    [ -624.85,  2235.41],     #28 boat corner - F 201230 - refounded
    [ -684.91,  2276.04],     #29 EF middle - F
    [ -644.70,  2261.65],     #30 office gap
    [ -653.41,  2229.63],     #31 EF rose gap
    [ -718.05,  2274.10],     #32 To main gate
    [ -775.75,  2299.39],     #33 To main gate
    [ -809.37,  2295.23],     #34 W of driveway
    [ -844.68,  2286.36],     #35 g gate
    [ -814.28,  2152.33],     #36 Down ag road
    [ -784.66,  2132.42],     #37 Horse gar entrance
    [ -725.68,  2148.27],     #38 Above horse gar
    [ -694.38,  2162.42],     #39 Drive way above gar
    [ -686.08,  2149.59],     #40 EF entrance
    [ -662.14,  2171.71],     #41 Into EF
    [ -662.99,  2114.42],     #42 Potty turn
    [ -648.05,  2126.97],     #43 Potty pass
    [ -672.13,  2294.55],     #44 EF mid to back gate
    [ -677.84,  2330.52],     #45 EF to back gate
    [ -663.71,  2339.98],     #46 EF to back gate
    [ -641.60,  2338.68],     #47 To Back gate driveway
    [ -634.85,  2340.83],     #48 Back drive driveway
    [ -608.32,  2295.08],     #49 Back gate t\into EF
    [ -647.22,  2276.52],     #50 into EF
    [  -0.00,  0.00]]

obsarray = [[-578.94,  2247.67],     # virtual tree = wp #14
    [-644.80, 2268.85],                  # virtual tree between #30- #29
    [-660.99, 2221.52],
    [-646.81, 2240.18],
    [-646.63, 2255.84],
    [-634.29, 2266.28],
    [-622.74, 2270.73],
    [2.,0.], [2.,5.]]
ndx = 0

version = "Rover 1.1 210730\n"
print(version)
tme = time.localtime(time.time())
print (tme)

log = open("logfile.txt", 'a')
log.write("====================================================================")
log.write(version)
path = open("path.txt", 'a')
#log.write(tme)
robot = motor_driver_ada.motor_driver_ada(log)
volts = robot.battery_voltage()
print("Voltage = ",volts)
log.write("Voltage: %5.1f\n" % volts)
Kfilter = cEKF.Kalman_filter()
port = "/dev/ttyUSB0"
tty = serial.Serial(port, 9600)
tty.flushInput()

#====================================================
def cartesian(compass):
    return (450 - compass) % 360
def vadd(U, V):
    return [U[0]+V[0], U[1]+V[1]]
def vdot(U, V):
    return (U[0]*V[0] + U[1]*V[1])
def vmag(V):
    return math.sqrt(V[0]*V[0] + V[1]*V[1])
def vsmult(V, scalar):
    return [V[0]*scalar, V[1]*scalar]
def vsub(headV, tailV):
    return [headV[0]-tailV[0], headV[1]-tailV[1]]
def vunit(V):
    mag = vmag(V)
    return [V[0]/mag, V[1]/mag]

# get compass course from direction vector
def vcourse(V):
    return (450 - math.degrees(math.atan2(V[1],V[0]))) % 360
# cvt lat/lon seconds to U.S survey feet
def vft2sec(feetE, feetN):
    return [feetN/latfeet, feetE/lonfeet]
# cvt US feet to lat/lon seconds
def vsec2ft(latsec, lonsec):
    return [lonsec*lonfeet, latsec*latfeet]
def vprint(txt, V):
    str = "%s: [%7.2f, %7.2f]" % (txt, V[0], V[1])
    logit(str)
# compute distance from point to vector, ret +dist if right turn
# indicated, else -dist for left see BOT 3:51
def distance(P, V):
    if V[0] == 0:
        return (P[0])
    m = V[1] / V[0]
    c = posAV[1] - m * posAV[0]
    dst = (m * P[0] - P[1] + c)/math.sqrt(m*m + 1)
    if (V[1] * m) < 0:        # if sign(Vy) != sign(m)
        return -dst
    return dst

def vclosestwp(V):
    global waypts
    wlen = len(waypts)
    lowdist = vmag(vsub(waypts[10], V))
    lowwp = 10
    for i in range(10, wlen):
        print(i)
        w =  waypts[i]
        if w[0] < 0:
            testdist = vmag(vsub(w, V))
            if testdist < lowdist:
                lowdist = testdist
                lowwp = i
    return lowwp, lowdist
    
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
    logit("Odo: %7.1f, speed: %4d" % (travel, spd))
    segstart = now
#==================================================================
def max_turn(angle, spd):
    global steer
    
    dt = 1
    if (angle < 0):
        if steer > (left_limit + 1):
            while (steer > left_limit):
                steer -= dt
                robot.motor(spd, steer)
                time.sleep(0.04)
#        robot.motor(speed, steer)
    else:
        if steer < (right_limit + 1):
            while (steer < right_limit):
                steer += dt
                robot.motor(spd, steer)
                time.sleep(0.04)
#        robot.motor(speed, steer)
    return
#===================================================================
def new_waypoint(nwpt):
    global destAV
    global trackRV
    global azimuth
    global wptdist
    global auto
    global wptflag
    global epoch
    
    vprint("track start======================================", startAV)
    destAV = [waypts[nwpt][0], waypts[nwpt][1]]
    logit("wpt: %d %7.2f, %7.2f" % (nwpt, destAV[0], destAV[1]))
    trackRV = vsub(destAV, startAV)
    vprint("track", trackRV)
    azimuth = vcourse(trackRV)
    logit("new wpt az set to %d" % azimuth)
    wptdist = vmag(trackRV)
    auto = True
    wptflag = True
    sendit("{aWp%2d}" % nwpt)
    Kfilter.Kalman_start(time.time(), posAV[0], posAV[1], \
        math.radians((450-hdg) % 360), \
        speed * spdfactor)
    epoch = time.time()
    return
#===================================================================
# look for point obstructions
def obstructions():
    global startAV
    obs = boxtest(posAV[0], posAV[1], destAV[0], destAV[1])
    obsAV = obs[0]             #just use 1st one for now
    if (obsAV[0] != 0):
        vprint("obstruction", obsAV)
        obsRV = vsub(obsAV, posAV)
        obsdist = vmag(obsRV)
        if (obsdist < wptdist):
            odot = vdot(obsRV, trackRV)
            if odot > 0:
                obsproj = odot/(wptdist*wptdist)
                obsprojRV = vsmult(trackRV, obsproj)
                vprint("detour", obsprojRV)
                obsxRV = vsub(obsprojRV, obsRV)
                obsxdist = vmag(obsxRV)
                if (obsxdist < 3):
                    obsxRV = vunit(obsxRV)
                    obsxRV = vsmult(obsxRV, 3.0)
                    obsAV = vadd(obsRV, obsxRV)
                    obsAV = vadd(obsAV, posAV)
                    vprint("avoidance", obsAV)
                    waypts[1] = obsAV
                    startAV = posAV
                    new_waypoint(1)
                    route.insert(rtseg, 1)
    return

def db_search(x0):
    global ndx
    ndx = 0
    while obsarray[ndx][0] < x0:
        ndx +=1
        if obsarray[ndx][0] == 0:
            return [0, 0]
    return [obsarray[ndx][0], obsarray[ndx][1]]

def db_next():
    global ndx
    ndx +=1
    return [obsarray[ndx][0], obsarray[ndx][1]]
    
def boxtest(x0, y0, x1, y1):
    if x0 > x1:
        x0, x1 = x1, x0
    if y0 > y1:
        y0, y1 = y1, y0
    x0 -= 3.0           # enlarge box
    x1 += 3.0
    y0 -= 3.0
    y1 += 3.0
    list = [[0, 0]]
    obs = db_search(x0)
    while (obs[0] != 0 and obs[0] <= x1):
        if obs[1] >= y0 and obs[1] <= y1:
            list.insert(0, [obs[0], obs[1]])
        obs = db_next()
    return list

#===================================================================
def simple_commands(schr):
    global azimuth
    global pan
    global speed
    global steer
    global startAV
    
    if schr == '0':                     # 0 - stop 
        odometer(speed)
        speed = 0
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
            if (wptflag):
                #make a 3 ft left detour
                dodgeV = [-aimRV[1], aimRV[0]]
                dodgeV = vunit(dodgeV)
                dodgeV = vsmult(dodgeV, 3.0)
                dodgeV = vadd(dodgeV, aimRV)
                dodgeV = vadd(dodgeV, posAV)
                vprint("dodging to", dodgeV)
                waypts[1] = dodgeV
                startAV = posAV
                new_waypoint(1)
                route.insert(rtseg, 1)
            else:
                azimuth += left_limit
                logit("az set to %d" % azimuth)
        else:
            max_turn(left_limit, speed)
 
    elif schr == '8':                   # 8 -  Reverse
        if speed >= -90:
            odometer(speed)
            speed -= 10
            robot.motor(speed, steer)

    elif schr == '9':                   # 9 - GEE steer right limit
        if (auto):
            if (wptflag):
                #make a 3 ft right detour
                dodgeV = [aimRV[1], -aimRV[0]]
                dodgeV = vunit(dodgeV)
                dodgeV = vsmult(dodgeV, 3.0)
                dodgeV = vadd(dodgeV, aimRV)
                dodgeV = vadd(dodgeV, posAV)
                vprint("dodging to", dodgeV)
                waypts[1] = dodgeV
                startAV = posAV
                new_waypoint(1)
                route.insert(rtseg, 1)
            else:
                azimuth += right_limit
                logit("az set to %d" % azimuth)
        else:
            max_turn(right_limit, speed)

    elif schr == 'L':
        pan += 5
        robot.sensor_pan(pan)
        print("pan left")
    elif schr == 'C':
        pan = 0
        robot.sensor_pan(0)
    elif schr == 'R':
        pan -= 5
        robot.sensor_pan(pan)
    elif schr == 'U':
        pass
    elif schr == 'D':
        pass
    return
#===================end of D commands
def star_commands(schr):
    global auto
    global azimuth
    global auto
    global hdg
    global oldhdg
    global yaw
#    global rteflag
    global wptflag
    global compass_bias
    global left
    
    if (schr == '0'):                   #standby
        auto = False
        wptflag = False
#        rteflag = False
        azimuth = hdg
        sendit("{aStby}")
        logit("Standby")
    elif (auto and schr == '1'):      #left 90 deg
        azimuth -= 90
        azimuth %= 360
        logit("az set to %d" % azimuth)
    elif (schr == '2'):               #autopilot on
        auto = True
        oldhdg = 159
        wptflag = False
        azimuth = hdg
        sendit("{aAuto}")
        logit("Auto-pilot")
    elif (auto and schr == '3'):      #right 90 deg
        azimuth += 90
        azimuth %= 360
        logit("az set to %d" % azimuth)
    elif (schr == '4'):               #adj compass
        compass_bias -= 1
        logit("Compass bias %d" % compass_bias)
        xstr = "{h%3d}" % hdg
        sendit(xstr)
    elif (schr == '5'):               # adjust to true north
#        compass_bias = (110-hdg - declination) % 360
#        compass_bias = (149 - yaw - declination) % 360
        compass_bias = (149 - yaw) % 360
        logit("Compass bias %d" % compass_bias)
        oldhdg = 159
        hdg = 149
        azimuth = hdg
        xstr = "{h%3d}" % hdg
        sendit(xstr)
    elif (schr == '6'):               #adj compass
        compass_bias += 1
        logit("Compass bias %d" % compass_bias)
        xstr = "{h%3d}" % hdg
        sendit(xstr)
    elif (auto and schr == '7'):      #left 180 deg
        left = True
        robot.motor(0, 0)       #stop
        max_turn(left_limit, -50)
        time.sleep(3.5)           #guess at time needed
        str = left_limit
        dt = 1
        while str < right_limit:
            str += dt
            robot.motor(0, str)
            time.sleep(0.04)
        azimuth += 180
        azimuth %= 360
        logit("az set to %d" % azimuth)
    elif (schr == '8'):
#        compass_bias = (329-hdg - declination) % 360
#         compass_bias = (329 - yaw - declination) % 360
        compass_bias = (329 - yaw) % 360
        logit("Compass bias %d" % compass_bias)
        hdg = 329
        oldhdg = 329
        azimuth = hdg
        xstr = "{h%3d}" % hdg
        sendit(xstr)
#     elif (auto and schr == '8'):    #T-bone U'ie
#         max_turn(left_limit, 50)
#         time.sleep(3)
#         robot.motor(-50, 0)
#         time.sleep(3)
#         max_turn(left_limit, speed)
#         azimuth += 180
#         azimuth %= 360
#         logit("az set to %d" % azimuth)
#        
    elif (auto and schr == '9'):      #right 180 deg
        left = False
        max_turn(right_limit, speed)
        azimuth += 180
        azimuth %= 360
        logit("az set to %d" % azimuth)
    return
#================================================================
def diag_commands(schr):
    if (schr == '0'):
        logit("diagnostic #1 ==============================================================")
        robot.motor_speed()
        volts = robot.battery_voltage()
        xchr = "{b%f5.1}" % volts
        sendit(xchr)
        print("Voltage = ",volts)
        log.write("Voltage: %5.1f\n" % volts)
        logit("odometer: %7.1f" % travel)
        logit("az set to %d" % azimuth)
        logit("yaw %d" % yaw)
        logit("hdg %d" % hdg)
        logit("bias %d" % compass_bias)
        log.flush()
    if (schr == '1'):
        exit()
    if (schr == '2'):
        dtt = datetime.datetime.now()
        dts = dtt.strftime("%H:%M:%S.%f")[:-3]
        path.write("%12s,%9.2f,%8.2f,%8.2f,%8.2f,%8.2f,%4d,%4d,%4d,%5.2f\n" % \
            (dts,0 ,posAV[0], posAV[1], workAV[0], workAV[1], speed, steer, hdg, accgps))
        path.flush()
    if (schr == '3'):
        logit("IMU non-op")
        exit()
    if (schr == '4'):
        logit("GPS non-op")
        exit()
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
sendit("{ln----}")
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
#            if (xchr != 'O'):             #ignore compass input (too many)
            if (xchr != 'Z'):             #show compass input
                tt = datetime.datetime.now()
#                tt = time.localtime()
#                ts = time.strftime("%H:%M:%S ", tt)
                ts = tt.strftime("%H:%M:%S.%f")[:-3]
                logit("msg: " + ts + cbuff)
               
            if (xchr >= 'A') and (xchr <= 'Z'):

#======================================================================
# single digit keypad commands
                if (xchr == 'D'):
                    xchr = cbuff[2]
                    simple_commands(xchr)
                    cogBase = 0              #invalidate COG baseline
#======================================================================
# Keypad commands preceded by a star
                if xchr == 'E':
                    xchr = cbuff[2]
                    star_commands(xchr)
                    cogBase = 0              #invalidate COG baseline
#======================================================================
#Keypad commands preceeded by a #
                elif xchr == 'F':                   #goto waypoint
                    try:
                        wpt = int(cbuff[2:msglen-1])
                        print("wpt= ")
                        print(wpt)
                        if wpt == 0:                # end of waypoint / route
                            wptflag = False
#                            rteflag = False
                            auto = False
                            sendit("{aStby}")
                            logit("Standby")
                            sendit("{d----}")
                            sendit("{c---}")
                            odometer(speed)
                            speed = 0
                            robot.motor(speed, steer)
                            
                        elif wpt == 1:                    # <<<<<<< RTB >>>>>>>>>
                            startwp, startdist = vclosestwp(posAV)
                            route = astar.astar(startwp, 28)   # except for car interference, 13
                            route.append(0)
                            if startdist < 3.0:           # if too close to starting waypoint
                                route.pop(0)
                            rtseg = 0
                            wptflag = True
                            wpt = route[0]
                            startAV = posAV
                            new_waypoint(wpt)
#                            obstructions()
                           
                        elif (wpt > 1 and wpt < 5):   # start of route
                            route = routes[wpt]
                            route.append(0)
                            startwp, startdist = vclosestwp(posAV)
                            if startdist < 3.0:           # if too close to starting waypoint
                                route.pop(0)
                            rtseg = 0
                            wptflag = True
                            wpt = route[0]
                            startAV = posAV
                            new_waypoint(wpt)
#                            obstructions()

                        elif (wpt >= 10 and wpt <= 50): #start of waypoint
                            startwp, startdist = vclosestwp(posAV)
                            route = astar.astar(startwp, wpt)
                            if startdist < 3.0:           # if too close to starting waypoint
                                route.pop(0)
                            if len(route) > 0:           # make sure there is a route
                                route.append(0)
                                for rt in route:
                                    cstr = "route: %d" % rt
                                    logit(cstr)
                                wpt = route[0]
                                rtseg = 0
                                startAV = posAV
                                new_waypoint(wpt)
#                               obstructions()
                        else:
                            pass
                        
                    except ValueError:
                        print("bad data" + cbuff)
#======================================================================
                elif xchr == "G":          # goto lat/lon command
                    xchr = cbuff[2]
                    try:
                        x = float(cbuff[3:msglen-1])
                        if (xchr == 'N'):
                            gotolon = x

                        elif xchr == 'T':
                            if (gotolon != 0):   # GN shoule be first
                                gotolat = x
                                gotoAV = [gotolon, gotolat]
                                startwp, startdist = vclosestwp(posAV)
                                endwp, enddist = vclosestwp(gotoAV)
                                route = astar.astar(startwp, endwp)
                                if startdist < 3.0:           # if too close to starting point
                                    route.pop(0)
                                    startwp = route[0]
                                if endwp != startwp:         # are we here yet?
                                    u = vsub(waypts[endwp], waypts[startwp])
                                    v = vsub(gotoAV, waypts[endwp])
                                    if enddist > 3 and vdot(u,v) > 0: #if goto beyond last wp
                                        waypts[9] = gotoAV
                                        route.append(9)
                                route.append(0)
                                for rt in route:
                                    cstr = "route: %d" % rt
                                    logit(cstr)
                                rtseg = 0
                                wpt = route[rtseg]
                                startAV = posAV
                                new_waypoint(wpt)
#                                obstructions()

                            gotolat = 0.0      # reset
                            gotolon = 0.0
      
                    except ValueError:
                        gotolat = 0.0     # resetS
                        gotolon = 0.0

                    
#======================================================================
                elif xchr == 'L':                   #lat/long input from GPS h/w
                    xchr = cbuff[2]
                    try:
                        x = float(cbuff[3:msglen-1])
                        if (xchr == 'T'):
                            ilatsec = x
                            posAV = vsec2ft(ilatsec, ilonsec)
                        elif xchr == 'N':
                            ilonsec = x
                            posAV = vsec2ft(ilatsec, ilonsec)
                            if (not wptflag):
                                cstr = "{ln%5.1f} " % posAV[0]
                                sendit(cstr)
                                logit(cstr)
                                cstr = "{lt%5.1f} " % posAV[1]
                                sendit(cstr)
                                logit(cstr)
                            gpsEpoch = time.time()
                        elif xchr == 'A':
                            accgps = x * .00328084   #cvt mm to feet
                            if (accgps < 50):
                                cstr = "{la%6.2f}" % accgps    #send to controller
                                sendit(cstr)
                                logit(cstr)
                            else:
                                logit("Poor GPS accuracy")
                                sendit("{la------}")

                        else:
                            pass

                    except ValueError:
                        print("bad data" + cbuff)
                    finally:
                        pass

#============================================================================= 
                elif xchr == 'O':                   #O - orientation esp hdg from arduino
                    yaw = int(cbuff[2:msglen-1])
                    hdg = (yaw + compass_bias)%360
                    
#                     if (oldhdg < 500):
#                         if (oldhdg < 90 and hdg >= 270):
#                             delhdg = (hdg - 360) - oldhdg
#                         elif (oldhdg >= 270 and hdg < 90):
#                             delhdg = hdg - (oldhdg - 360)
#                         else:
#                             delhdg = hdg - oldhdg
#                             
#                         if (abs(delhdg > 2)):
#                             oldbias = compass_bias
#                             compass_bias = (compass_bias - delhdg) % 360
#                             hdg = (yaw + compass_bias)%360      #recompute
#                             logit("Yaw delta: Compass bias was %d now %d" % (oldbias, compass_bias))
#===========================================================================
#               {S1 P/S <dist> , <CW angle>} BOT 4:16
#               sensor #1 - TFmini giving obstacle main side and avoidence angle
#               'P'/'S' - port/starboard - side of obstruction
                elif xchr == 'S':             # sensor data
                    if (auto and wptflag):
                        xchr = cbuff[3]           # Port / Starboard
                        args = cbuff[4:msglen-1]
                        sdist, sang = args.split(',')
                        dist = float(sdist)
                        ang = int(sang)
                        rang = (ang * 71.0) / 4068.0   #angle to radians (approx)
                        aimUV = vunit(aimRV)
                        aimdistV = vsmult(aimUV, dist)  #where obstructuction intersects
                        
                        if (xchr == 'S'):               # if obs is mainly right of aimRV
                            xaimV = [-aimUV[1], aimUV[0]]   # CCW 90 deg
                            dodgeV = vsmult(xaimV, 3.0)
                            if rang < 0:
                                dodgeV = vadd(dodgeV, vsmult(xaimV, dist * math.sin(-rang)))# + intrusion
                        else: # xchr == 'P'
                            xaimV = [aimUV[1], -aimUV[0]]   # CW 90 deg
                            dodgeV = vsmult(xaimV, 3.0)
                            if rang > 0:
                                dodgeV = vadd(dodgeV, vsmult(xaimV, dist * math.sin(rang)))# + intrusion
                            
                        dodgeV = vadd(dodgeV, aimdistV)
                        dodgeV = vadd(posAV, dodgeV)
                        vprint("dodgeAV", dodgeV)

                        new_waypoint(1)
                        route.insert(rtseg, 1)
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
            # endif flag
#======================================================================
        if (auto):
                     
            if (time.time() > (epoch + 1)):     #once per sec
                oldEpoch = epoch
                epoch = time.time()

                if wptflag:
                    v = speed * spdfactor
                    phi = math.radians((450-hdg) % 360)
                    estAV = posAV
                    if (gpsEpoch < oldEpoch):          # no recent GPS lat/lon
                        delt = epoch - oldEpoch
                        dist = delt * v
                        xd = dist * math.sin(phi)
                        yd = dist * math.cos(phi)
                        estAV[0] = estAV[0] + xd
                        estAV[1] = estAV[1] + yd
                        vprint("estAV", estAV)
                    else:    
                        vprint("posAV", posAV)
                        
                    logit("time: " + str(epoch))
                    logit("wpt: %2d raw hdg: %6.1f" % (wpt, hdg))
                    logit("raw speed: %5.2f" % v)
                    xEst = Kfilter.Kalman_step(epoch, estAV[0], estAV[1], phi, v)
                    fhdg = int((450 - math.degrees(xEst[2, 0])) % 360)
                    logit("filtered EN pos: %7.2f/%7.2f" % (xEst[0, 0], xEst[1, 0]))
                    logit("Filtered hdg: %6.1f" % fhdg)
                    logit("Filtered speed: %6.2f" %xEst[3, 0])
                    workAV = [xEst[0, 0], xEst[1, 0]]   # see BOT 3:41 for diagram
                    filterRV = vsub(workAV, startAV)
                    vprint("Kalman pos vec", filterRV)
                    cstr = "{ln%5.1f} " % workAV[0]
                    sendit(cstr)
                    cstr = "{lt%5.1f} " % workAV[1]
                    sendit(cstr)
                    pathRV = vsub(trackRV, filterRV)      # to wpt end
                    dtg = vmag(pathRV)
                    udotv = vdot(trackRV, filterRV)
                    if (udotv > 0):
                        progRV = vsmult(trackRV, udotv/vdot(trackRV, trackRV)) # w = (u.v/v.v)*v
                        vprint("progress vec", progRV)
                        xtrackRV = vsub(progRV, filterRV)
                        vprint("xtrack vec", xtrackRV)
                        xtrk = vmag(xtrackRV)
                        
                        prog = vmag(progRV)/wptdist       # progress along track (fraction)
                        if (xtrk < 2.0):
                            aim = (1.0 - prog) / 2 + prog     # aim at half the remaining dist on trackV
                        else:
                            aim = (1.0 - prog) / 3 + prog     # aim at a third of the remaining dist on trackV
                        workRV = vsmult(trackRV, aim)
                        aimRV = vsub(workRV, filterRV)    # vector from filteredV to aimV                     
                        azimuth = vcourse(aimRV)
                    else:
                        xtrk = 0
                        azimuth = vcourse(trackRV)

                    vprint("aiming vector", aimRV)
                    logit("Kalman az set to %d" % azimuth)

                    if (dtg < 1000):
                        cstr = "{d%5.1f} " % dtg
                        sendit(cstr)
                        logit(cstr)
                    else:
                        sendit("{d-----}")
                    
                    cstr = "{c%3.0f} " % azimuth
                    sendit(cstr)
                    logit(cstr)

                    if (xtrk < 100.0):
                        cstr = "{lx%5.2f} " % xtrk   #send to controller
                        sendit(cstr)
                        logit(cstr)
 
                    if (accgps < 100.0):
                        cstr = "{la%5.2f} " % accgps    #send to controller
                        sendit(cstr)
                        logit(cstr)

                    if (wptflag and dtg < 8.0):
                        if (not reducedflag):
                            resume_speed = speed
                            odometer(speed)
                            speed = int(approach_factor * speed)
                            reducedflag = True
                        
                    #closing on waypoint
#                    if (dtg < max(2.0, accgps) or vdot(pathRV, trackRV) <= 0):
                    if (dtg < 2.0 or vdot(pathRV, trackRV) <= 0):
                        logit("close to waypoint")
#                        if rteflag:
                        rtseg += 1
                        wpt = route[rtseg]
                        if (wpt == 0):
                            sendit("{aStby}")
                            logit("Standby")
                            sendit("{d----}")
                            sendit("{c----}")
                            sendit("{lx----}")
                            wptflag = False
                            odometer(speed)
                            speed = 0
                            auto = False
                        else:
                            odometer(speed)
                            speed = resume_speed
                            reducedflag = False
                            startAV = destAV       # new wpt start = old wpt end
                            new_waypoint(wpt)

                        #endif dtg ===================
                    #endif wptflag ===================
                
                if (steer >= -1 and steer <= 1 and speed > 50):
                    if cogBase > 10:                                  # line long enough to compute heading
                        cogBaseRV = vsub(posAV, cogAV)
                        hdg = vcourse(cogBaseRV)
                        oldhdg = hdg
                        vprint("COG base course", cogBaseRV)
                        oldbias = compass_bias
#                         compass_bias = (hdg - yaw - declination) % 360
#                        compass_bias = (hdg - yaw) % 360
                        newbias = (hdg - yaw) % 360   #beware zero crossing
                        if (newbias > oldbias):
                            compass_bias += 1
                        elif (newbias < oldbias):
                            compass_bias -= 1
                            
#                         cstr = "{h%3d}" % hdg
#                         sendit(cstr)
                        logit("cogBase: Compass bias was %d now %d" % (oldbias, compass_bias))
                        azimuth = hdg
                        cogBase = 0
                    elif cogBase == 0:
                        cogBase = 1
                        cogAV = posAV
                    else:
                        cogBase += 1
                else:
                    cogBase = 0
                    
#                 if (wptflag and xtrk > 3.0):               # if xtrack grows too much, use filtered heading
#                     oldbias = compass_bias
# #                    compass_bias = (int(fhdg) - yaw - declination) % 360
#                     compass_bias = (int(fhdg) - yaw) % 360
#                     logit("XTrack: Compass bias was %d now %d" % (oldbias, compass_bias))
                    
                tt = datetime.datetime.now()
                ts = tt.strftime("%H:%M:%S.%f")[:-3]
                path.write("%12s,%9.2f,%8.2f,%8.2f,%8.2f,%8.2f,%4d,%4d,%4d,%5.2f\n" % \
                    (ts,epoch-starttime,posAV[0],posAV[1],workAV[0],workAV[1],speed,steer,hdg,accgps))
                path.flush()

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
            cstr = "hdg: %3d, bias: %3d" % (hdg, compass_bias)
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
    robot.motor(0, 0)
    time.sleep(0.5)           #wait for roboclaws
    volts = robot.battery_voltage()
    print("Voltage = ",volts)
    robot.depower()
    odometer(speed)
    logit("odometer: %7.1f" % travel)
    log.close()
    path.close()
    cstr = "{aStop}"
    tty.write(cstr.encode("utf-8"))
    print("Stopped")
#    robot.deinit()
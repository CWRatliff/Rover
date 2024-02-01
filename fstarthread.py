#210730 - A* trip planning
#210918 - Lidar sensor / object dodging
#211012 - over dodge due to vcross2 fixed
#211207 - use nearby path's waypoints if within 3 ft
#211210 - heartbeat
#220511 - revised structure wrt heartbeat
#220524 - DR when gps stuck, slipping, not current
#220616 - compute average heading from gps
#220720 - Tturn with thread
#231030 - pivot moved from single L/R click to triple click
'''
wptflag also gcode flag 220601
'''

'''
Sent codes:
a - status
c - course to wpt
d - distance to wpt
h - heading
l - lat/long/acc/xtrack
r - sensor
s - steering angle
v - speed

Received codes
D - one digit commands
E - '*' commnds - autopilot
F - '#' commands - route / waypts
G - goto lat/lon
L - gps lat/lon/accuracy
O - compass heading
P - start gcode script
Q - heartbeat
S - sensor input
T - 'D' commands, diagnostics
'''

from whichrover import *
import serial
import datetime
import time
import threading
import math
import motor_driver_ada
import cEKF
import astar2
from waypts import *
from vectors import *

DODGE = 2.0                             # obstruction dodge distance
LEFT = -1
RIGHT = 1
Left_Limit = -36
Right_Limit = 36
steeringdir = RIGHT                     # default start

compass_bias = Rcompass_bias
azimuth = Rcompass_bias                 # desired course
hdg = Rcompass_bias                     # true compass heading
spdfactor = Rspeed                      # fps @ 1%

steer = 0                               # current steering angle clockwise
speed = 0                               # current speed plus->forward
fhdg = 0                                # Kalman filtered heading
yaw = 0                                 # latest IMU yaw (True north)reading
travel = 0                              # odometer
cogBase = 0                             # course over ground distance
gpsokflag = False                       # gps looks current
gpsheading = 0                          # gps supplied heading of motion
gpsavghdg = 0                           # gps heading average
gpsavgcnt = 0                           # sample count for average heading
approach_factor = .75                   # after waypoint slowdown
resume_speed = speed
reducedflag = False
countheartbeat = 0                      # count of tardy heartbeats events
countgpstardy = 0                       # count of gps readings late events
countgpsrepeat = 0                      # count of gps stuck events
countgpsacc = 0                         # count of gps inaccurate events
countgpsfix = 0                         # count of good gps hits

epoch = time.time()
starttime = epoch
gpsEpoch = epoch                        # how long GPS good
oldEpoch = epoch
heartbeat = epoch                       # most recent heartbeat
tensecepoch = epoch                     # 10 seconds
seconds = 0
pan = 0
pivotflag = False                       # pivoting in-progress flag

oldsteer = 500                          # big values to trigger update
oldspeed = 500
oldhdg = 500
oldsec = 0
# compass_bias = 98                     # for canopy table (using gyro/quat, no-mag

# all vectors in US Survey feet, AV - 34N14 by 119W04 based, RV - relative
aimRV = [0.0, 0.0]                      # aim point
cogAV = [0.0, 0.0]                      # cogBase starting point
destAV = [0.0, 0.0]                     # waypoint destination
estAV = [0.0, 0.0]                      # est pos either posAV or posAV+DR
filterRV = [0.0, 0.0]                   # Kalman filtered loc
pathRV = [0.0, 0.0]                     # from present pos to wpt end
posAV = [-604.0, 2221.0]                # gps position
prevAV = [0.0, 0.0]                     # gps position previous sampling
startAV = [0.0, 0.0]                    # waypoint track start
trackRV = [0.0, 0.0]                    # waypoint path from initial position to destination
ekfAV = [0.0, 0.0]                      # Kalman filtered AV

ilatsec = 0.0
ilonsec = 0.0
gotolat = 0.0
gotolon = 0.0
latitude = math.radians(34.24)          # Camarillo
latfeet = 6079.99/60                    # Kyle's converter
lonfeet = -latfeet * math.cos(latitude)
accgps = 0.0                            # gps accuracy in ft
segstart = time.time()                  # speed segment start (seconds)

auto = False
cmdflag = False
wptflag = False
wpt = 0
startwp = 0
rtseg = 0
route = [0]
wptdist = 0.0
gcodeflag = False
#gcodefile = open

routes = [[0,0],                    #0
    [28, 0],                        # <<<<<<<<< RTB >>>>>>>>>>
    [28, 30, 29, 32, 33, 34, 35, 36, 37, 38, 39, 42, 43, 27, 28, 0],  #2
    [28, 30, 29, 31, 27, 28, 0],        #3 - E.F. meander
    [14, 15, 16, 22, 18, 22, 16, 15, 14, 13, 0], #4 - to hut #4 and back
    [0]]
          
log = open("logfile.txt", 'w')
gcodefile = log
robot = motor_driver_ada.motor_driver_ada(log)
log.write("====================================================================")
version = "Rover 1.2 240201\n"
log.write(version)
tme = time.localtime(time.time())
print(tme)
volts = robot.battery_voltage()
log.write("Voltage: %5.1f\n" % volts)
path = open("path.txt", 'w')

port = "/dev/ttyUSB0"
tty = serial.Serial(port, 9600)
tty.flushInput()

Kfilter = cEKF.Kalman_filter()
Kfilter.Kalman_start(time.time(), posAV[0], posAV[1], \
    math.radians((450-hdg) % 360), \
    speed * spdfactor)
epoch = time.time()

# Threads
# =========================================================================
# thread to monitor CW rotation, ending at goal 
def watchdogCW(goal):
    global pivotflag

    if hdg > 180 and goal < 180:
        while hdg > 180:         # keep turning til past the 359->0 hazard
            time.sleep(0.05)
            if pivotflag is False: # abort?
                return
    while hdg < goal:
        time.sleep(0.05)
        if pivotflag is False: # abort?
            return

    robot.stop_all()
    robot.motor(0, 0)
    time.sleep(0.1)
    pivotflag = False
    logit("watchdog thread ended")
    return
# =========================================================================
# thread to monitor CCW rotation, ending at goal 
def watchdogCCW(goal):
    global pivotflag

    if hdg < 180 and goal > 180:
        while hdg < 180:         # keep turning til past the 359->0 hazard
            time.sleep(0.05)
            if pivotflag is False: # abort?
                return
    while hdg > goal:
        time.sleep(0.05)
        if pivotflag is False: # abort?
            return

    robot.stop_all()
    robot.motor(0, 0)
    time.sleep(0.1)
    pivotflag = False
    logit("watchdog thread ended")
    return
#==================================================================
# rotate 4 corner wheels to allow pure rotation
# dir - LEFT or RIGHT
# delta - change in heading
# assumes drive motors are at full stop
def pivot(dir, delta):
    global pivotflag
    global azimuth

    if pivotflag is False:          # turn cannot be in progress
        robot.pivot1()              # turn corner wheels
        time.sleep(0.5)             # SWAG 
        robot.pivot2(dir)           # rotate in dir direction
        pivotflag = True
        if dir == LEFT:
            azimuth = (azimuth - delta) % 360
            thdg = (hdg - delta) % 360
            logit("pivot turn hdg %d" % thdg)
            bot_thread = threading.Thread(target = watchdogCCW,args=[thdg])
        else: # RIGHT
            azimuth = (azimuth + delta) % 360
            thdg = (hdg + delta) % 360
            logit("pivot turn hdg %d" % thdg)
            bot_thread = threading.Thread(target = watchdogCW,args=[thdg])

        logit("az set to %d" % azimuth)
        bot_thread.start()
    return
#=============================================================================
def pivotangle(heading, goal):
    delta = goal - heading
    if delta > 180:
        pivot(LEFT, 360 - delta)
    elif delta >= 0:
        pivot(RIGHT, delta)
    elif delta > -180:
        pivot(LEFT, -delta)
    else:
        pivot(RIGHT, delta + 360)
    return
# ============================================================================
# cvt lat/lon seconds to U.S survey feet
def vft2sec(feetE, feetN):
    return [feetN/latfeet, feetE/lonfeet]
# cvt US feet to lat/lon seconds
def vsec2ft(latsec, lonsec):
    return [lonsec*lonfeet, latsec*latfeet]
def vprint(txt, V):
    str = "%s: [%7.2f, %7.2f]" % (txt, V[0], V[1])
    logit(str)

def vclosestwp(V):
    global waypts
    wlen = len(waypts)
    lowdist = vmag(vsub(waypts[10], V))
    lowwp = 10
    for i in range(10, wlen):
        w =  waypts[i]
        if w[0] < 0:         # real waypoint?, long. should be neg.
            testdist = vmag(vsub(w, V))
            if testdist < lowdist:
                lowdist = testdist
                lowwp = i
    return lowwp, lowdist
'''
def compass_quadrant(angle):      # inverted & rotated for compass heading
    if (angle >= 0 and angle < 90):
        return 1
    if (angle >= 90 and angle < 180):
        return 2
    if (angle >= 180 and angle < 270):
        return 3
    if (angle >= 270 and angle < 360):
        return 4

# return turn angle given a compass hdg & compass goal, -CCW/+CW
def turn_angle(hdg, goal):
    dlt = (goal - hdg) % 360
    if dlt > 180:
        dlt -= 360
        pivot(LEFT, dlt)
    else:
        pivot(RIGHT, dlt)
'''        
# =================================================================
# from present pos (AV) to destination (AV), return safest,shortest route
def bestroute(pos, dest):

    rte1 = []
    rte2 = []
    wp0, _ = vclosestwp(pos)
    wpn, _ = vclosestwp(dest)

    dist1, rte1 = astar2.astar(wp0, wpn)         # start at closest waypoint
    dist1 += vmag(vsub(pos, waypts[rte1[0]]))
    dist2, rte2 = astar2.astar(rte1[1], wpn)     # see if skipping closest waypoint any shorter
    dist2 += vmag(vsub(pos, waypts[rte2[0]]))
    if dist1 < dist2:
        return rte1
    else:
        return rte2
'''
    global startwp

    rte = []
    endwpt, _ = vclosestwp(dest)
    # if near to a known path, use one of it's path's wpts although
    # closest actual waypt might be shorter. paths are safer
    wp1, wp2 = astar2.nearpath(pos)
    logit("bestroute near path wp1,2 %d, %d", (wp1, wp2))
    if (wp1 != 0):  # path(s) close by, two waypt candidates each
        dist1, route1 = astar2.astar(wp1, endwpt)
        dist2, route2 = astar2.astar(wp2, endwpt)
        if (dist1 < dist2): # which of the two wpts is closer
            startwp = wp1
            rte = route1
        else:   # not near any path
            startwp = wp2
            rte = route2
#        startdistb = vmag(vsub(pos, waypts[bstartwpt]))
    else: # overland path
        _, rte = astar2.astar(startwp, endwpt)
        
#    if startdistb < 3.0:           # if very close to starting point !?code also in route_waypoint
#        rte.pop(0)
    return rte   
'''
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
        if steer > (Left_Limit + 1):
            while (steer > Left_Limit):
                steer -= dt
                robot.motor(spd, steer)
                time.sleep(0.04)
#        robot.motor(speed, steer)
    else:
        if steer < (Right_Limit + 1):
            while (steer < Right_Limit):
                steer += dt
                robot.motor(spd, steer)
                time.sleep(0.04)
#        robot.motor(speed, steer)
    return
#===================================================================
# setup for a new waypoint

def new_waypoint(nwpt):
    global aimRV
    global destAV
    global pathRV
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
    aimRV = trackRV
    pathRV = trackRV
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

# =====================================================================
# setup for next waypoint in a list {route}
def advance_waypoint():
    global auto
    global destAV
    global reducedflag
    global route
    global rtseg
    global speed
    global wpt
    global wptflag
    global startAV
    
    rtseg += 1
    wpt = route[rtseg]
    if (wpt == 0):                  # end of route
        sendit("{aStby}")
        logit("Standby")
        sendit("{d----}")
        sendit("{c----}")
        sendit("{lx----}")
        wptflag = False
        reducedflag = False
        route = [0]
        rtseg = 0
        odometer(speed)
        speed = 0
        auto = False
    else:
        odometer(speed)
        if reducedflag:
            logit("resuming to %4d" % resume_speed)
            speed = resume_speed
        reducedflag = False
        startAV = destAV       # new wpt start = old wpt end
        new_waypoint(wpt)
    return
#===================================================================
# navigate a file of g-codes
# "g0 xnnn.nn ynnn.nn"          - goto delta lon = x, deltalat = y
# "g0 annn"                     - pivot to angle nnn
# "g1 xnnn.nn ynnn.nn [fnnn]"   - goto lon = x, lat = y, speed = nnn
# "g1 annn"                     - pivot by angle nnn
def advance_gcode_script():
    global destAV
    global startAV
    global gcodefile
    global gcodeflag

    if gcodeflag:
        try:
            gline = gcodefile.readline()
            parts = gline.split()
            if parts[0] == "g1":
                parts.pop(0)
                for part in parts:
                    code = part[0]
                    spec = float(part[1:10])
                    if code == 'x':
                        glon = spec
                    elif code == 'y':
                        glat = spec
                startAV = posAV
                destAV[0] = glon
                destAV[1] = glat
                new_waypoint(1)
 #               if wpt != 1:
 #                   route.insert(rtseg, 1)
                logit("gcode to %7.2f / %7.2f" % (glon, glat))
            elif parts[0] == "g0":
                pass

        finally:
            gcodeflag = False
    return
#===================================================================
# single digit commands as if from 16 key keypad
def simple_commands(schr):
    global azimuth
    global pan
    global pivotflag
    global speed
    global steer
    global startAV
    
    if schr == '0':                     # 0 - stop 
        odometer(speed)
        speed = 0
        robot.motor(speed, steer)
        pivotflag = False

    elif schr == '1':                   # 1 - Left
        if (auto):
            azimuth -= 1
            logit("az set to %d" % azimuth)
        else:
#             if speed != 0:
#                 steer += 1
#                 robot.motor(speed, steer)
#             else:
#                 pivot(LEFT, 45)
            steer += 1
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
#             if speed != 0:
#                 steer += 1
#                 robot.motor(speed, steer)
#             else:
#                 pivot(RIGHT, 45)
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
        pivotflag = False
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
                azimuth += Left_Limit
                logit("az set to %d" % azimuth)
        else:
#             max_turn(Left_Limit, speed)
            if speed != 0:
                max_turn(Left_Limit, speed)
            else:
                pivot(LEFT, 45)
 
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
                azimuth += Right_Limit
                logit("az set to %d" % azimuth)
        else:

#             max_turn(Right_Limit, speed)
            if speed != 0:
                max_turn(Right_Limit, speed)
            else:
                pivot(RIGHT, 45)

           
#============================ pan/tilt camera
    elif schr == 'Q':           #dont forget to change in tframe
        pan += 5
#        robot.sensor_pan(pan)
        robot.sensor_pan(0)
        print("pan left")
    elif schr == 'C':
        pan = 0
        robot.sensor_pan(0)
    elif schr == 'R':
        pan -= 5
#        robot.sensor_pan(pan)
        robot.sensor_pan(0)
    elif schr == 'U':
        for ang in range(0, 140):
            robot.crane(ang)
            time.sleep(0.1)
#        pass
    elif schr == 'D':
        for ang in range(140, 0, -1):
            robot.crane(ang)
            time.sleep(0.1)
#        pass
    elif schr == 'I':           # switch off: I, J, K
        robot.switch(0, 0)
    elif schr == 'X':            # switch on: X, Y, Z
        robot.switch(0, 1)

    return
#===================end of D commands

#================================================
# Autopilot commands

def star_commands(schr):
    global pivotflag
    global auto
    global azimuth
    global auto
    global hdg
    global steeringdir
    global yaw
    global wptflag
    
    if (schr == '0'):                 # '*0' - standby
        auto = False
        wptflag = False
        azimuth = hdg
        pivotflag = False
        sendit("{aStby}")
        logit("Standby")
    elif (auto and schr == '1'):      # '*1' - left 90 deg
        if speed == 0:
            pivot(LEFT, 90)
        else:
            azimuth -= 90
            azimuth %= 360
        logit("az set to %d" % azimuth)
    elif (schr == '2'):               # autopilot on
        auto = True
        wptflag = False
        azimuth = hdg
        pivotflag = False
        sendit("{aAuto}")
        logit("Auto-pilot")
    elif (auto and schr == '3'):      # right 90 deg
        if speed == 0:
            pivot(RIGHT, 90)
        else:
            azimuth += 90
            azimuth %= 360
        logit("az set to %d" % azimuth)
    elif (auto and schr == '7'):      # left 180 deg
        if speed == 0:
            pivot(LEFT, 180)
        else:
            steeringdir = LEFT
            max_turn(Left_Limit, speed)
            azimuth += 180
            azimuth %= 360
        logit("az set to %d" % azimuth)
    elif (schr == '8'):
        pass
    elif (auto and schr == '9'):      # right 180 deg
        if speed == 0:
            pivot(RIGHT, 180)
        else:
            steeringdir = RIGHT
            max_turn(Right_Limit, speed)
            azimuth += 180
            azimuth %= 360
        logit("az set to %d" % azimuth)
        return
#================================================================
def diag_commands(schr):
    global compass_bias
    global hdg
    global azimuth
    
    if (schr == '0'):
        logit("diagnostic #1 ===============================")
        dvolts = robot.battery_voltage()
        print("Voltage = ",dvolts)
        log.write("Voltage: %5.1f\n" % dvolts)
        robot.motor_diag()
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
            (dts,0 ,posAV[0], posAV[1], ekfAV[0], ekfAV[1], speed, steer, hdg, accgps))
        path.flush()
    if (schr == '3'):
        logit("IMU non-op")
        exit()
    if (schr == '4'):
        logit("GPS non-op")
        exit()
    if (schr == '5'):
        if gpsokflag is True and gpsavgcnt > 3:
            avg = int(gpsavghdg / gpsavgcnt)
            compass_bias = (avg - yaw) % 360
            logit("Average gps heading %d" % avg)
            logit("Yaw %d" % yaw)
            azimuth = avg
            hdg = avg
            logit("Compass bias set to: %d" % compass_bias)          
    return

#=================================================================
# new route from controller

def route_waypoint(schr):
    global auto
    global reducedflag
    global route
    global rtseg
    global speed
    global startAV
    global startwp
    global wpt
    global wptflag
    
    try:
        wpt = int(schr[2:msglen-1])
        print("wpt= ", wpt)
        if wpt == 0:                # end of waypoint / route
            wptflag = False
            auto = False
            route = [0]
            rtseg = 0
            sendit("{aStby}")
            logit("Standby")
            sendit("{d----}")
            sendit("{c---}")
            odometer(speed)
            speed = 0
            reducedflag = False
            robot.motor(speed, steer)
        else:
            startwp, startdist = vclosestwp(posAV)
            cstrt = "startwp, dist %d, %5.2f " % (startwp, startdist)
            logit(cstrt)
            
            if wpt == 1:                    # <<<<<<< RTB >>>>>>>>>
                route = bestroute(posAV, waypts[75])
                startdist = vmag(vsub(posAV, waypts[startwp]))          
                
            elif (wpt > 1 and wpt < 5):   # start of route
                rtewp = routes[wpt][0]    # 0th wpt in route
                _, route = astar2.astar(startwp, rtewp) # goto start of route
                routewpt = routes[wpt]
                routewpt.pop(0)              # delete common wpt
                route += routewpt
                
            elif (wpt >= 10 and wpt <= 76): #start of waypoint
                route = bestroute(posAV, waypts[wpt])
                startdist = vmag(vsub(posAV, waypts[startwp]))          
                
            if len(route) > 0:
                if startdist < 3.0:  # if too close to starting waypoint
                    route.pop(0)
                    logit("start is close, advancing route")
                if len(route) > 1:            # if start between wpts
                    rwp0 = route[0]
                    rwp1 = route[1]
                    rwpts0 = waypts[rwp0]
                    rwpts1 = waypts[rwp1]
                    dot = vdot(vsub(posAV, rwpts0), vsub(rwpts1, rwpts0))
                    if dot > 0:
                        rdist = pldistance(posAV, rwpts0, rwpts1)
                        if rdist < 3.0:
                            route.pop(0)
                            logit("track is near, advancing route")

                route.append(0)
                for rt in route:
                    crstr = "route: %d" % rt
                    logit(crstr)
                wpt = route[0]
                rtseg = 0
                startAV = posAV
                new_waypoint(wpt)
#                               obstructions()
        
    except ValueError:
        print("bad data" + cbuff)
#=====================================================================
# Goto command from controller

def goto_command(schr):
    global gotolat
    global gotolon
    global route
    global rtseg
    global startAV
    global startwp
    global wpt
    
    gchr = schr[2]
    try:
        x = float(schr[3:msglen-1])
        if (gchr == 'N'):
            gotolon = x

        elif gchr == 'T':
            if (gotolon != 0):   # GN should be first
                gotolat = x
                gotoAV = [gotolon, gotolat]
                ##endwp, enddist = vclosestwp(dest)
                
                route = bestroute(posAV, gotoAV)
                if (len(route) > 0):
                    startwp = route[0]
                endwp = route[-1]
                if endwp != startwp:         # are we here yet?
                    u = vsub(waypts[endwp], waypts[startwp]) #start to finish
                    v = vsub(gotoAV, waypts[endwp]) # diff between last waypt & actual endpoint
                    enddist = vmag(v)  # enddist = dist between last waypt & actual end
                    if enddist > 3 and vdot(u,v) > 0: #if goto beyond last wp
                        waypts[9] = gotoAV
                        route.append(9)
                route.append(0)
                for rt in route:
                    crstr = "route: %d" % rt
                    logit(crstr)
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

#=================================================================
def GPS_data(schr):        # input from Arduino
    global accgps
    global ilatsec
    global ilonsec
    global gpsavgcnt
    global gpsavghdg
    global gpsEpoch
    global gpsheading
    global gpsokflag
    global posAV
    global oldsec
    global prevAV
    global seconds
    
    gchr = schr[2]
    try:
        x = float(schr[3:msglen-1])
        if (gchr == 'T'):
            ilatsec = x
            posAV = vsec2ft(ilatsec, ilonsec)
            
        elif gchr == 'N':
            ilonsec = x
            posAV = vsec2ft(ilatsec, ilonsec)
            if (not wptflag):
                cgstr = "{ln%5.1f} " % posAV[0]
                sendit(cgstr)
                logit(cgstr)
                cgstr = "{lt%5.1f} " % posAV[1]
                sendit(cgstr)
                logit(cgstr)
            gpsEpoch = time.time()
            gpsokflag = True
            if prevAV == posAV and oldsec == seconds:
                gpsokflag = False
                vprint("prevAV", prevAV)
                vprint("posAV ", posAV)
                logit("GPS not updating ===============")
            prevAV = posAV
            
        elif gchr == 'A':
            accgps = x * .00328084   #cvt mm to feet
            if (accgps < 50):
                cgstr = "{la%5.2f}" % accgps    #send to controller
                sendit(cgstr)
                logit(cgstr)
            else:
                sendit("{la------}")

            if accgps > 3:
                gpsokflag = False
                logit("Poor GPS accuracy")
                
        elif gchr == 'S':
            oldsec = seconds
            seconds = int(x)
        
        elif gchr == 'P':
            gpsheading = int(x)
            if abs(steer) <= 2 and speed >= 50 and gpsokflag is True:
                gpsavghdg += gpsheading
                gpsavgcnt += 1
            else:
                gpsavghdg = 0
                gpsavgcnt = 0

        else:
            pass

    except ValueError:
        print("bad data" + cbuff)
    finally:
        pass
#========================================================================
# {S1 P/S <dist> , <CW angle>} BOT 4:16
# sensor #1 - TFmini giving obstacle main side and avoidence angle
# 'P'/'S' - port/starboard - side of obstructiondef sensor_data(schr):
def sensor_data(schr):
    global speed
    global startAV
    global wpt
    global waypts
    
    if (auto and wptflag and Rsensor):
        gchr = schr[3]           # Port / Starboard
        args = schr[4:msglen-1]
        sdist, sang, cswath = args.split(',')
        distan = float(sdist)
        ang = int(sang)
        swath = int(cswath)
        dtogo = vmag(pathRV)
        obsUV = vcompass(hdg + ang)
        obsRV = vsmult(obsUV, dist)
        vprint("obstacleV", obsRV)
        # if dist > dtg:
        #   advance_waypoint()
        # or get next wpt aiming vector and see if it dodges
        #if dist < (dtg + 3.0):  # if obs closer than wpt & 3ft margin
        if (dtg < 3.0 and distan > dtogo):
            logit("advancing wpt")
            advance_waypoint()
            
        aimUV = vunit(aimRV)
        proj = vdot(obsRV, aimUV)
        print("proj", proj)
        if proj >= 0:       #obs on same side as aimRV
            projRV = vsmult(aimUV, proj) #u.v/v.v * v
            vprint("projRV", projRV)
            dodgeRV = vsub(projRV, obsRV)   # from obs to aim
            vprint("dodgeRV", dodgeRV)
            wdist = vmag(dodgeRV)
            print("wdist", wdist)
            dodgeUV = vunit(dodgeRV)
            dodgeRV = vsmult(dodgeUV, DODGE)
            if abs(wdist) < 3.0:      # obs within minimum
                rot = vcross2(obsRV, aimRV)    # obsV to aimV rotation
                print("rot", rot)
                
                if gchr == 'S':      # if obs is mainly right of aimRV
                    if not rot:
                        dodgeRV = vsmult(dodgeUV, wdist + DODGE)# + intrusion
                if (gchr == 'P'):
                    swath = -swath
                    if rot:
                        dodgeRV = vsmult(dodgeUV, wdist + DODGE)# + intrusion
                 
                vprint("dodgeV", dodgeRV)
                dodgeRV = vadd(dodgeRV, projRV)
                dodgeAV = vadd(dodgeRV, posAV)
                waypts[1] = dodgeAV
                startAV = posAV
                new_waypoint(1)
                if wpt != 1:
                    route.insert(rtseg, 1)
                wpt = 1
                odometer(speed)
                speed = 0
                robot.motor(speed, steer)
                cgstr = "{r%d,%d,%d} " % (distan, hdg+ang, swath)
                sendit(cgstr)
#=================================================================================
# controller commands
def commander(cmdbuff):
    global gcodeflag
    global gcodefile
    global heartbeat
    global cogBase
    global hdg
    global yaw
    
    xcmd = cmdbuff[1]
    if (xcmd >= 'A') and (xcmd <= 'Z'):
        if (xcmd == 'D'):       # single digit keypad commands
            xcmd = cmdbuff[2]
            simple_commands(xcmd)
            cogBase = 0         #invalidate COG baseline

        elif xcmd == 'E':       # Keypad commands preceded by a star
            xcmd = cmdbuff[2]
            star_commands(xcmd)
            cogBase = 0         #invalidate COG baseline

        elif xcmd == 'F':       # Keypad commands preceeded by a #
            route_waypoint(cmdbuff)

        elif xcmd == "G":       # goto lat/lon command
            goto_command(cmdbuff)

        elif xcmd == 'L':       #lat/long input from GPS h/w
            GPS_data(cmdbuff)

        elif xcmd == 'O':       #O - orientation esp hdg from arduino
            yaw = int(cmdbuff[2:msglen-1])
            hdg = (yaw + compass_bias)%360

        elif xcmd == 'P':       #P - gcode script
            gcodefile = open("gcode.gcode", 'r')
            gcodeflag = True

        elif xcmd == 'Q':       # Q - heartbeat
#           print("heartbeat")
            heartbeat = time.time()

        elif xcmd == 'S':       # sensor data
            sensor_data(cmdbuff)

        elif xcmd == 'T':       #'D' key + number button Diagnostic
            xcmd = cmdbuff[2]
            diag_commands(xcmd)
                   
        else:
            pass
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
                cmdflag = False
            cbuff += d
            if (d == '}'):
                cmdflag = True
                break
            #endwhile read

#========================================================================
        if (cmdflag):                          # flag means we got a command
            cmdflag = False
            heartbeat = time.time()
            msglen = len(cbuff)
            if (msglen < 3 or cbuff[0] != '{'):
                print("bad msg: ", cbuff)
                cbuff = ""
                continue
            xchr = cbuff[1]
            # if (xchr != 'O'):             # don'tshow compass input
            if (xchr != 'x' and xchr != 'Q'): # don'tshow heartbeat input
                tt = datetime.datetime.now()
                ts = tt.strftime("%H:%M:%S.%f")[:-3]
                logit("msg: " + ts + cbuff)
                
                commander(cbuff)
               
            cmdflag = False
            cbuff = ""
            if pivotflag is True:
                time.sleep(0.05)          # give a thread a time slice SWAG
            # endif cmdflag
            
#========================================================================
        if (time.time() > (tensecepoch + 10)): # ten second timer
            tensecepoch = time.time()
            volts = robot.battery_voltage()
            xchr = "{b%5.1f}" % volts
            sendit(xchr)
            
#======================================================================
        if (time.time() > (epoch + 1)):     # once per sec
            oldEpoch = epoch
            epoch = time.time()
            
            # heartbeat
            sendit("{qqq}")
            if (time.time() > (heartbeat + 2.0)):   # if more than 2 seconds since last controller msg
                logit(" heartbeat tardy")
                countheartbeat += 1
                # if in R/C mode, stop
                if (not auto):
                    odometer(speed)
                    speed = 0
                    robot.motor(speed, steer)

            vel = speed * spdfactor
            phi = math.radians((450-hdg) % 360)
            estAV = posAV
            # if no recent GPS or (no GPS movement when speed > 0)
            # then do dead reconing
            if (gpsEpoch < oldEpoch) \
                or (gpsokflag is False and speed > 0) \
                or accgps > 4.0:   # no recent GPS lat/lon
                
                delt = epoch - oldEpoch
                dist = delt * vel
                estAV[0] = estAV[0] + dist * math.sin(phi)
                estAV[1] = estAV[1] + dist * math.cos(phi)
                vprint("estAV", estAV)
                cogbase = 0
                if gpsEpoch < oldEpoch:
                    logit("gps tardy")
                    countgpstardy += 1
                elif accgps > 4.0:
                    logit("gps inaccurate")
                    countgpsacc += 1
                else:
                    logit("gps repeat position")
                    countgpsrepeat += 1
                logit("Dead Reconing =========================")
#               gpsokflag = True
            else:    
                vprint("posAV", posAV)
                countgpsfix += 1
                
#           logit("time: " + str(epoch))
            logit("wpt: %2d raw hdg: %6.1f" % (wpt, hdg))
            logit("raw speed: %5.2f" % vel)

            xEst = Kfilter.Kalman_step(epoch, estAV[0], estAV[1], phi, vel)
            fhdg = int((450 - math.degrees(xEst[2, 0])) % 360)
            ekfAV = [xEst[0, 0], xEst[1, 0]]   # see BOT 3:41 for diagram
            filterRV = vsub(ekfAV, startAV)
            vprint("Kalman pos vec", filterRV)
            vprint("filtered E-N pos: ", ekfAV)
            logit("Filtered hdg: %6.1f" % fhdg)
            logit("Filtered speed: %6.2f" % xEst[3, 0])

            cstr = "{ln%5.1f} " % ekfAV[0]
            sendit(cstr)
            cstr = "{lt%5.1f} " % ekfAV[1]
            sendit(cstr)
                
            if (auto):
                if wptflag:
                    pathRV = vsub(trackRV, filterRV)      # to wpt end
                    dtg = vmag(pathRV)
                    udotv = vdot(trackRV, filterRV)       # see if we are past track start
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
                        azimuth = int(vcourse(aimRV))
                    else:
                        xtrk = 0
                        azimuth = int(vcourse(trackRV))

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

                    if (dtg < 8.0):         # starting to get close to waypoint
                        if (not reducedflag):
                            resume_speed = speed
                            odometer(speed)
                            speed = int(approach_factor * speed)
                            reducedflag = True
                        resume_speed = max(resume_speed, speed)           
                        
                    #closing on waypoint
                    if (dtg < 2.0 or vdot(pathRV, trackRV) <= 0):
                        logit("close to waypoint")
                        if gcodeflag == True:
                            advance_gcode_script()
                        else:
                            advance_waypoint()

                        #endif dtg ===================
                    #endif wptflag ===================
                
                if (steer >= -1 and steer <= 1 and speed > 50 and gpsokflag is True):
                    if cogBase > 10:                 # line long enough to compute heading
                        cogBaseRV = vsub(posAV, cogAV) ############# estAV?????
                        hdg = vcourse(cogBaseRV)
                        oldhdg = hdg
                        vprint("COG base course", cogBaseRV)
                        logit("COG hdg %d" % hdg)
                        oldbias = compass_bias
                        newbias = (hdg - yaw) % 360   #beware zero crossing
                        # consider splitting difference instead of 1 deg
                        if (newbias > oldbias):
                            compass_bias += 1
                        elif (newbias < oldbias):
                            compass_bias -= 1
                            
                        logit("cogBase: Compass bias was %d now %d" % (oldbias, compass_bias))
                        # azimuth = hdg # ******************************************** 230618
                        cogBase = 0
                    elif cogBase == 0:
                        cogBase = 1
                        cogAV = posAV
                    else:
                        cogBase += 1
                else:
                    cogBase = 0
                    
                if pivotflag is False:         # no pivot maneuver in progress
                    steer = int(azimuth - hdg)
                    logit("new hdg steering %d" % steer)
                    if (steer < -180):
                        steer = steer + 360
                    elif (steer > 180):
                        steer = steer - 360
                    if (abs(steer) == 180):
                        if steeringdir == LEFT:
                            steer = -180
                        else:
                            steer = 180
                    robot.motor(speed, steer)
                #endif auto ===========================
                
            tt = datetime.datetime.now()
#            ts = tt.strftime("%H:%M:%S.%f")[:-3]
#            path.write("%12s,%9.2f,%8.2f,%8.2f,%8.2f,%8.2f,%4d,%4d,%4d,%5.2f\n" % \
#                (ts,epoch-starttime,posAV[0],posAV[1],ekfAV[0],ekfAV[1],speed,steer,hdg,accgps))
            ts = tt.strftime("%H:%M:%S")
            path.write("%8s,%8.2f,%8.2f,%8.2f,%4d,%4d,%4d,%5.2f\n" % \
                (ts,epoch-starttime,posAV[0],posAV[1],speed,steer,hdg,accgps))
            path.flush()

            #endif epoch timer ===================

        # things to do as often as possible outside of timer               
        # if on course, don't oversteer
        if auto is True and pivotflag is False:
            if steer != 0:
                diff = int(azimuth - hdg)
                if diff > 180:
                    diff -= 360
                if diff < -180:
                    diff += 360
                if steer > 0 and diff < 0:           # right turn
                    diff = 0                         # overshot
                if steer < 0 and diff > 0:           # left turn
                    diff = 0                         # overshot
                if diff != steer:
                    steer = diff
                    robot.motor(speed, steer)
                    logit("adjusting steer to meet az %d" % steer)
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
    logit("Voltage: %5.1f" % volts)
    robot.depower()
    odometer(speed)
    logit("odometer: %7.1f" % travel)
    logit("missing heartbeats: %5d" % countheartbeat)
    logit("late gps readings: %5d" % countgpstardy)
    logit("repeat gps readings: %5d" % countgpsrepeat)
    logit("inaccurate gps: %5d" % countgpsacc)
    logit("good gps: %5d" % countgpsfix)

    log.close()
    path.close()
    cstr = "{aStop}"
    sendit(cstr)
    print("Stopped")
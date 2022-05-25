#210730 - A* trip planning
#210918 - Lidar sensor / object dodging
#211012 - over dodge due to vcross2 fixed
#211207 - use nearby path's waypoints if within 3 ft
#211210 - heartbeat
#220511 - revised structure wrt heartbeat
#220524 - DR when gps stuck, slipping, no current

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
E - '*' commnds
F - '#' commands
G - goto lat/lon
L - gps lat/lon/accuracy
O - compass heading
S - sensor input
T - 'D' commands, diagnostics
'''

from whichrover import *
import serial
import datetime
import time
import math
import motor_driver_ada
import cEKF
import astar2
from waypts import *
from vectors import *

compass_bias = Rcompass_bias
azimuth = Rcompass_bias                 # desired course
hdg = Rcompass_bias                     # true compass heading

DODGE = 2.0                             # obstruction dodge distance
steer = 0                               # current steering angle clockwise
speed = 0                               # current speed plus->forward
fhdg = 0                                # Kalman filtered heading
yaw = 0                                 # latest IMU yaw (True north)reading
travel = 0                              # odometer
cogBase = 0                             # course over ground distance
nogpsflag = False                       # gps looks current
approach_factor = .75                   # after waypoint slowdown
resume_speed = speed
reducedflag = False
countheartbeat = 0                      # count of tardy heartbeats events
countgpstardy = 0                       # count of gps readings late events
countgpsrepeat = 0                      # count of gps stuck events
countgpsacc = 0                         # count of gps innacurate events
epoch = time.time()
starttime = epoch
gpsEpoch = epoch
oldEpoch = epoch
heartbeat = epoch
tensecepoch = epoch

pan = 0

oldsteer = 500                          # big values to trigger update
oldspeed = 500
oldhdg = 500
# compass_bias = 98                     # for canopy table (using gyro/quat, no-mag

# all vectors in US Survey feet, AV - 34N14 by 119W04 based, RV - relative
aimRV = [0.0, 0.0]                      # aim point
cogAV = [0.0, 0.0]                      # cogBase starting point
destAV = [0.0, 0.0]                     # waypoint destination
estAV = [0.0, 0.0]                      # est pos either posAV or posAV+DR
filterRV = [0.0, 0.0]                   # Kalman filtered loc
pathRV = [0.0, 0.0]                     # from present pos to wpt end
posAV = [0.0, 0.0]                      # gps position
prevAV = [0.0, 0.0]                     # gps position previous sampling
startAV = [0.0, 0.0]                    # waypoint track start
trackRV = [0.0, 0.0]                    # waypoint path from initial position to destination
workAV = [0.0, 0.0]                     # Kalman filtered AV

ilatsec = 0.0
ilonsec = 0.0
gotolat = 0.0
gotolon = 0.0
latitude = math.radians(34.24)          # Camarillo
latfeet = 6079.99/60                    # Kyle's converter

lonfeet = -latfeet * math.cos(latitude)
accgps = 0.0                            # grps accuracy in ft
segstart = time.time()                  # speed segment start (seconds)
spdfactor = Rspeed                      # fps @ 1%

left = False
left_limit = -36
right_limit = 36

auto = False
flag = False
wptflag = False
wpt = 0
rtseg = 0
route = [0]

routes = [[0,0],                    #0
    [28, 0],                        # <<<<<<<<< RTB >>>>>>>>>>
    [28, 30, 29, 32, 33, 34, 35, 36, 37, 38, 39, 42, 43, 27, 28, 0],  #2
    [28, 30, 29, 31, 27, 28, 0],        #3 - E.F. meander
    [14, 15, 16, 22, 18, 22, 16, 15, 14, 13, 0], #4 - to hut #4 and back
    [0]]
          
wptdist = 0.0
ndx = 0

log = open("logfile.txt", 'w')
robot = motor_driver_ada.motor_driver_ada(log)
log.write("====================================================================")
version = "Rover 1.1 220524\n"
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

#====================================================
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
        print(i)
        w =  waypts[i]
        if w[0] < 0:         # real waypoint?, long. should be neg.
            testdist = vmag(vsub(w, V))
            if testdist < lowdist:
                lowdist = testdist
                lowwp = i
    return lowwp, lowdist
# =================================================================
# from present pos (AV) to destination (AV), return safest,shortest route
def bestroute(pos, dest):
    rte = []
    startwp, startdist = vclosestwp(pos)
    endwp, enddist = vclosestwp(dest)
    # if near to a known path, use one of it's path's wpts although
    # closest actual waypt might be shorter. paths are safer
    wp1, wp2 = astar2.nearpath(pos)
    if (wp1 != 0):  # path(s) close by, two waypt candidates each
        dist1, route1 = astar2.astar(wp1, endwp)
        dist2, route2 = astar2.astar(wp2, endwp)
        if (dist1 < dist2): # which of the two wpts is closer
            startwp = wp1
            rte = route1
        else:   # not near any path
            startwp = wp2
            rte = route2
        startdist = vmag(vsub(pos, waypts[startwp]))
    else: # overland path
        dist, rte = astar2.astar(startwp, endwp)
        
    if startdist < 3.0:           # if very close to starting point
        rte.pop(0)
    return rte
    
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
def advance_waypoint():
    global auto
    global reducedflag
    global route
    global rtseg
    global speed
    global wpt
    global wptflag
    global startAV
    
    rtseg += 1
    wpt = route[rtseg]
    if (wpt == 0):
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
#============================ pan/tilt camera
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
    global wptflag
    global compass_bias
    global left
    
    if (schr == '0'):                   #standby
        auto = False
        wptflag = False
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
        pass
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
        pass
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
        logit("diagnostic #1 ===============================")
        volts = robot.battery_voltage()
#         xchr = "{b%5.1f}" % volts
#         sendit(xchr)
        print("Voltage = ",volts)
        log.write("Voltage: %5.1f\n" % volts)
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
            # if (xchr != 'O'):             # don'tshow compass input
            if (xchr != 'O' and xchr != 'Q'): # don'tshow heartbeat input
                tt = datetime.datetime.now()
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
                            cstr = "startwp, dist %d, %5.2f " % (startwp, startdist)
                            logit(cstr)
                            
                            if wpt == 1:                    # <<<<<<< RTB >>>>>>>>>
                                route = bestroute(posAV, waypts[75])
                                
                            elif (wpt > 1 and wpt < 5):   # start of route
                                rtewp = routes[wpt][0]    # 0th wpt in route
                                dist, route = astar2.astar(startwp, rtewp) # goto start of route
                                route2 = routes[wpt]
                                route2.pop(0)              # delete common wpt
                                route += route2
                                
                            elif (wpt >= 10 and wpt <= 76): #start of waypoint
                                route = bestroute(posAV, waypts[wpt])
                                
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
                                        dist = pldistance(posAV, rwpts0, rwpts1)
                                        if dist < 3.0:
                                            route.pop(0)
                                            logit("track is near, advancing route")

                                route.append(0)
                                for rt in route:
                                    cstr = "route: %d" % rt
                                    logit(cstr)
                                wpt = route[0]
                                rtseg = 0
                                startAV = posAV
                                new_waypoint(wpt)
#                               obstructions()
                        
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
                            nogpsflag = False
                            if prevAV == posAV:
                                if speed == 0:
                                    nogpsflag = True
                                else:
                                    prevAV = posAV
                        elif xchr == 'A':
                            accgps = x * .00328084   #cvt mm to feet
                            if (accgps < 50):
                                cstr = "{la%5.2f}" % accgps    #send to controller
                                sendit(cstr)
                                logit(cstr)
                                if accgps < 3:
                                    nogpsflag = True
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

#===========================================================================
                elif xchr == 'Q':                   # Q - heartbeat
#                     print("heartbeat")
                    heartbeat = time.time()

#===========================================================================
#               {S1 P/S <dist> , <CW angle>} BOT 4:16
#               sensor #1 - TFmini giving obstacle main side and avoidence angle
#               'P'/'S' - port/starboard - side of obstruction
                elif xchr == 'S':             # sensor data
                    if (auto and wptflag):
                        xchr = cbuff[3]           # Port / Starboard
                        args = cbuff[4:msglen-1]
                        sdist, sang, cswath = args.split(',')
                        dist = float(sdist)
                        ang = int(sang)
                        swath = int(cswath)
                        dtg = vmag(pathRV)
                        obsUV = vcompass(hdg + ang)
                        obsRV = vsmult(obsUV, dist)
                        vprint("obstacleV", obsRV)
                        # if dist > dtg:
                        #   advance_waypoint()
                        # or get next wpt aiming vector and see if it dodges
                        #if dist < (dtg + 3.0):  # if obs closer than wpt & 3ft margin
                        if (dtg < 3.0 and dist > dtg):
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
                                
                                if xchr == 'S':      # if obs is mainly right of aimRV
                                    if not rot:
                                        dodgeRV = vsmult(dodgeUV, wdist + DODGE)# + intrusion
                                if (xchr == 'P'):
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
                                cstr = "{r%d,%d,%d} " % (dist, hdg+ang, swath)
                                sendit(cstr)
#===========================================================================
                elif xchr == 'T':                   #'D' key + number button Diagnostic
                    xchr = cbuff[2]
                    diag_commands(xchr)
#=========================================================================                    
                else:
                    pass
            flag = False
            cbuff = ""
            # endif flag
            
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

            if (auto):
                     
                if wptflag:
                    v = speed * spdfactor
                    phi = math.radians((450-hdg) % 360)
                    estAV = posAV
                    # if no recent GPS or (no GPS movement when speed > 0)
                    # then do dead reconning
                    if (gpsEpoch < oldEpoch) \
                        or (nogpsflag and speed > 0) \
                        or accgps > 4.0:   # no recent GPS lat/lon
                        
                        delt = epoch - oldEpoch
                        dist = delt * v
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
                        logit("Dead Reconning =========================")
                    else:    
                        vprint("posAV", posAV)
                        
#                    logit("time: " + str(epoch))
                    logit("wpt: %2d raw hdg: %6.1f" % (wpt, hdg))
                    logit("raw speed: %5.2f" % v)
                    '''
                    workAV = estAV
                    '''
                    xEst = Kfilter.Kalman_step(epoch, estAV[0], estAV[1], phi, v)
                    fhdg = int((450 - math.degrees(xEst[2, 0])) % 360)
                    workAV = [xEst[0, 0], xEst[1, 0]]   # see BOT 3:41 for diagram
                    filterRV = vsub(workAV, startAV)
                    vprint("Kalman pos vec", filterRV)
                    vprint("filtered E-N pos: ", workAV)
                    logit("Filtered hdg: %6.1f" % fhdg)
                    logit("Filtered speed: %6.2f" % xEst[3, 0])
#
                    cstr = "{ln%5.1f} " % workAV[0]
                    sendit(cstr)
                    cstr = "{lt%5.1f} " % workAV[1]
                    sendit(cstr)
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

                    if (dtg < 8.0):
                        if (not reducedflag):
                            resume_speed = speed
                            odometer(speed)
                            speed = int(approach_factor * speed)
                            reducedflag = True
                        resume_speed = max(resume_speed, speed)           
                        
                    #closing on waypoint
                    if (dtg < 2.0 or vdot(pathRV, trackRV) <= 0):
                        logit("close to waypoint")
                        advance_waypoint()

                        #endif dtg ===================
                    #endif wptflag ===================
                
                if (steer >= -1 and steer <= 1 and speed > 50 and nogpsflag):
                    if cogBase > 10:                 # line long enough to compute heading
                        cogBaseRV = vsub(posAV, cogAV) ############# estAV?????
                        hdg = vcourse(cogBaseRV)
                        oldhdg = hdg
                        vprint("COG base course", cogBaseRV)
                        oldbias = compass_bias
                        newbias = (hdg - yaw) % 360   #beware zero crossing
                        if (newbias > oldbias):
                            compass_bias += 1
                        elif (newbias < oldbias):
                            compass_bias -= 1
                            
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
                    
                steer = int(azimuth - hdg)
                logit("new hdg steering")
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
                
            tt = datetime.datetime.now()
            ts = tt.strftime("%H:%M:%S.%f")[:-3]
            path.write("%12s,%9.2f,%8.2f,%8.2f,%8.2f,%8.2f,%4d,%4d,%4d,%5.2f\n" % \
                (ts,epoch-starttime,posAV[0],posAV[1],workAV[0],workAV[1],speed,steer,hdg,accgps))
            path.flush()

            #endif epoch timer ===================
               
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
    logit("innacurate gps: %5d" % countgpsrepeat)

    log.close()
    path.close()
    cstr = "{aStop}"
    sendit(cstr)
    print("Stopped")


import math

def vsub(headV, tailV):
    x = headV[0] - tailV[0]
    y = headV[1] - tailV[1]
    return [x, y]
def vmag(V):
    mag = math.sqrt(V[0]*V[0] + V[1]*V[1])
    return mag
def vcourse(V):
    return (450-math.degrees(math.atan2(V[1],V[0]))% 360)
def vdot(U, V):
    return (U[0]*V[0] + U[1]*V[1])
def vmult(V, scalar):
    return [V[0]*scalar, V[1]*scalar]

Camarillo = 34.238
latfeet = 6076/60
lonfeet = -latfeet*math.cos(math.radians(Camarillo))

startlat = 22.0615
startlon = 7.3213

destlat = 21.491
destlon = 7.646

startV=[startlon*lonfeet, startlat*latfeet]
destV=[destlon*lonfeet, destlat*latfeet]
trackV=vsub(destV, startV)
print("trackV (V) ",trackV)
dist = vmag(trackV)
print(dist)
crse = vcourse(trackV)
print(crse)

pointlat = 22.0111
pointlon = 7.4639
pointV = [pointlon*lonfeet, pointlat*latfeet]
posV = vsub(pointV, startV)
print("posV (U) ",posV)
dot = vdot(trackV, posV)
print(dot)
trk = dot/dist
print("trk: ", trk)
progV = vmult(trackV, trk/dist)
print("progress (W) ", progV)
prog = vmag(progV)/dist
print("progress % ", prog)
xtrackV = vsub(progV, posV)
print("xtrackV: ", xtrackV)
xtrk = vmag(xtrackV)
print("xtrack: ",xtrk)
aim = (1.0 - prog) / 2 + prog
print("aim: ",aim)
aimV = vmult(trackV, aim)
print("aimV: ",aimV)
targV = vsub(aimV, posV)
print("targetV: ", targV)
newhdg = vcourse(targV)
print("new course: ", newhdg)


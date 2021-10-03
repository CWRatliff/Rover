import math
#====================================================
def cartesian(compass):
    return (450 - compass) % 360
def vadd(U, V):
    return [U[0]+V[0], U[1]+V[1]]
def vcross2(U, V):
    x = U[1] * V[0] - U[0] * V[1]
    if x > 0:
        return True      # U x V right hand rotation
    return False
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
# get unit vector from compass heading
def vcompass(angle):
    cart = cartesian(angle)
    print("cart angle", cart)
    rcart = math.radians(cart)
    return [math.cos(rcart), math.sin(rcart)]
# cvt lat/lon seconds to U.S survey feet
def vft2sec(feetE, feetN):
    return [feetN/latfeet, feetE/lonfeet]
# cvt US feet to lat/lon seconds
def vsec2ft(latsec, lonsec):
    return [lonsec*lonfeet, latsec*latfeet]

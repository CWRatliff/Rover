import math
#====================================================
def cartesian(compass):
    return (450 - compass) % 360
def vadd(U, V):
    return [U[0]+V[0], U[1]+V[1]]
def vcross2(U, V):
    x = U[0] * V[1] - U[1] * V[0]
    if x > 0:
        return True      # U x V right hand rotation
    return False
def vdist(U, V):
    W = vsub(U, V)
    return vmag(W)
def vdot(U, V):
    return (U[0]*V[0] + U[1]*V[1])
def vproj(U, V):
    d = vdot(U, V)
    return vsmult(V, d/vmag(V))
def vprojunitv(U, UV):
    return vsmult(UV, vdot(U, UV))
def vmag(V):
    return math.sqrt(V[0]*V[0] + V[1]*V[1])
def vsmult(V, scalar):
    return [V[0]*scalar, V[1]*scalar]
def vsub(headV, tailV):
    return [headV[0]-tailV[0], headV[1]-tailV[1]]
def vunit(V):
    mag = vmag(V)
    return [V[0]/mag, V[1]/mag]
# compute absolute distance from point to line
# see BOT 3:51
def pldistance(P, U, V):
    m = (U[1] - V[1]) / (U[0] - V[0])
    c = V[1] - m * V[0]
    dst = (m * P[0] - P[1] + c)/math.sqrt(m*m + 1)
    dst = abs(dst)
    return dst

# get compass course from direction vector
def vcourse(V):
    return (450 - math.degrees(math.atan2(V[1],V[0]))) % 360
# get unit vector from compass heading
def vcompass(angle):
    cart = cartesian(angle)
    print("cart angle", cart)
    rcart = math.radians(cart)
    return [math.cos(rcart), math.sin(rcart)]


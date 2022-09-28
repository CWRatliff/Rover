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

def vcross_k(U):               # returns vector:k X U
    return [-U[1], U[0]]

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

def two_point_formula(p1, p2): #returns [a, b, c] to wit: ax+by=c
    xa = p2[0] - p1[0]
    ya = p2[1] - p1[1]
    return([-ya, xa, xa*p1[1] - ya*p1[0]])

def crammer_2d(l1, l2):        # returns point: intersection of two lines
    denom = l1[0]*l2[1] - l2[0]*l1[1]
    if denom == 0:
        denom = 1
    x = l1[2]*l2[1] - l1[1]*l2[2]
    y = l1[0]*l2[2] - l1[2]*l2[0]
    return [x/denom, y/denom]

# from wikipedia "line-line_intersection"
# also https://observablehq.com/@toja/line-bos-intersection
# t - scale of linea, u - scale of lineb, t & u >=0 && <= 1 : line seg intersection
# 0 <= t <= 1 and 0 <= u <= 0 => line segment intersection
# point == [0,0] => no intersection (parallel)
def line_intersect(pa1, pa2, pb1, pb2):
    a = pa1[0] - pb1[0] # x1-x3
    b = pb1[0] - pb2[0] # x3-x4
    c = pa1[0] - pa2[0] # x1-x2
    d = pa1[1] - pb1[1] # y1-y3
    e = pb1[1] - pb2[1] # y3-y4
    f = pa1[1] - pa2[1] # y1-y2
    den = c * e - f * b
    if den == 0:
        return ([0, 0], 0, 0)
    t = (a * e - d * b) / den
    u = (a * f - d * c) / den
#    return([pb1[0] + u*(pb2[0]-pb1[0]), pb1[1] + u*(pb2[1]-pb1[1])], t, u)
    return([pa1[0] + t*(pa2[0]-pa1[0]), pa1[1] + t*(pa2[1]-pa1[1])], t, u)


           
    

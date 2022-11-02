from vectors import *
import math
'''
a0 = [3.0, 0.0]
a1 = [9.0, 0.0]
b0 = [5.0, 3.0]
#b0 = [0.0, 3.0]
#b1 = [8.0, 3.0]
b1 = [8.0, 8.0]
'''
p0 = [-593.08, 2132.60]
p1 = [-576.59, 2136.79]
q0 = [-598.46, 2124.46]
q1 = [-577.12, 2133.53]

# distance and direction between two line segments
def segdist(a0, a1, b0, b1):
    v = vsub(a1, a0)
    u = vsub(b1, b0)
    w = vsub(b0, a0)
    x = vsub(b1, a1)
    wv = vdot(w, v)
    vx = -vdot(v, x)   # reverse direction of v
    if wv <= 0 and vx <= 0:      # case 1 a line within b line
        dist1 = pldistance(a0, b0, b1)
        dist2 = pldistance(a1, b0, b1)
        perp = vperp(u)
    elif wv > 0 and vx > 0:      # case 2 b line within a line
        dist1 = pldistance(b0, a0, a1)
        dist2 = pldistance(b1, a0, a1)
        perp = vperp(v)
    elif wv > 0 and vx < 0:      # case 3 offset lines right
        dist1 = pldistance(b0, a0, a1)
        dist2 = pldistance(a1, b0, b1)
        if vmag(v) > vmag(u):
            perp = vperp(v)
        else:
            perp = vperp(u)
    else:                        # case 4 offset lines left
        dist1 = pldistance(b1, a0, a1)
        dist2 = pldistance(a0, b0, b1)
        if vmag(v) > vmag(u):
            perp = vperp(v)
        else:
            perp = vperp(u)

    dist = min(dist1, dist2)
    return(dist, perp)

sdist, sdir = segdist(p0, p1, q0, q1)
print("pitch", math.degrees(math.atan(1/sdist)))
print("heading", vcourse(sdir))


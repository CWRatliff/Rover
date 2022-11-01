from vectors import *
'''
a0 = [3.0, 0.0]
a1 = [9.0, 0.0]
b0 = [5.0, 3.0]
#b0 = [0.0, 3.0]
#b1 = [8.0, 3.0]
b1 = [8.0, 8.0]
'''
a0 = [-593.08, 2132.60]
a1 = [-576.59, 2136.79]
b0 = [-598.46, 2124.46]
b1 = [-577.12, 2133.53]

v = vsub(a1, a0)
u = vsub(b1, b0)
w = vsub(b0, a0)
x = vsub(b1, a1)
print("v", v)
print("u", u)
print("w", w)
print("x", x)
wv = vdot(w, v)
vx = -vdot(v, x)   # reverse direction of v
print(wv, vx)
if wv <= 0 and vx <= 0:      # case 1 a line within b line
    dist1 = pldistance(a0, b0, b1)
    dist2 = pldistance(a1, b0, b1)
    print("line b outside line a", dist1, dist2)
    perp = vperp(u)
elif wv > 0 and vx > 0:      # case 2 b line within a line
    dist1 = pldistance(b0, a0, a1)
    dist2 = pldistance(b1, a0, a1)
    print("line b inside line a", dist1, dist2)
    perp = vperp(v)
elif wv > 0 and vx < 0:      # case 3 offset lines right
    dist1 = pldistance(b0, a0, a1)
    dist2 = pldistance(a1, b0, b1)
    print("line b alongside line a", dist1, dist2)
    if vmag(v) > vmag(u):
        perp = vperp(v)
    else:
        perp = vperp(u)
else:                        # case 4 offset lines left
    dist1 = pldistance(b1, a0, a1)
    dist2 = pldistance(a0, b0, b1)
    print("line b beside line a", dist1, dist2)
    if vmag(v) > vmag(u):
        perp = vperp(v)
    else:
        perp = vperp(u)

    
dist = min(dist1, dist2)

print(1/dist, vunit(perp))
print(vcourse(perp))


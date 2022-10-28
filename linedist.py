from vectors import *
a0 = [3.0, 0.0]
a1 = [9.0, 0.0]
b0 = [5.0, 3.0]
#b0 = [0.0, 3.0]
#b1 = [8.0, 3.0]
b1 = [8.0, 8.0]

v = vsub(a1, a0)
u = vsub(b1, b0)
w = vsub(b0, a0)
x = vsub(b1, a1)
print("v", v)
print("u", u)
print("w", w)
print("x", x)
wv = vdot(w, v)
vx = -vdot(v, x)
print(wv, vx)
if wv < 0 and vx < 0:      # both angles obtuse
    dist1 = pldistance(b1, a0, a1)
    dist2 = pldistance(b0, a0, a1)
    dist = min(dist1, dist2)
    print("line b outside line a", dist)
if wv > 0 and vx > 0:      # both angles acute
    dist1 = pldistance(a0, b1, b0)
    dist2 = pldistance(a1, b1, b0)
    dist = min(dist1, dist2)
    print("line b inside line a", dist1, dist2, dist)
if wv > 0 and vx < 0:
    dist1 = pldistance(b0, a0, a1)
    dist2 = pldistance(a1, b0, b1)
    dist = min(dist1, dist2)
if wv < 0 and vx > 0:
    dist1 = pldistance(b1, a0, a1)
    dist2 = pldistance(a0, b0, b1)
    dist = min(dist1, dist2)

print(dist)

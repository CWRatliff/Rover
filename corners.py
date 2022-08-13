# given a polygon, compute points for new corners .5 feet from sides
# assume points in CCW order
# 220811
from vectors import *
MINLEG = 5.0

waypts = [[0,0],[0,0], [-600.0, 2100.0], [-650.0, 2150.0], [-675.0, 2100.0], [0., 0.]]
# corner0 = [-600.0, 2100.0]
# corner1 = [-650.0, 2150.0]
# corner2 = [-675.0, 2100.0]
# clist = [corner0, corner1, corner2]
clist = []

for i in range(3):
    if waypts[i+2][0] == 0:
        break
    clist.append(waypts[i+2])
print(clist)
    
clen = len(clist)

for cndx in range(0, clen):
    cmod0 = (cndx-1) % clen
    cmod1 = (cndx) % clen
    cmod2 = (cndx+1) % clen
    print(cmod0, cmod1, cmod2)
    
    leg1 = vsub(clist[cmod1], clist[cmod0])
    print("leg1", leg1)
    offset1 = vcross_k(leg1)
    offset1 = vunit(offset1)
    offset1 = vsmult(offset1, 0.5)
    print("offset1", offset1)

    leg2 = vsub(clist[cmod1], clist[cmod2])
    offset2 = vcross_k(leg2)
    offset2 = vunit(offset2)
    offset2 = vsmult(offset2, 0.5)
    print("offset2", offset2)

    point1a = vadd(clist[cmod0], offset1)
    point2a = vadd(clist[cmod1], offset1)
    line1 = two_point_formula(point2a, point1a)
    print("line1", line1)

    point1b = vadd(clist[cmod1], offset2)
    point2b = vadd(clist[cmod2], offset2)
    line2 = two_point_formula(point2b, point1b)
    print("line2", line2)

    pivot = crammer_2d(line1, line2)
    print(pivot)

    waypts[cndx+2] = pivot
#    waypts.append(pivot)
print(waypts)
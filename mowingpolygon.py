#comvert polygon lat/lon into GT/GN & AZimuth) commands
import numpy as np
import matplotlib.pyplot as plt
from vectors import *

proppts = [
    [-196.29,  725.43],
    [-139.29,  629.11],
    [-137.47,  580.34],
    [-176.17,  504.14],
    [-190.20,  448.06],
    [-205.13,  481.28],
    [-245.67,  508.71],
    [-243.84,  612.34],
    [-262.74,  703.78],
    [-196.29,  725.43],
    ]
    
housepts = [
    [-158.80,  632.16],
    [-167.95,  627.58],
    [-170.99,  626.67],
    [-176.17,  625.15],
    [-175.57,  623.01],
    [-181.05,  621.49],
    [-182.27,  625.15],
    [-190.20,  622.71],
    [-192.63,  630.94],
    [-184.40,  633.07],
    [-185.93,  637.95],
    [-180.44,  639.47],
    [-179.83,  637.95],
    [-174.65,  639.47],
    [-175.57,  642.52],
    [-165.51,  647.09],
    ]
mow1 = [
    [-240.00, 700.00],
    [-240.00, 675.00],
    [-220.00, 670.00],
    [-200.00, 650.00],
    [-200.00, 675.00],
    [-220.00, 700.00],
    ]

swath = 5 #m for testing   
    
ox, oy = zip(*proppts)
#print(ox)
#print(oy)
#plt.plot(ox, oy, "-xb")
px, py = zip(*housepts)
#plt.plot(ox, oy, "-xr")
#plt.plot(px, py, "-r")
poly = len(mow1)
if (mow1[0] != mow1[-1]):   # complete ring polygon
    mow1.append(mow1[0])
ox, oy = zip(*mow1)
plt.plot(ox, oy, "-g")
plt.axis("equal")
plt.grid(True)
plt.pause(1)
#plt.close()
angs = []
course = []
route = []
for j in range(poly):
    i = (j - 1) % poly
    k = (j + 1) % poly
    u = vsub(mow1[j], mow1[i])
    v = vsub(mow1[k], mow1[j])
    theta = vang(u, v)
    angs.append(theta)
#    print(j)
#    print("c", math.degrees(vangc(u, v)))
#    print("s", math.degrees(vangs(u, v)))
#    print("t", math.degrees(theta))
    print("steer", math.degrees(-theta))
    courz = vcourse(v)
    course.append(courz)
    print("course", courz)
print('{GN%7.2f}' % mow1[0][0])
print('{GT%7.2f}' % mow1[0][1])
print('{AZ%7.2f}' % course[0])
print("-----------")
for j in range(poly):
    i = (j + 1) % poly
    print('{GN%7.2f}' % mow1[i][0])
    print('{GT%7.2f}' % mow1[i][1])
    print('{AZ%7.2f}' % course[i])
# recompute mow for smaller path
for j in range(poly):
    i = (j + 1) % poly
    i = (j - 1) % poly
    k = (j + 1) % poly
    u = vsub(mow1[j], mow1[i])
    v = vsub(mow1[k], mow1[j])
    phi = math.radians(course[i] - 90)
    short = swath / math.sin(phi) #degree:radian problem
    print("new swath", short)
    
# convert lat/lon in degrees to UTM Z11N

# compliments to excelinsder.com
import math
def Z11Nxy(lat, lon):
    a = 6378137
    f = 1/298.257223563
    k0 = 0.9996
    #e = sqrt(2*f - f**2)
    e2 = 2*f - f**2
    e4 = e2 * e2
    e6 = e2 * e4
    #zone = 11
    long0 = -117
    latrad = lat*math.pi/180
    lonrad = lon*math.pi/180
    longrad0 = long0*math.pi/180
    n = a/math.sqrt(1 - e2 * math.sin(latrad)**2)
    t = math.tan(latrad)**2
    c = (e2 / (1 - e2)) * math.cos(latrad)**2
    aterm = math.cos(latrad) * (lonrad - longrad0)
    m = a * ((1 - e2/4 - 3*e4/64 - 5*e6/256) * latrad
        - (3*e2/8 + 3*e4/32 + 45*e6/1024) * math.sin(2*latrad)
        + (15*e4/256 + 45*e6/1024) * math.sin(4*latrad)
        - (35*e6/3072) * math.sin(6*latrad))

    east = k0 * n * (aterm + (1 - t + c) * aterm**3/6 + (5 - 18*t + t**2 + 72*c - 58*e2)
        * aterm**5/120) + 500000
    north = k0 *(m + n * math.tan(latrad) * (aterm**2/2 + (5 - t + 9*c + 4*c**2) * aterm**4/24
        + (61 - 58*t + t**2 + 600*c - 330*e2) * aterm**6/720))                               
    print("east: ",east)
    print("north:",north)
    return north, east

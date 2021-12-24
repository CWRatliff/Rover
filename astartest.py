import astar2
from waypts import *
from vectors import *

def vclosestwp(V):
    global waypts
    wlen = len(waypts)
    lowdist = vmag(vsub(waypts[10], V))
    lowwp = 10
    for i in range(10, wlen):
#        print(i)
        w =  waypts[i]
        if w[0] < 0:         # real waypoint?, long. should be neg.
            testdist = vmag(vsub(w, V))
            if testdist < lowdist:
                lowdist = testdist
                lowwp = i
    return lowwp, lowdist
def bestroute(pos, dest):
    rte = []
    startwp, startdist = vclosestwp(pos)
    endwp, enddist = vclosestwp(dest)
#    print("wpts",startwp, endwp)
    # if near to a known path, use one of it's path's wpts although
    # closest actual waypt might be shorter. paths are safer
    wp1, wp2 = astar2.nearpath(pos)
    if (wp1 != 0):  # path(s) close by, two waypt candidates each
        dist1, route1 = astar2.astar(wp1, endwp)
        print("bestroute1", route1)
        dist2, route2 = astar2.astar(wp2, endwp)
        print("bestroute2", route2)
        if (dist1 < dist2): # which of the two wpts is closer
            startwp = wp1
            rte = route1
        else:   # not near any path
            startwp = wp2
            rte = route2
        startdist = vmag(vsub(pos, waypts[startwp]))
    else: # overland path
        dist, rte = astar2.astar(startwp, endwp)
        
    if startdist < 3.0:           # if very close to starting point
        rte.pop(0)
    return rte
'''
#rte = astar(14, 18)
posAV = [-620, 2240]
gotoAV = [-850, 2290]
#gotoAV = [-620, 2240]
startdist = 1000000
enddist = 1000000
startwp = 33
endwp = 23
i = 0

route = astar.astar(startwp, endwp)
route.append(0)
print(route)
'''
#posAV = [-662, 2270]
#wpt1, wpt2 = astar2.nearpath(posAV)
#print(wpt1, wpt2)
destAV = [ -636.74,  2176.04]
#destAV = [ -640.51,  2179.75]
posAV = [-664.74, 2269.35]
rte = bestroute(posAV, destAV)

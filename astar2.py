# 211012 - total trip calc added

import math
import waypts
from vectors import *
graph = [
    [10,33],[10,34],[10,39],[11,12],[11,36],[11,52],[11,74],[12,17],[12,53],[13,14],[13,28],[13,76],
    [14,21],[14,49],[15,16],[15,21],[16,22],[16,26],[17,20],[18,22],[18,71],[19,20],[19,24],
    [23,48],[23,49],[24,26],[24,67],[26,67],[27,28],[27,41],[27,43],[28,30],
    [28,31],[28,41],[28,76],[29,30],[29,32],[29,44],[29,50],
    [30,50],[32,33],[33,73],[34,35],[35,36],[36,37],[37,38],[37,68],[38,39],[39,40],[39,41],
    [40,41],[40,42],[42,43],[42,51],[44,45],[45,46],[45,73],[46,47],[47,48],[49,50],
    [51,52],[52,74],[53,54],[54,55],[55,56],[56,57],[57,58],[58,59],[59,60],
    [60,61],[61,62],[62,63],[63,64],[64,65],[65,66],[66,67],[68,69],[68,74],[69,70],[69,74],
    [70,71],[75,76]]
    
class Node:
    def __init__(self, parent = None, position = None):
        self.parent = parent
        self.position = position
        self.f = 0    # total cost
        self.g = 0    # step cost
        self.h = 0    # heuristic cost
        
    def pnode(self, what):
        print(what, self.parent, self.position, self.f, self.g, self.h)

def wpeuclidian(a, b):
    eft = waypts.waypts[a][0] - waypts.waypts[b][0]
    nft = waypts.waypts[a][1] - waypts.waypts[b][1]
    # sqrt not really needed
    return (math.sqrt(eft ** 2 + nft **2))
            
def veuclidian(a, b):
    eft = a[0] - b[0]
    nft = a[1] - b[1]
    return (math.sqrt(eft ** 2 + nft **2))
            
def scanlist(nodeno, list1):
    for i in list1:
        if nodeno == i.position:
            return i
    return None

# from "Towards Data Science", "A-Star Search Algorithm", Baijayanta, Roy Sept 29, 2019
def astar(startno, endno):
    Start_node = Node(None, startno)
    open_list = [Start_node]
    closed_list = []
    while len(open_list):
#        open_list.sort()       #prioritize
        min = open_list[0].f
        ino = 0
        indx = 0
        trip = 0
        for i in open_list:
            i.pnode("    open")
            if (i.f < min):
                min = i.f
                ino = indx
            indx = indx + 1
        Current = open_list[ino]
        open_list.pop(ino)
        closed_list.append(Current)
        Current.pnode("new current node")
        if Current.position == endno:   #success
            node = Current
            route = [endno]
            while True:
                parent = node.parent
                if parent == None:
                    break
                trip += node.g
                route.insert(0, parent)
                for nd in closed_list:
                    if nd.position == parent:
                        parent = nd.parent
                        node = nd
 #                       nd.pnode("this path")
                        break
            # search for parent=Current.position in closed
            #for n in closed_list:
            #   print("route = ", n.position)
            print("trip distance", trip)
            return trip, route
        
        # create Sucessor nodes
        for g in graph:          # scan graph for lower nodes
#            print("g, Current",g, Current.position)
            if g[0] == Current.position:
                gcurrno = g[0]       # from nodeno
                gtono = g[1]         # to nodeno
            elif g[1] == Current.position:
                gcurrno = g[1]       # from nodeno
                gtono = g[0]         # to nodeno
            else:
                continue;
            
            if scanlist(gtono, closed_list):
                continue
            Child_node = Node(gcurrno, gtono)
            Child_node.g = wpeuclidian(gcurrno, gtono)
            Child_node.h = wpeuclidian(gtono, endno)
            Child_node.f = Child_node.g + Child_node.h
#                Child_node.pnode("Child Node")
            open_node = scanlist(gtono, open_list)
            if (open_node):
                open_node.pnode("scanned from open")
                if open_node.g > Child_node.g:
                    print("opennode.g > child.g")
                    open_node.parent = Current.position
                    open_node.g = Child_node.g
                    open_node.f = open_node.g + open_node.h
                    open_node.pnode("scanned after")
            else:
                open_list.append(Child_node)
#               print("Child appended")
    return 0, []
#================================================================================
# return two wayptn #s when pos is within 3 feet of path between them
# N.B. might be more than one qualifying path

def nearpath(pos):
    for g in graph:
        u = waypts.waypts[g[0]]
        v = waypts.waypts[g[1]]
        d = pldistance(pos, u, v)
#        print("distance:", d)
        if (d < 3.0):
            print ("wpts:", u, v)
            utov = vsub(u, v)
            utop = vsub(u, pos)
            dot1 = vdot(utov, utop)
            vtou = vsub(v, u)
            vtop = vsub(v, pos)
            dot2 = vdot(vtou, vtop)
            print("dot1&2", dot1, dot2)
            if (dot1 > 0 and dot2 > 0): # within bounds?
                return g[0], g[1]
    return 0, 0

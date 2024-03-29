from tkinter import *
from tkinter.font import Font

# property taken from 2018 WM Survey DWG
proppts = [
    [-644, 2380],
    [-457, 2064],
    [-451, 1904],
    [-578, 1654],
    [-624, 1470],
    [-673, 1579],
    [-806, 1669],
    [-800, 2009],
    [-862, 2309]
    ]
# personal survey 210506
housepts = [
    [-521, 2074],
    [-551, 2059],
    [-561, 2056],
    [-578, 2051],
    [-576, 2044],
    [-594, 2039],
    [-598, 2051],
    [-624, 2043],
    [-632, 2070],
    [-605, 2077],
    [-610, 2093],
    [-592, 2098],
    [-590, 2093],
    [-573, 2098],
    [-576, 2108],
    [-543, 2123]
    ]

backpts = [
    [-648, 2374],
    [-503, 2130],
    [-527, 2148],
    [-657, 2365]
    ]

frontpts = [
    [-807, 2328],
    [-753, 2260],
    [-657, 2113],
    [-666, 2107],
    [-755, 2242],
    [-820, 2321]
    ]

waypts = [
    [ -787.36,  2298.03],     #10 
    [ -647.55,  2108.54],     #11 speed bump
    [ -619.57,  2091.62],     #12 T seam
    [ -599.97,  2236.32],     #13 workshop F 
    [ -578.94,  2247.67],     #14 driveway center F
    [ -490.20,  2097.75],     #15 gravel rev 210430
    [ -471.80,  2053.82],     #16 fig tree fork F
    [ -532.36,  1963.03],     #17 stairs pivot
    [ -592.93,  1931.82],     #18 shed #3/#4 F
    [ -526.33,  1863.82],     #19 longe center
    [ -661.79,  1842.34],     #20 stall ctr
    [ -511.84,  2145.63],     #21 E dway start
    [ -548.45,  1951.78],     #22 hut row bend F
    [ -619.07,  2315.06],     #23 trash
    [ -599.72,  2290.03],     #24 EF east entry
    [ -665.89,  2108.14],     #25 ref corner - F
    [ -646.13,  2126.38],     #26 hose bib - F
    # [ -640.51,  2177.75],     #27 rose bush - F
    [ -640.51,  2179.75],     #27 rose bush - F 210102 modified to avoid roses
    [ -624.85,  2235.41],     #28 boat corner - F 201230 - refounded
    [ -684.91,  2276.04],     #29 EF middle - F
    [ -644.70,  2261.65],     #30 office gap
    [ -653.41,  2229.63],     #31 EF rose gap
    ]
# wpts = []
root = Tk()
root.wm_title('350 VdM Chart')
root.geometry("1024x600+0+0")

ffont = Font(family="URW Chancery L", size=20, weight = "bold")
efont = Font(family="URW Chancery L", size=16)
button1 = Button(root, text="NEWS", font=ffont)
button1.pack()
# convert array of USfeet pairs into linear array of pixels (in pairs)
# slat, slon starting lat/lon usf from 34-14, -119-04
def usf2pix(usf, gscale, slat, slon):
    pix = []
    for ll in usf:
        pix.insert(0, int(gscale * (slon-ll[0])))
        pix.insert(0, int(gscale * (slat-ll[1])))
    return pix

scale = 1.0
stlat = 2400
stlon = -440
mode = 1        #zoom in
mx = 0
my = 0
wpts = usf2pix(waypts, scale, stlat, stlon)

canvas= Canvas(root, width=950, height=430, bg='white')
canvas.pack()

points = usf2pix(proppts, scale, stlat, stlon)
plot = canvas.create_polygon(points, outline='black', fill='green2', width=1)

house = usf2pix(housepts, scale, stlat, stlon)
rez = canvas.create_polygon(house, outline='black', fill='red', width=1)

back = usf2pix(backpts, scale, stlat, stlon)
bdrv = canvas.create_polygon(back, outline='black', fill='gray75', width=2)

front = usf2pix(frontpts, scale, stlat, stlon)
fdrv = canvas.create_polygon(front, outline='black', fill='gray75', width=2)

longe =usf2pix([[-526-25, 1863.82-25],[-526+25, 1863.82+25]], scale, stlat, stlon)
lunge = canvas.create_oval(longe[0], longe[1], longe[2], longe[3], outline='black', fill='gold', width=1)
llen = len(wpts)
for i in range(0, llen, 2):
    canvas.create_rectangle(wpts[i], wpts[i+1], wpts[i]+2, wpts[i+1]+2, fill='blue', outline='blue')
canvas.create_line(100,100,200,200,arrow=LAST,arrowshape=(20,25,5),width=4,fill="blue")
canvas.create_text(220, 220, text="NEWS", font = ffont, angle=90)
canvas.create_text(275, 275, text = "NW", font = efont, angle=315)
# enlarge button array ===================================================
zoom = Frame(root)
zoom.place(x=975, y=200)
fmax = Button(zoom, text = "+", command = lambda:scaler(1))
fmax.config(width = 1, height = 1, font=(NONE,15), bg="green2",fg="black",borderwidth=2)
fmax.grid(row=0,column=0)

rmax = Button(zoom, text = "-", command = lambda:scaler(-1))
rmax.config(width = 1, height = 1, font=(NONE,15), bg="pink",fg="black",borderwidth=4)
rmax.grid(row=1,column=0)
        
rmax = Button(zoom, text = "X", command = lambda:scaler(0))
rmax.config(width = 1, height = 1, font=(NONE,15), bg="blue",fg="black",borderwidth=4)
rmax.grid(row=2,column=0)
        
def scaler(scl):
    global mode
    mode = scl
#     print (mode)
def bpress(event):
    global mx
    global my
    mx = event.x
    my = event.y
    print('button press')
    
    
def mouse(event):
    global mx
    global my
    global stlat
    global stlon
    if (mode == 0):
        return
    if (mx == 0 and my == 0):
        mx = event.x
        my = event.y
    x = event.x - mx
    stlat += x/scale
    y = event.y - my
    stlon += y/scale
    
    print("move")
    print(str(event.x)+" "+str(event.y))
    print(str(int(stlat))+" "+str(int(stlon)))
    canvas.move(plot, x, y)
    canvas.move(rez, x, y)
    canvas.move(bdrv, x, y)
    canvas.move(fdrv, x, y)
    mx = event.x
    my = event.y

def taptap(event):
    global stlat
    global stlon
    global scale
    global rez
    global plot
    if (mode == 0):
        lat = stlat - event.x/scale
        lon = stlon - event.y/scale
        print (str(lat)+"/"+str(lon))
        pnt = usf2pix([[lon, lat]], scale, stlat, stlon)
        canvas.create_rectangle(pnt[0], pnt[1], pnt[0]+4, pnt[1]+4, fill='blue',outline='blue')
        return
    if mode > 0:
        stlat = stlat - (event.x/scale)/3
        stlon = stlon - (event.y/scale)/3
        scale *= 1.5
    else:
        stlat = stlat + (event.x/scale)/2
        stlon = stlon + (event.y/scale)/2
        scale *= 2/3
        
    print("zoom")
    print(str(event.x)+" "+str(event.y))
    print(str(int(stlat))+" "+str(int(stlon)))
    canvas.delete(rez)
    canvas.delete(plot)
    points = usf2pix(proppts, scale, stlat, stlon)
    plot = canvas.create_polygon(points, outline='black', fill='green2', width=1)

    house = usf2pix(housepts, scale, stlat, stlon)
    rez = canvas.create_polygon(house, outline='black', fill='red', width=1)

# def done(event):
#     pass
    
canvas.bind('<B1-Motion>', mouse)
canvas.bind('<Double-Button-1>', taptap)
canvas.bind('<Button-1>', bpress)

root.mainloop()

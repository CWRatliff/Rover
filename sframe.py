# property taken from 2018 WM Survey DWG
# 2 Dec 21 revised horse bldgs
# 211208 - goto spot work

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

# monopod survey 5/27/21
horsepts = [
    [-469.81, 1928.73],
    [-488.51, 1903.72],
    [-517.62, 1890.01],
#    [-572.02, 1876.52],
    [-529.04, 1902.04],
    [-551.43, 1901.93],
    [-560.83, 1895.85],
    
    [-590.65, 1861.59],
    [-617.51, 1854.86],
    [-643.08, 1855.40],
    [-679.72, 1863.53],
    [-716.17, 1880.55],
    [-743.18, 1911.50],
    [-766.27, 1947.05],
    [-780.30, 1989.13],
    [-786.57, 2044.97],
    [-767.28, 2079.82],
    [-758.68, 2087.74],
    [-747.62, 2103.76],
    [-742.69, 2117.81],
    [-751.92, 2147.04],
    [-786.90, 2139.61],
    [-795.40, 2140.37],
    [-801.76, 2148.42],
    [-814.63, 2169.50],
    [-829.34, 2163.91],
    [-818.83, 2113.80],
    [-800.86, 2037.84],
    [-794.34, 1966.27],
    [-787.11, 1948.70],
    [-766.87, 1913.13],
    [-770.79, 1902.29],
    [-781.41, 1888.04],
    [-785.35, 1873.30],
    [-777.48, 1872.48],
    [-770.32, 1886.67],
    [-758.64, 1899.47],
    [-730.45, 1867.07],
    [-707.87, 1845.26],
    [-697.78, 1826.98],
    [-691.52, 1822.06],
    [-672.97, 1816.98],
    [-644.24, 1812.56],
    [-615.23, 1813.15],
    [-566.84, 1823.47],
    [-543.23, 1834.23],
    [-519.25, 1850.02],
    [-507.06, 1866.63],
    [-500.59, 1880.48],
    [-468.79, 1903.69],
    [-459.34, 1928.01]
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
hut1 = [
    [-639.91,1955.08],
    [-642.17,1945.43],
    [-630.83,1942.62],
    [-628.49,1952.45]
    ]
hut2 = [
    [-620.55,1951.13],
    [-613.23,1950.71],
    [-613.88,1943.62],
    [-621.21,1944.38]
    ]
hut3 = [
    [-604.58,1951.39],
    [-597.77,1952.87],
    [-596.32,1945.91],
    [-603.32,1944.72]
    ]
hut4 = [
    [-587.80,1954.31],
    [-581.14,1955.12],
    [-580.05,1948.40],
    [-586.86,1947.46]
    ]
lhouse = [
    [-583.55,2182.98],
    [-571.94,2189.43],
    [-576.49,2197.60],
    [-588.09,2191.15]
    ]
horsecanopy = [
    [-582.28,2219.30],
    [-596.56,2223.87],
    [-604.27,2197.31],
    [-590.79,2192.48]
    ]
workshop = [
    [-612.08, 2226.97],
    [-617.51, 2207.77],
    [-607.15, 2204.32],
    [-599.91, 2222.88]
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

trailergar = [
    [-754.93, 2143.08],
    [-778.77, 2135.18],
    [-770.63, 2111.26],
    [-747.24, 2119.04]
    ]

shedrow = [
    [-684.24, 1858.56],
    [-648.19, 1848.22],
    [-655.13, 1823.44],
    [-691.00, 1833.00]
    ]

# route = [28, 30, 29, 32, 33, 34, 35, 36, 37, 38, 39, 42, 43, 27, 28]
route = [28, 30, 29, 31, 27, 28]        #3 - E.F. meander

from tkinter import *
from tkinter.font import Font
import serial
import RPi.GPIO as GPIO
import math
import ctypes
import time

cdlib = ctypes.CDLL("/home/pi/projects/ctypes/libsapphire.so")
cdlib.COpenTable.restype = ctypes.c_void_p
cdlib.COpenIndex.restype = ctypes.c_void_p
cdlib.CGetDouble.restype = ctypes.c_double
cdlib.CGetCharPtr.restype = ctypes.c_char_p

GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP) # green
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP) # black
GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_UP) # red
GPIO.setup(7, GPIO.IN, pull_up_down=GPIO.PUD_UP) # blue
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_UP) # yellow

ser = serial.Serial(port='/dev/ttyS0',      #xbee to rover
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
    )
rhdg = math.radians(450 - 98)       # x-y initial heading to compass
# bwidth = 0
# bhdg = 0
# bdist = 0
bwidth = 6
bhdg = 320
bdist = 12.7
strhdg = 0
track = []
lat = 0.0
lon = 0.0
pathflag = False
butngreen = False
butngreen2 = False
greenepoch = time.time()
butnred = False
butnred2 = False
redepoch = time.time()
butnblack = False
blackepoch = time.time()
butnblue = False
butnyellow = False
arrlen = 75
scale = 1.0
gotolatlon = []
stlat = 2400
stlon = 950
mode = 0        # 0:move, 2:zoom in, 3:zoom out, 4:goto
mx = 0
my = 0

# ==============================================================================
try:
    pathfile = open("path.txt", 'r')
    pathflag = True
    
except IOError:
    pass

def Chart(mstr):
    global arena
    global bdrv
    global charter
    global canvas
    global fdrv
    global lunge
    global plot
    global rez

    points = Usf2Pix(proppts, scale, stlat, stlon)
    plot = canvas.create_polygon(points, outline='black', fill='lemon chiffon', width=1)
    horse = Usf2Pix(horsepts, scale, stlat, stlon)
#    arena = canvas.create_polygon(horse, outline='black', fill='gold', width=1)
    arena = canvas.create_polygon(horse, outline='black', fill='gray75', width=1)
    longe =Usf2Pix([[-526-25, 1863.82-25],[-526+25, 1863.82+25]], scale, stlat, stlon)
    lunge = canvas.create_oval(longe[0], longe[1], longe[2], longe[3], outline='black', \
        fill='gold', width=1)
    
    house = Usf2Pix(housepts, scale, stlat, stlon)
    rez = canvas.create_polygon(house, outline='black', fill='red', width=1, tags="bldg")
    h1 = Usf2Pix(hut1, scale, stlat, stlon)
    canvas.create_polygon(h1, outline='black', fill='red', width=1, tags="bldg")
    h2 = Usf2Pix(hut2, scale, stlat, stlon)
    canvas.create_polygon(h2, outline='black', fill='red', width=1, tags="bldg")
    h3 = Usf2Pix(hut3, scale, stlat, stlon)
    canvas.create_polygon(h3, outline='black', fill='red', width=1, tags="bldg")
    h4 = Usf2Pix(hut4, scale, stlat, stlon)
    canvas.create_polygon(h4, outline='black', fill='red', width=1, tags="bldg")
    casita = Usf2Pix(lhouse, scale, stlat, stlon)
    canvas.create_polygon(casita, outline='black', fill='red', width=1, tags="bldg")
    canopy = Usf2Pix(horsecanopy, scale, stlat, stlon)
    canvas.create_polygon(canopy, outline='black', fill='red', width=1, tags="bldg")
    work = Usf2Pix(workshop, scale, stlat, stlon)
    canvas.create_polygon(work, outline='black', fill='red', width=1, tags='bldg')
    tgar = Usf2Pix(trailergar, scale, stlat, stlon)
    canvas.create_polygon(tgar, outline='black', fill='gray50', width=1, tags='bldg')
    shed = Usf2Pix(shedrow, scale, stlat, stlon)
    canvas.create_polygon(shed, outline='black', fill='gray50', width=1, tags='bldg')
    
    back = Usf2Pix(backpts, scale, stlat, stlon)
    bdrv = canvas.create_polygon(back, outline='black', fill='gray75', width=1)
    front = Usf2Pix(frontpts, scale, stlat, stlon)
    fdrv = canvas.create_polygon(front, outline='black', fill='gray75', width=1)
    
    trees = Usf2Pix(alltrees, scale, stlat, stlon)
    llen = len(trees)
    rad = scale * 3
    for i in range(0, llen, 2):
        canvas.create_oval(trees[i]-rad, trees[i+1]-rad, trees[i]+rad, trees[i+1]+rad, \
            fill='green2', outline='black', width=2, tags = 'forest')
        
    wpts = Usf2Pix(waypts, scale, stlat, stlon)
    llen = len(wpts)
    for i in range(0, llen, 2):
        canvas.create_rectangle(wpts[i], wpts[i+1], wpts[i]+2, wpts[i+1]+2, fill='blue', outline='blue', tags = "wpts")
        canvas.create_text(wpts[i]+2, wpts[i+1], text = str(i//2+10), fill='red', tags = "wpts", font = nfont)
        
    rtpts = []
    for i in route:
        rtpts.append([waypts[i-10][0], waypts[i-10][1]])
    rtlst = Usf2Pix(rtpts, scale, stlat, stlon)
    llen = len(rtlst)
    rline = canvas.create_line(rtlst, fill = 'red', tags = "wpts")
    
    tracks = Usf2Pix(track, scale, stlat, stlon)
    llen = len(tracks)
    for i in range(0, llen, 2):
        canvas.create_text(tracks[i], tracks[i+1], text='x', fill='blue', tags = 'path')

def Xspot(mstr, xlon, xlat):
    spot = Usf2Pix([[xlon, xlat]], scale, stlat, stlon)
    canvas.create_text(spot[0], spot[1], text='.', fill='blue', tags = 'path')
          
def Guage(mstr):
    rose.delete('arrow')
    rose.delete('blip')
    xarrow = arrlen * math.cos(rhdg)
    yarrow = arrlen * math.sin(rhdg)
    rose.create_line(240-xarrow, 240+yarrow, 240 + xarrow, 240-yarrow, arrow=LAST, \
        arrowshape=(20,25,5), width=8,fill="deep sky blue", tags="arrow")
    if (strhdg < -1 or strhdg > 1):
        strad = math.radians(strhdg)
        xarrow = arrlen * math.cos(rhdg - strad)
        yarrow = arrlen * math.sin(rhdg - strad)
        if (strhdg < 0):
            rose.create_line(240, 240, 240+xarrow, 240-yarrow, arrow = LAST, \
                width = 3, fill = "red2", tags='arrow')
        else:
            rose.create_line(240, 240, 240+xarrow, 240-yarrow, arrow = LAST, \
                width = 3, fill = "green2", tags='arrow')

    blhdg = math.radians(450 - bhdg)
    blwidth = math.radians(bwidth)
    if bwidth > 0:
        blhe = blhdg
        brhe = blhdg + blwidth
    else:
        blhe = blhdg - blwidth
        brhe = blhdg

    lblipx = bdist * math.cos(blhe) * 10
    lblipy = -bdist * math.sin(blhe) * 10
    rblipx = bdist * math.cos(brhe) * 10
    rblipy = -bdist * math.sin(brhe) * 10
    rose.create_line(lblipx+240, lblipy+240, rblipx+240, rblipy+240, \
        width=8,fill="yellow", tags="blip")


# convert array of USfeet pairs into linear array of pixels (in pairs)
# slat, slon starting lat/lon usf from 34-14, -119-04
def Usf2Pix(usf, gscale, slat, slon):
    pix = []
    for x in usf:
#         pix.insert(0, int(gscale * (slon-x[0])))   # North:x coord, East:y coord
        pix.append(int(gscale * (slon+x[0])))
        pix.append(int(gscale * (slat-x[1])))
    return pix

# having an event handler for button down as well as button release may look redundant
# but touch and mouse event sequences are not identical, to wit: touch gives movement followed
# by button down, mouse gived button then movement

def Bpress(event):
    global mx
    global my
    mx = event.x
    my = event.y
    
def Bupress(event):
    global mx
    global my
    global mode
    global goto
    global gotolatlon
    mx = 0
    my = 0
    if (mode == 3):
        lat = stlat - event.y/scale # North: y coord East:x coord
        lon = event.x/scale - stlon
        gotolatlon = [lon, lat]
#        print (str(lat)+"/"+str(lon))
        pnt = Usf2Pix([gotolatlon], scale, stlat, stlon)
        goto = canvas.create_rectangle(pnt[0], pnt[1], pnt[0]+4, pnt[1]+4, \
            fill='blue',outline='blue', tags = "wpts")
        msg = '{GN%7.2f}' % lon
        ser.write(msg.encode('utf-8'))
        print(msg)
        msg = '{GT%7.2f}' % lat
        ser.write(msg.encode('utf-8'))
        print(msg)
        mode = 0
    
def Mouse(event):
    global mx
    global my
#    global goto
    global stlat
    global stlon
    if (mode == 3):
        return

    if (mx == 0 and my == 0):
        mx = event.x
        my = event.y
        return
    x = event.x - mx
    y = event.y - my
    stlat += y/scale
    stlon += x/scale
    
#    print("move")
#     print(str(event.x)+" "+str(event.y))
#     print(str(int(stlat))+" "+str(int(stlon)))
    canvas.move(plot, x, y)
    canvas.move(bdrv, x, y)
    canvas.move(fdrv, x, y)
    canvas.move(arena, x, y)
    canvas.move(lunge, x, y)
#    canvas.move(goto, x, y)
    canvas.move('bldg', x, y)
    canvas.move('forest', x, y)
    canvas.move('path', x, y)
    canvas.move('wpts', x, y)
    mx = event.x
    my = event.y

def Zoomer(inout):
    global stlat
    global stlon
    global scale
    global rez
    global fdrv
    global bdrv
    global plot
#    global goto
    global arena
    global lunge
    global mode

    if inout == 1:
        stlat = stlat - (300/scale)/3
        stlon = stlon - (300/scale)/3
        scale *= 1.5
    if inout == 2:
        stlat = stlat + (300/scale)/2
        stlon = stlon + (300/scale)/2
        scale *= 2/3
        
#    print("zoom")
    canvas.delete(arena)
    canvas.delete(plot)
    canvas.delete(lunge)
    canvas.delete(fdrv)
    canvas.delete(bdrv)
#    canvas.delete(goto)
    canvas.delete('bldg')
    canvas.delete('forest')
    canvas.delete('path')
    canvas.delete('wpts')
    Chart(root)
    pnt = Usf2Pix([gotolatlon], scale, stlat, stlon)
    goto = canvas.create_rectangle(pnt[0], pnt[1], pnt[0]+4, pnt[1]+4, \
        fill='blue',outline='blue', tags = "wpts")
    mode = 0

class App:
    
    def __init__(self, master):
        self.mode = IntVar()
        self.ibuffer = ""
        self.piflag = False

        # telemetry array ===========================================
        data = Frame(master)
        data.place(x=20,y=360)
        sta=Label(data,text="STS:", font=(None,15))
        sta.grid(row=0,column=0)
        spd=Label(data,text="SPD:", font=(None,15))
        spd.grid(row=1,column=0)
        hdg=Label(data,text="HDG:", font=(None,15))
        hdg.grid(row=2,column=0)
        ste=Label(data,text="STR:", font=(None,15))
        ste.grid(row=3,column=0)
        dtgl=Label(data,text="RNG:", font=(None,15))
        dtgl.grid(row=4,column=0)
        ctgl=Label(data,text="BRG:", font=(None,15))
        ctgl.grid(row=5,column=0)
        xtel=Label(data,text="XTE:", font=(None,15))
        xtel.grid(row=6,column=0)
        
#         self.accr=Label(data,text="GPS:", font=(None,20))
#         self.accr.grid(row=7,column=0)
#         bat=Label(data,text="BAT:", font=(None,20))
#         bat.grid(row=8,column=0)
# 
        self.status = StringVar()
        Label(data,text="MM",width=5,font=(None,20),bg="white",fg="blue", \
              borderwidth=1,relief="solid",\
              textvariable=self.status).grid(row=0,column=1)
        self.speed = StringVar()
        Label(data,width=5,font=(None,20),bg="white",fg="blue", \
              borderwidth=1,relief="solid",\
              textvariable=self.speed).grid(row=1,column=1)
        self.head = StringVar()
        Label(data,width=5,font=(None,20),bg="white",fg="blue", \
              borderwidth=1,relief="solid",\
              textvariable=self.head).grid(row=2,column=1)
        self.steer = StringVar()
        Label(data,width=5,font=(None,20),bg="white",fg="blue", \
              borderwidth=1,relief="solid",\
              textvariable=self.steer).grid(row=3,column=1)
        self.dtg = StringVar()
        Label(data,width=5,font=(None,20),bg="white",fg="blue", \
              borderwidth=1,relief="solid",\
              textvariable=self.dtg).grid(row=4,column=1)
        self.ctg = StringVar()
        Label(data,width=5,font=(None,20),bg="white",fg="blue", \
              borderwidth=1,relief="solid",\
              textvariable=self.ctg).grid(row=5,column=1)
        self.xte = StringVar()
        Label(data,width=5,font=(None,20),bg="white",fg="blue", \
              borderwidth=1,relief="solid",\
              textvariable=self.xte).grid(row=6,column=1)
        
        alert = Frame(master)
        alert.place(x=20,y=620)
        self.accr=Label(alert,text="GPS:", font=(None,15))
        self.accr.grid(row=0,column=0)
        self.bat=Label(alert,text="BAT:", font=(None,15))
        self.bat.grid(row=1,column=0)

        '''
        self.acc = StringVar()
        self.acclab = Label(data,width=5,font=(None,20),bg="white",fg="blue", \
              borderwidth=1,relief="solid",\
              textvariable=self.acc).grid(row=7,column=1)
        self.bat = StringVar()
        Label(data,width=5,font=(None,20),bg="white",fg="blue", \
              borderwidth=1,relief="solid",\
              textvariable=self.bat).grid(row=8,column=1)
         '''

        # mode menu ===========================================================
        radio = Frame(master)
        radio.place(x=20, y=20)
        rb1 = Radiobutton(radio, text="Standby", variable=self.mode, value = 0, \
            anchor=W, command=lambda:self.mode_set(master, self.mode.get()))
        rb1.config(width = 7, height = 2, font=(NONE,15))
        rb1.grid(row=0, column=0)
        
        rb2 = Radiobutton(radio, text="Auto", variable=self.mode, value = 1, \
            anchor=W, command=lambda:self.mode_set(master, self.mode.get()))
        rb2.config(width = 6, height = 2, font=(NONE,15))
        rb2.grid(row=1, column=0)
        
        rb3 = Radiobutton(radio, text="Path", variable=self.mode, value = 2, \
            anchor=W, command=lambda:self.mode_set(master, self.mode.get()))
        rb3.config(width = 6, height = 2, font=(NONE,15))
        rb3.grid(row=2, column=0)
        
        rb4 = Radiobutton(radio, text="PanTilt", variable=self.mode, value = 3, \
            anchor=W, command=lambda:self.mode_set(master, self.mode.get()))
        rb4.config(width = 6, height = 2, font=(NONE,15))
        rb4.grid(row=3, column=0)
        
        rb5 = Radiobutton(radio, text="Misc", variable=self.mode, value = 4, \
            anchor=W, command=lambda:self.mode_set(master, self.mode.get()))
        rb5.config(width = 6, height = 2, font=(NONE,15))
        rb5.grid(row=4, column=0)
        
# enlarge button array ===================================================
        zoom = Frame(root)
        zoom.place(x=825, y=20)
        fmax = Button(zoom, text = "+", command = lambda:Zoomer(1))
        fmax.config(width = 1, height = 1, font=(NONE,15), \
            bg="green2",fg="black",borderwidth=4)
        fmax.grid(row=0,column=0)

        rmax = Button(zoom, text = "-", command = lambda:Zoomer(2))
        rmax.config(width = 1, height = 1, font=(NONE,15), \
            bg="pink",fg="black",borderwidth=4)
        rmax.grid(row=1,column=0)
                
        rmax = Button(zoom, text = "X", command = lambda:scaler(3))
        rmax.config(width = 1, height = 1, font=(NONE,15), \
            bg="deep sky blue",fg="black",borderwidth=4)
        rmax.grid(row=2,column=0)
        
        def scaler(scl):
            global mode
            mode = scl
#
# compass rose ============================================================
        rose.create_oval(30, 30, 450, 450, width=1, outline='black', fill="black")
        rose.create_oval(85, 85, 395, 395, width=1, outline='white', fill="black")
        rose.create_oval(140, 140, 340, 340, width=1, outline='white', fill="black")
        rose.create_oval(195, 195, 285, 285, width=1, outline='white', fill="black")
        rose.create_line(91, 91, 388, 388, width = 1, fill = 'white')
        rose.create_line(91, 388, 388, 91, width = 1, fill = 'white')
        
        rose.create_line(30, 240, 450, 240, width = 1, fill = 'white')
        rose.create_line(240, 30, 240, 450, width = 1, fill = 'white')
        rose.create_text(240, 8, text="N", font = ffont, angle=0)
        rose.create_text(240, 475, text="S", font = ffont, angle=0)
        rose.create_text(12, 240, text="W", font = ffont, angle=90)
        rose.create_text(465, 240, text="E", font = ffont, angle=270)
        
        rose.create_text(400, 85, text = "NE", font = efont, angle=315)
        rose.create_text(405, 395, text = "SE", font = efont, angle=225)
        rose.create_text(75, 400, text = "SW", font = efont, angle=135)
        rose.create_text(80, 85, text = "NW", font = efont, angle=45)
        
    # destroy old frames when changing mode via radiobuttons ====================
    def mode_set(self, mstr, val):

        try:
            lister.destroy()
        except:
            pass
        try:
            auto.destroy()
        except:
            pass
        try:
            pntlt.destroy()
        except:
            pass
        try:
            miscer.destroy()
        except:
            pass
        
        if (val == 0):
#            Chart(mstr)
            Zoomer(0)
            Guage(mstr)
            
        if (val == 1):
            self.AutoTurns(mstr)
            
        if (val == 2):
            self.Paths(mstr)
            
        if (val == 3):
            self.PanTilt(mstr)
            
        if (val == 4):
            self.Misc(mstr)
           
    # frame for wapoint/route selection =====================================================    
    def Paths(self, mstr):
        global lister
        lister = Frame(mstr)
        lister.place(x=200, y=450)
        lab = Label(lister, text="Select NAV path")
        lab.grid(row=0, column=0)
        lscroll = Scrollbar(lister, orient=VERTICAL, width=25, bg = "black")
        lbox =Listbox(lister, height=6, selectmode=SINGLE,font=(NONE,15), \
            yscrollcommand=lscroll.set)
        for rt in routes:
            lbox.insert(END, rt[0])
        for wp in wayptnames:
            lbox.insert(END, wp)

        lbox.grid(row=1, column=0)
        lscroll.config(width=25, command=lbox.yview)
        lscroll.grid(row=1, column=1, sticky=N+S)
        ex = Button(lister, text="Execute", command=lambda:self.lrevert(lbox.get(ANCHOR)))
        ex.config(width=5, height=3, font=(None,15), bg="green2")
        ex.grid(row = 2, column = 0)
        quit = Button(lister, text="Cancel", command=lambda:self.lrevert('000'))
        quit.config(width=5, height=3, font=(None,15), bg="red",fg="black")
        quit.grid(row=4, column=0)

    #could call fxmit directly if no radiobutton action wanted
    def lrevert(self, pth):
        self.fxmit(pth[1:3])
        lister.destroy()
        self.mode.set(0)

    # AUTO mode button array ==================================================
    def AutoTurns(self, mstr):
        global auto
        auto = Frame(mstr)
        auto.place(x=250, y=470)
        bs=Button(auto, text="Start", command = lambda:self.exmit('2'))
        bs.config(width=4,height=2,font=(None,15),bg="white",fg="black")
        bs.grid(row=0,column=0,columnspan=2)
        
        bl90=Button(auto, text="< 90", command = lambda:self.exmit('1'))
        bl90.config(width=3,height=2,font=(None,15),bg="pink",fg="black")
        bl90.grid(row=1,column=0)
        
        br90=Button(auto, text="90 >", command = lambda:self.exmit('3'))
        br90.config(width=3,height=2,font=(None,15),bg="green2",fg="black")
        br90.grid(row=1,column=1)
        
        blt=Button(auto, text="T 90", command = lambda:self.exmit('4'))
        blt.config(width=3,height=2,font=(None,15),bg="pink",fg="black")
        blt.grid(row=2,column=0)
        
        brt=Button(auto, text="90 T", command = lambda:self.exmit('6'))
        brt.config(width=3,height=2,font=(None,15),bg="green2",fg="black")
        brt.grid(row=2,column=1)
        
        bl180=Button(auto, text="< 180", command = lambda:self.exmit('7'))
        bl180.config(width=3,height=2,font=(None,15),bg="pink",fg="black")
        bl180.grid(row=3,column=0)
        
        br180=Button(auto, text="180 >", command = lambda:self.exmit('9'))
        br180.config(width=3,height=2,font=(None,15),bg="green2",fg="black")
        br180.grid(row=3,column=1)
        
        bcan=Button(auto, text="Cancel", command=self.arevert)
        bcan.config(width=4,height=2,font=(None,15),bg="yellow",fg="black")
        bcan.grid(row=4,column=0,columnspan=2)

    # cancel AUTO mode
    def arevert(self):
        auto.destroy()
        self.mode.set(0)
        self.exmit('0')
           
    # AUTO mode button array ==================================================
    def PanTilt(self, mstr):
        global pntlt
        pntlt = Frame(mstr)
        pntlt.place(x=200, y=490)
        bup=Button(pntlt, text="tilt Up", command = lambda:self.dxmit('U'))
        bup.config(width=5,height=2,font=(None,15),bg="cyan",fg="black")
        bup.grid(row=0,column=0,columnspan=3)
        
        bleft=Button(pntlt, text="pan left", command = lambda:self.dxmit('L'))
        bleft.config(width=5,height=2,font=(None,15),bg="pink",fg="black")
        bleft.grid(row=1,column=0)
        
        bctr=Button(pntlt, text="center", command = lambda:self.dxmit('C'))
        bctr.config(width=5,height=2,font=(None,15),bg="white",fg="black")
        bctr.grid(row=1,column=1)
        
        brght=Button(pntlt, text="pan right", command = lambda:self.dxmit('R'))
        brght.config(width=5,height=2,font=(None,15),bg="green2",fg="black")
        brght.grid(row=1,column=2)
        
        bdwn=Button(pntlt, text="tilt down", command = lambda:self.dxmit('D'))
        bdwn.config(width=5,height=2,font=(None,15),bg="sandy brown",fg="black")
        bdwn.grid(row=2,column=0,columnspan=3)
        
        bcan=Button(pntlt, text="Cancel", command=self.ptquit)
        bcan.config(width=5,height=2,font=(None,15),bg="yellow",fg="black")
        bcan.grid(row=4,column=0,columnspan=3)

    def ptquit(self):
        pntlt.destroy()
        self.mode.set(0)
        
    # misc commands ==========================================================
    def Misc(self, mstr):
        global miscer
        miscer = Frame(mstr)
        self.bat.config(text = "BAT: 13.9", fg = "red", bg = "white")
        miscer.place(x=200, y=450)
        msb1=Button(miscer, text="Diag", command=lambda:self.txmit('0'))
        msb1.config(width=4,height=2,font=(None,15),bg="white",fg="black")
        msb1.grid(row=0,column=0)
        
        msb2=Button(miscer, text="Mark", command=lambda:self.txmit('2'))
        msb2.config(width=4,height=2,font=(None,15),bg="white",fg="black")
        msb2.grid(row=2,column=0)
        
        msb3=Button(miscer, text="Pic", command=lambda:self.txmit('3'))
        msb3.config(width=4,height=2,font=(None,15),bg="white",fg="black")
        msb3.grid(row=3,column=0)

        msbs=Button(miscer)
        msbs.config(width=4,height=2,font=(None,15),bg="grey85",fg="grey85")
        msbs.grid(row=4,column=0)

        msb4=Button(miscer, text="Stop", command=lambda:self.txmit('1'))
        msb4.config(width=6,height=4,font=(None,15),bg="red",fg="black")
        msb4.grid(row=6,column=0)

    def dxmit(self, key):
        self.msg = '{D' + key + '}'
        ser.write(self.msg.encode('utf-8'))
        print(self.msg)

    def exmit(self, key):
        self.msg = '{E' + key + '}'
        ser.write(self.msg.encode('utf-8'))
        print(self.msg)

    def fxmit(self, key):
        self.msg = '{F' + key + '}'
        ser.write(self.msg.encode('utf-8'))
        print(self.msg)

    def txmit(self, key):
        self.msg = '{T' + key + '}'
        ser.write(self.msg.encode('utf-8'))
        print(self.msg)

#   Listen to serial port for status info from rover pi ================================
    def Listen(self):
        global butngreen
        global butngreen2
        global butnred
        global butnred2
        global butnblack
        global butnblue
        global butnyellow
        global blackepoch
        global greenepoch
        global redepoch
        global track
        global lat
        global lon
        global rhdg
        global strhdg
        global bdist
        global bhdg
        global bwidth
        
        while ser.in_waiting:
            try:
                inpt = ser.read(1).decode("utf-8")
            except UnicodeDecodeError:
                continue

            if (inpt == '{'):
                self.ibuffer = ""
                continue
            if (inpt == '}'):
                self.piflag = True
                break;
            self.ibuffer = self.ibuffer + inpt
           
        if self.piflag:
            if (len(self.ibuffer) >= 3):
                
                xchar = self.ibuffer[0]
                lbuffer = self.ibuffer[1:]
#                print (self.ibuffer)
                
                if (xchar == 'a'):               # status
                    self.status.set(lbuffer)
                        
                elif (xchar == 'b'):             # battery
                    batry = float(lbuffer)
                    if (batry > 14.0):
                        self.bat.config(text = "BAT:" + lbuffer, fg = "blue", bg = "white")
                    else:
                        self.bat.config(text = "BAT:" + lbuffer, fg = "red", bg = "white")
                    self.bat.set(lbuffer)
                        
                elif (xchar == 'c'):             # course to wpt
                    self.ctg.set(lbuffer)
                        
                elif (xchar == 'd'):             # distance to wpt
                    self.dtg.set(lbuffer)
                        
                elif (xchar == 'h'):
                    self.head.set(lbuffer)
                    rhdg = math.radians(450 - float(lbuffer))
                    Guage(root)
                        
                elif (xchar == 'l'):
                    xchar = lbuffer[0]
                    lbuffer = lbuffer[1:]
                    if (xchar == 'a'):          # GPS accuracy
                        self.acc.set(lbuffer)
                        accry = float(lbuffer)
                        if (accry < 1.0):
                            self.accr.config(text = "GPS:" + lbuffer, fg = "blue", bg = "white")
                        else:
                            self.accr.config(text = "GPS:" + lbuffer, fg = "red", bg = "white")
                            
                    if (xchar == 'x'):          # x-track error
                        self.xte.set(lbuffer)
                    if (xchar == 't'):
                        lat = float(lbuffer)
                    if (xchar == 'n'):
                        try:
                            lon = float(lbuffer)
                        except ValueError:
                            pass
                        # TBD dont append if same lat/lon
                        # track.append([lon, lat])
                        Xspot(root, lon, lat)

                elif xchar == 'r':
                    bdist, bhdg, bwidth = lbuffer.split(',')
                    print(bdist, bhdg, bwidth)
                    bdist = int(bdist)
                    bhdg = int(bhdg)
                    bwidth = int(bwidth)
                    Guage(root)
                    
                elif (xchar == 's'):            # steering angle
                    self.steer.set(lbuffer)
                    strhdg = int(lbuffer)
                    Guage(root)
                        
                elif (xchar == 'v'):            # speed
                    self.speed.set(lbuffer)
                    
            self.piflag = False
            self.ibuffer = ""

        # check tactile buttons
        if (GPIO.input(21) == False):              # button grounds out GPIO
            if not butngreen:                      # if not stale tap
                if ((time.time() - greenepoch) < .6): # if less than .6 sec
                    if butngreen2:                 # if double tap in progress
                        self.dxmit('9')            # 35 deg turn
                    else:
                        self.dxmit('6')            # 5 deg steering
                        butngreen2 = True          # double tap started
                else:                              # else 1st tap
                    self.dxmit('3')                # 1 deg steering
                    butngreen2 = False             # first tap
                butngreen = True                   # register the pressed state
                greenepoch = time.time()           # start timer
        else:
            butngreen = False                      # button released
            
        if (GPIO.input(5) == False):
            if not butnblack:
                if ((time.time() - blackepoch) < .6): # if less than .6 sec
                    self.dxmit('0')                # all stop!
                else:
                    self.dxmit('5')                # zero steering
                butnblack = True
                blackepoch = time.time()           # reset
        else:
            butnblack = False
            
        if (GPIO.input(13) == False):
            if not butnred:
                if ((time.time() - redepoch) < .6): # if less than .6 sec
                    if butnred2:
                        self.dxmit('7')             # 35 deg steering
                    else:
                        self.dxmit('4')             # 5 deg steering
                        butnred2 = True
                else:
                    self.dxmit('1')                 # 1 deg steering
                    butnred2 = False
                butnred = True
                redepoch = time.time()
        else:
            butnred = False
        if (GPIO.input(7) == False):
            if not butnblue:
                self.dxmit('2')
                butnblue = True
        else:
            butnblue = False
            
        if (GPIO.input(10) == False):
            if not butnyellow:
                self.dxmit('8')
                butnyellow = True
        else:
            butnyellow = False
            
        if pathflag:
            try:
                cdfline = pathfile.readline()
                if (cdfline != ''):
                    line = cdfline.split(',')
                    Xspot(root, float(line[2]), float(line[3]))
                    # track.append([float(line[2]), float(line[3])])
                    rhdg = math.radians(450 - float(line[8]))
                    strhdg = int(line[7])
#                    Chart(root)
                    Guage(root)

            except IOError:
                pass

        root.after(25, self.Listen)

#======================================================================
# Initialize from database

rc = cdlib.Cdblogin()
treetable = cdlib.COpenTable("TreeTable".encode())
locndx = cdlib.COpenIndex(treetable, "TreeNdx".encode())

rc = cdlib.CFirst(treetable, locndx)

alltrees = []
while (rc >= 0):
    lon = cdlib.CGetDouble(treetable, "Lonft".encode())
    lat = cdlib.CGetDouble(treetable, "Latft".encode())
    alltrees.append([lon, lat])
    rc = cdlib.CNext(treetable, locndx)
 
rtetable = cdlib.COpenTable("Routes".encode())
rc = cdlib.CFirst(rtetable, 0)           # using primary index
routes = []
while (cdlib.CNext(rtetable, 0) >= 0):
    pstr = cdlib.CGetCharPtr(rtetable, "Name".encode())
    routes.append([pstr.decode()])
    
waytable = cdlib.COpenTable("WayPoint".encode())
wayptnames = []
waypts = []
rc = cdlib.CFirst(waytable, 0)           # using primary index
while (rc >= 0):
    lon = cdlib.CGetDouble(waytable, "Efeet".encode())
    lat = cdlib.CGetDouble(waytable, "Nfeet".encode())
    pstr = cdlib.CGetCharPtr(waytable, "Name".encode())
#    print("lon, lat, name",lon, lat, pstr.decode())
    wayptnames.append(pstr.decode())
    waypts.append([lon, lat])
    rc = cdlib.CNext(waytable, 0)
    
#print(waypts)   
#print("trees = ", alltrees)
root = Tk()
ffont = Font(family="URW Chancery L", size=20, weight = "bold")
efont = Font(family="URW Chancery L", size=16)
nfont = Font(family="Century Schoolbook L", size=14)
root.wm_title('Rover Controller 211210')
chartform = Frame(root)
chartform.place(x=200, y=20)
canvas= Canvas(chartform, width=600, height=600, bg='white')
canvas.pack()
Chart(root)

rosefrm = Frame(root)
rosefrm.place(x = 800, y = 100)
rose = Canvas(rosefrm, width=500, height=500, bg='gray85')
rose.pack()
Guage(root)

app = App(root)
root.geometry("1280x800+0+0")

root.after(25, app.Listen)
canvas.bind('<B1-Motion>', Mouse)
#canvas.bind('<Double-Button-1>', taptap)
canvas.bind('<Button-1>', Bpress)
canvas.bind('<ButtonRelease>', Bupress)

root.mainloop()

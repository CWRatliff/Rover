#07_04_temp_final.py

from tkinter import *
import serial

ser = serial.Serial(port='/dev/ttyS0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
    )

       
class App:
    
    def __init__(self, master):
        self.mode = IntVar()

        # telemetry array ===========================================
        data = Frame(master)
        sta=Label(data,text="STS:", font=(None,15))
        sta.grid(row=0,column=0)
        spd=Label(data,text="SPD:", font=(None,15))
        spd.grid(row=1,column=0)
        hdg=Label(data,text="HDG:", font=(None,15))
        hdg.grid(row=2,column=0)
        ste=Label(data,text="STR:", font=(None,15))
        ste.grid(row=3,column=0)
        dtg=Label(data,text="DTG:", font=(None,15))
        dtg.grid(row=4,column=0)
        ctg=Label(data,text="CTG:", font=(None,15))
        ctg.grid(row=5,column=0)
        xte=Label(data,text="XTE:", font=(None,15))
        xte.grid(row=6,column=0)
        lat=Label(data,text="ACC:", font=(None,15))
        lat.grid(row=7,column=0)

        self.status = StringVar()
        Label(data,width=7,font=(None,20),bg="white",fg="blue",borderwidth=1,relief="solid",\
              textvariable=self.status).grid(row=0,column=1)
        self.speed = StringVar()
        Label(data,width=7,font=(None,20),bg="white",fg="blue",borderwidth=1,relief="solid",\
              textvariable=self.speed).grid(row=1,column=1)
        self.head = StringVar()
        Label(data,width=7,font=(None,20),bg="white",fg="blue",borderwidth=1,relief="solid",\
              textvariable=self.head).grid(row=2,column=1)
        self.steer = StringVar()
        Label(data,width=7,font=(None,20),bg="white",fg="blue",borderwidth=1,relief="solid",\
              textvariable=self.steer).grid(row=3,column=1)
        self.dtg = StringVar()
        Label(data,width=7,font=(None,20),bg="white",fg="blue",borderwidth=1,relief="solid",\
              textvariable=self.dtg).grid(row=4,column=1)
        self.ctg = StringVar()
        Label(data,width=7,font=(None,20),bg="white",fg="blue",borderwidth=1,relief="solid",\
              textvariable=self.ctg).grid(row=5,column=1)
        self.xte = StringVar()
        Label(data,width=7,font=(None,20),bg="white",fg="blue",borderwidth=1,relief="solid",\
              textvariable=self.xte).grid(row=6,column=1)
        self.acc = StringVar()
        Label(data,width=7,font=(None,20),bg="white",fg="blue",borderwidth=1,relief="solid",\
              textvariable=self.acc).grid(row=7,column=1)
        

        # STOP button ==================================================
        estop=Frame(master)
        sb = Button(estop, text="STOP", command=self.zero)
        sb.config(width=3,height=2,font=(None,25),bg="red",fg="white",borderwidth=4)
        sb.grid(row=0,column=0)

        # Steering button array ========================================
        steer = Frame(master)
        l35 = Button(steer, text="<<<")
        l35.config(width=3,height=1,font=(None,15),bg="pink",fg="black",borderwidth=4)
        l35.grid(row=0,column=0)
        
        l5 = Button(steer, text="<<")
        l5.config(width=3,height=1,font=(None,15),bg="pink",fg="black",borderwidth=4)
        l5.grid(row=0,column=1)
        
        l1 = Button(steer, text="<")
        l1.config(width=3,height=1,font=(None,15),bg="pink",fg="black",borderwidth=4)
        l1.grid(row=0,column=2)
        
        z0 = Button(steer, text=".")
        z0.config(width=3,height=1,font=(None,25),bg="linen",fg="black",borderwidth=4)
        z0.grid(row=0,column=3)
        
        r1 = Button(steer, text=">")
        r1.config(width=3,height=1,font=(None,15),bg="green2",fg="black",borderwidth=4)
        r1.grid(row=0,column=4)
        
        r5 = Button(steer, text=">>")
        r5.config(width=3,height=1,font=(None,15),bg="green2",fg="black",borderwidth=4)
        r5.grid(row=0,column=5)

        r35 = Button(steer, text=">>>")
        r35.config(width=3,height=1,font=(None,15),bg="green2",fg="black",borderwidth=4)
        r35.grid(row=0,column=6)

        # speed button array ===================================================
        speed = Frame(master)
        fmax = Button(speed, text = "+")
        fmax.config(width = 2, height = 2, font=(NONE,15), bg="green2",fg="black",borderwidth=4)
        fmax.grid(row=0,column=0)
        f0 = Button(speed, text = "0")
        f0.config(width = 2, height = 2, font=(NONE,15), bg="linen",fg="black",borderwidth=4)
        f0.grid(row=3,column=0)
        rmax = Button(speed, text = "-")
        rmax.config(width = 2, height = 2, font=(NONE,15), bg="pink",fg="black",borderwidth=4)
        rmax.grid(row=6,column=0)
        

        # mode menu ===========================================================
        radio = Frame(master)
        rb1 = Radiobutton(radio, text="Standby", variable=self.mode, value = 0, \
            anchor=W, command=lambda:self.mode_set(master, self.mode.get()))
        rb1.config(width = 6, height = 2, font=(NONE,15))
        rb1.grid(row=0, column=0)
        rb2 = Radiobutton(radio, text="Auto", variable=self.mode, value = 1, \
            anchor=W, command=lambda:self.mode_set(master, self.mode.get()))
        rb2.config(width = 6, height = 2, font=(NONE,15))
        rb2.grid(row=1, column=0)
        rb3 = Radiobutton(radio, text="Path", variable=self.mode, value = 2, \
            anchor=W, command=lambda:self.mode_set(master, self.mode.get()))
        rb3.config(width = 6, height = 2, font=(NONE,15))
        rb3.grid(row=2, column=0)
        
        steer.place(x=400,y=520)
        speed.place(x=950, y=310)
        data.place(x=20,y=20)
        radio.place(x=320, y=20)
        estop.place(x=20, y=480)
        
        self.ibuffer = ""
        self.msg = ""
        self.exeflag = False
        self.lbflag = False
        self.lb2flag = False
        self.piflag = False

    def mode_set(self, mstr, val):
        if (val == 0):
            try:
                lister.destroy()
            except:
                print("val = 0, couldnt destroy lister")
                pass
            try:
                auto.destroy()
            except:
                pass
        if (val == 1):
            try:
                lister.destroy()
            except:
                pass
            self.auto_turns(mstr)
        if (val == 2):
            try:
                auto.destroy()
            except:
                pass
            self.paths(mstr)
            
    
    def paths(self, mstr):
        global lister
        lister = Frame(mstr)
        lister.place(x=650, y=20)
        lab = Label(lister, text="Select NAV path")
        lab.grid(row=0, column=0)
        lscroll = Scrollbar(lister, orient=VERTICAL)
        lbox =Listbox(lister, height=4, selectmode=SINGLE,font=(NONE,15),yscrollcommand=lscroll.set)
        lbox.insert(END, "R3 - E.F. drive")
        lbox.insert(END, "R4 - hut row")
        lbox.insert(END, "W13 - canopy")
        lbox.insert(END, "W14 - driveway center")
        lbox.insert(END, "W23 - trash cans")
        lbox.insert(END, "W27 - rose bush")
        lbox.insert(END, "W29 - E,F, middle")
        lbox.insert(END, "W30 - office gap")
        lbox.insert(END, "W23 - rose passage")

        lbox.grid(row=1, column=0)
        lscroll.config(width=25, command=lbox.yview)
        lscroll.grid(row=1, column=1, sticky=N+S)
        ex = Button(lister, text="Execute", command=self.lrevert)
        ex.grid(row = 2, column = 0)
        
    def lrevert(self):
        lister.destroy()
        self.mode.set(0)
        
    def auto_turns(self, mstr):
        # auto button array =================================================
        global auto
        auto = Frame(mstr)
        auto.place(x=600, y=20)
        bs=Button(auto, text="Start")
        bs.config(width=4,height=2,font=(None,15),bg="white",fg="black")
        bs.grid(row=0,column=0,columnspan=2)
        bl90=Button(auto, text="< 90")
        bl90.config(width=3,height=2,font=(None,15),bg="pink",fg="black")
        bl90.grid(row=1,column=0)
        br90=Button(auto, text="90 >")
        br90.config(width=3,height=2,font=(None,15),bg="green2",fg="black")
        br90.grid(row=1,column=1)
        blt=Button(auto, text="T 90")
        blt.config(width=3,height=2,font=(None,15),bg="pink",fg="black")
        blt.grid(row=2,column=0)
        brt=Button(auto, text="90 T")
        brt.config(width=3,height=2,font=(None,15),bg="green2",fg="black")
        brt.grid(row=2,column=1)
        bl180=Button(auto, text="< 180")
        bl180.config(width=3,height=2,font=(None,15),bg="pink",fg="black")
        bl180.grid(row=3,column=0)
        br180=Button(auto, text="180 >")
        br180.config(width=3,height=2,font=(None,15),bg="green2",fg="black")
        br180.grid(row=3,column=1)
        bcan=Button(auto, text="Cancel", command=self.arevert)
        bcan.config(width=4,height=2,font=(None,15),bg="yellow",fg="black")
        bcan.grid(row=4,column=0,columnspan=2)

    def arevert(self):
        auto.destroy()
        self.mode.set(0)
        
# keypad button actions
    def star(self):
        self.exeflag = True
        self.status.set("Auto")
        self.steer.set("-16")
        self.xte.set("3.45")
        self.speed.set("-100")

    def pound(self):
        self.lbflag = True
        self.lb2flag = False
        self.speed.set("10")
    def zero(self):
        self.key = '0'
        self.xmit()
        print("STOP")
    def one(self):
        self.key = '1'
        self.xmit()
    def two(self):
        self.key = '2'
        self.xmit()
    def three(self):
        self.key = '3'
        self.xmit()
    def four(self):
        self.key = '4'
        self.xmit()
    def five(self):
        self.key = '5'
        self.xmit()
    def six(self):
        self.key = '6'
        self.xmit()
    def seven(self):
        self.key = '7'
        self.xmit()
    def eight(self):
        self.key = '8'
        self.xmit()
    def nine(self):
        self.key = '9'
        self.xmit()
        
    def xmit(self):
        if (self.exeflag):
            self.msg = '{E' + self.key + '}'
            ser.write(self.msg.encode('utf-8'))
            self.msg = ""
            self.exeflag = False
        elif (self.lbflag):
            if (self.lb2flag):
                msg = self.msg + self.key + '}'
                ser.write(msg.encode('utf-8'))
                self.msg = ""
                self.lbflag = False
                self.lb2flag = False
            else:
                self.msg = '{F' + self.key
                self.lb2flag = True
        else:
            self.msg = '{D' + self.key + '}'
            ser.write(self.msg.encode('utf-8'))
            self.msg = ""

#   Listen to serial port for status info from rover pi
    def listen(self):
        while ser.in_waiting:
            inpt = ser.read(1).decode("utf-8")
            if (inpt == '{'):
                self.ihead = 0
                continue
            if (inpt == '}'):
                self.piflag = True
                break;
            self.ibuffer = self.ibuffer + inpt

           
        if self.piflag:
            if (len(self.ibuffer) >= 3):
                
                print(self.ibuffer)
                xchar = self.ibuffer[0]
                lbuffer = self.ibuffer[1:]
                
                if (xchar == 'a'):               # status
                    self.status.set(lbuffer)
                        
                elif (xchar == 'c'):             # course to wpt
                    self.ctg.set(lbuffer)
                        
                elif (xchar == 'd'):             # distance to wpt
                    self.dtg.set(lbuffer)
                        
                elif (xchar == 'h'):
                    self.head.set(lbuffer)
                        
                elif (xchar == 'l'):
                    xchar = lbuffer[0]
                    lbuffer = self.ibuffer[2:]
                    if (xchar == 'a'):          # GPS accuracy
                        self.acc.set(lbuffer)
                        
                elif (xchar == 's'):            # steering angle
                    self.steer.set(lbuffer)
                        
                elif (xchar == 'v'):            # speed
                    self.speed.set(lbuffer)
 
            self.piflag = False
 
        self.ibuffer = "" 
        root.after(25, self.listen)
 
        
root = Tk()
root.wm_title('Rover Controller')
app = App(root)
root.geometry("1024x600+0+0")
root.after(25, app.listen)


root.mainloop()


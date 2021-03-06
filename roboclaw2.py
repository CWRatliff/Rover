import serial
import struct

class Controller:
    def __init__(self, ttyStr):
        # Open the command self.port
        #Rasberry Pi/Linux Serial instance example
        self.checksum = 0
        self.port = serial.Serial(ttyStr, baudrate=115200, timeout=0.1)

    # Cleanup by closing USB serial self.port
    def close(self):
        self.port.close()

    def sendcommand(self,address,command):
            self.checksum = address
            self.port.write(bytes([address]));
            self.checksum += command
            self.port.write(bytes([command]));
            return;

    def readbyte(self):
            val = struct.unpack('>B',self.port.read(1));
            self.checksum += val[0]
            return val[0];
    def readsbyte(self):
            val = struct.unpack('>b',self.port.read(1));
            self.checksum += val[0]
            return val[0];	
    def readword(self):
            val = struct.unpack('>H',self.port.read(2));
            self.checksum += (val[0]&0xFF)
            self.checksum += (val[0]>>8)&0xFF
            return val[0];	
    def readsword(self):
            val = struct.unpack('>h',self.port.read(2));
            self.checksum += val[0]
            self.checksum += (val[0]>>8)&0xFF
            return val[0];	
    def readlong(self):
            val = struct.unpack('>L',self.port.read(4));
            self.checksum += val[0]
            self.checksum += (val[0]>>8)&0xFF
            self.checksum += (val[0]>>16)&0xFF
            self.checksum += (val[0]>>24)&0xFF
            return val[0];	
    def readslong(self):
            val = struct.unpack('>l',self.port.read(4));
            self.checksum += val[0]
            self.checksum += (val[0]>>8)&0xFF
            self.checksum += (val[0]>>16)&0xFF
            self.checksum += (val[0]>>24)&0xFF
            return val[0];	
    def writebyte(self, val):
            self.checksum += val
            return self.port.write(struct.pack('>B',val));
    def writesbyte(self, val):
            self.checksum += val
            return self.port.write(struct.pack('>b',val));
    
    def writeword(self,val):
            
            self.checksum += val
            self.checksum += (val>>8)&0xFF
            return self.port.write(struct.pack('>H',val));
    
    def writesword(self, val):
            
            self.checksum += val
            self.checksum += (val>>8)&0xFF
            return self.port.write(struct.pack('>h',val));
    
    def writelong(self, val):
            
            self.checksum += val
            self.checksum += (val>>8)&0xFF
            self.checksum += (val>>16)&0xFF
            self.checksum += (val>>24)&0xFF
            return self.port.write(struct.pack('>L',val));
    
    def writeslong(self, val):
            
            self.checksum += val
            self.checksum += (val>>8)&0xFF
            self.checksum += (val>>16)&0xFF
            self.checksum += (val>>24)&0xFF
            return self.port.write(struct.pack('>l',val));
    
    def M1Forward(self, val):
            self.sendcommand(128,0)
            self.writebyte(val)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def M1Backward(self, val):
            self.sendcommand(128,1)
            self.writebyte(val)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetMinMainBattery(self, val):
            self.sendcommand(128,2)
            self.writebyte(val)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetMaxMainBattery(self, val):
            self.sendcommand(128,3)
            self.writebyte(val)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def M2Forward(self, val):
            self.sendcommand(128,4)
            self.writebyte(val)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def M2Backward(self, val):
            self.sendcommand(128,5)
            self.writebyte(val)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def DriveM1(self, val):
            self.sendcommand(128,6)
            self.writebyte(val)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def DriveM2(self, val):
            self.sendcommand(128,7)
            self.writebyte(val)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def ForwardMixed(self, val):
            self.sendcommand(128,8)
            self.writebyte(val)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def BackwardMixed(self, val):
            self.sendcommand(128,9)
            self.writebyte(val)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def RightMixed(self, val):
            self.sendcommand(128,10)
            self.writebyte(val)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def LeftMixed(self, val):
            self.sendcommand(128,11)
            self.writebyte(val)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def DriveMixed(self, val):
            self.sendcommand(128,12)
            self.writebyte(val)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def TurnMixed(self, val):
            self.sendcommand(128,13)
            self.writebyte(val)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def readM1encoder(self):
            self.sendcommand(128,16);
            enc = self.readslong();
            status = self.readbyte();
            crc = self.checksum&0x7F
            if crc==self.readbyte()&0x7F:
                    return (enc,status);
            return (-1,-1);
    
    def readM2encoder(self):
            self.sendcommand(128,17);
            enc = self.readslong();
            status = self.readbyte();
            crc = self.checksum&0x7F
            if crc==self.readbyte()&0x7F:
                    return (enc,status);
            return (-1,-1);
    
    def readM1speed(self):
            self.sendcommand(128,18);
            enc = self.readslong();
            status = self.readbyte();
            crc = self.checksum&0x7F
            if crc==self.readbyte()&0x7F:
                    return (enc,status);
            return (-1,-1);
    
    def readM2speed(self):
            self.sendcommand(128,19);
            enc = self.readslong();
            status = self.readbyte();
            crc = self.checksum&0x7F
            if crc==self.readbyte()&0x7F:
                    return (enc,status);
            return (-1,-1);
    
    def ResetEncoderCnts(self):
            self.sendcommand(128,20)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def readversion(self):
            self.sendcommand(128,21)
            return self.port.read(32);
    
    def readmainbattery(self):
            self.sendcommand(128,24);
            val = self.readword()
            crc = self.checksum&0x7F
            if crc==self.readbyte()&0x7F:
                    return val
            return -1
    
    def readlogicbattery(self):
            self.sendcommand(128,25);
            val = self.readword()
            crc = self.checksum&0x7F
            if crc==self.readbyte()&0x7F:
                    return val
            return -1
    
    def SetM1pidq(self,p,i,d,qpps):
            self.sendcommand(128,28)
            self.writelong(d)
            self.writelong(p)
            self.writelong(i)
            self.writelong(qpps)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetM2pidq(self,p,i,d,qpps):
            self.sendcommand(128,29)
            self.writelong(d)
            self.writelong(p)
            self.writelong(i)
            self.writelong(qpps)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def readM1instspeed(self):
            self.sendcommand(128,30);
            enc = self.readslong();
            status = self.readbyte();
            crc = self.checksum&0x7F
            if crc==self.readbyte()&0x7F:
                    return (enc,status);
            return (-1,-1);
    
    def readM2instspeed(self):
            self.sendcommand(128,31);
            enc = self.readslong();
            status = self.readbyte();
            crc = self.checksum&0x7F
            if crc==self.readbyte()&0x7F:
                    return (enc,status);
            return (-1,-1);
    
    def SetM1Duty(self, val):
            self.sendcommand(128,32)
            self.writesword(val)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetM2Duty(self, val):
            self.sendcommand(128,33)
            self.writesword(val)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetMixedDuty(self,m1,m2):
            self.sendcommand(128,34)
            self.writesword(m1)
            self.writesword(m2)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetM1Speed(self, val):
            self.sendcommand(128,35)
            self.writeslong(val)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetM2Speed(self, val):
            self.sendcommand(128,36)
            self.writeslong(val)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetMixedSpeed(self,m1,m2):
            self.sendcommand(128,37)
            self.writeslong(m1)
            self.writeslong(m2)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetM1SpeedAccel(self,accel,speed):
            self.sendcommand(128,38)
            self.writelong(accel)
            self.writeslong(speed)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetM2SpeedAccel(self,accel,speed):
            self.sendcommand(128,39)
            self.writelong(accel)
            self.writeslong(speed)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetMixedSpeedAccel(self,accel,speed1,speed2):
            self.sendcommand(128,40)
            self.writelong(accel)
            self.writeslong(speed1)
            self.writeslong(speed2)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetM1SpeedDistance(self,speed,distance,buffer):
            self.sendcommand(128,41)
            self.writeslong(speed)
            self.writelong(distance)
            self.writebyte(buffer)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetM2SpeedDistance(self,speed,distance,buffer):
            self.sendcommand(128,42)
            self.writeslong(speed)
            self.writelong(distance)
            self.writebyte(buffer)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetMixedSpeedDistance(self,speed1,distance1,speed2,distance2,buffer):
            self.sendcommand(128,43)
            self.writeslong(speed1)
            self.writelong(distance1)
            self.writeslong(speed2)
            self.writelong(distance2)
            self.writebyte(buffer)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetM1SpeedAccelDistance(self,accel,speed,distance,buffer):
            self.sendcommand(128,44)
            self.writelong(accel)
            self.writeslong(speed)
            self.writelong(distance)
            self.writebyte(buffer)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetM2SpeedAccelDistance(self,accel,speed,distance,buffer):
            self.sendcommand(128,45)
            self.writelong(accel)
            self.writeslong(speed)
            self.writelong(distance)
            self.writebyte(buffer)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetMixedSpeedAccelDistance(self,accel,speed1,distance1,speed2,distance2,buffer):
            self.sendcommand(128,46)
            self.writelong(accel)
            self.writeslong(speed1)
            self.writelong(distance1)
            self.writeslong(speed2)
            self.writelong(distance2)
            self.writebyte(buffer)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def readbuffercnts(self):
            self.sendcommand(128,47);
            buffer1 = self.readbyte();
            buffer2 = self.readbyte();
            crc = self.checksum&0x7F
            if crc==self.readbyte()&0x7F:
                    return (buffer1,buffer2);
            return (-1,-1);
    
    def readcurrents(self):
            self.sendcommand(128,49);
            motor1 = self.readword();
            motor2 = self.readword();
            crc = self.checksum&0x7F
            if crc==self.readbyte()&0x7F:
                    return (motor1,motor2);
            return (-1,-1);
    
    def SetMixedSpeedIAccel(self,accel1,speed1,accel2,speed2):
            self.sendcommand(128,50)
            self.writelong(accel1)
            self.writeslong(speed1)
            self.writelong(accel2)
            self.writeslong(speed2)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetMixedSpeedIAccelDistance(self,accel1,speed1,distance1,accel2,speed2,distance2,buffer):
            self.sendcommand(128,51)
            self.writelong(accel1)
            self.writeslong(speed1)
            self.writelong(distance1)
            self.writelong(accel2)
            self.writeslong(speed2)
            self.writelong(distance2)
            self.writebyte(buffer)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetM1DutyAccel(self,accel,duty):
            self.sendcommand(128,52)
            self.writesword(duty)
            self.writeword(accel)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetM2DutyAccel(self,accel,duty):
            self.sendcommand(128,53)
            self.writesword(duty)
            self.writeword(accel)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetMixedDutyAccel(self,accel1,duty1,accel2,duty2):
            self.sendcommand(128,54)
            self.writesword(duty1)
            self.writeword(accel1)
            self.writesword(duty2)
            self.writeword(accel2)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def readM1pidq(self):
            self.sendcommand(128,55);
            p = self.readlong();
            i = self.readlong();
            d = self.readlong();
            qpps = self.readlong();
            crc = self.checksum&0x7F
            if crc==self.readbyte()&0x7F:
                    return (p,i,d,qpps);
            return (-1,-1,-1,-1)
    
    def readM2pidq(self):
            self.sendcommand(128,56);
            p = self.readlong();
            i = self.readlong();
            d = self.readlong();
            qpps = self.readlong();
            crc = self.checksum&0x7F
            if crc==self.readbyte()&0x7F:
                    return (p,i,d,qpps);
            return (-1,-1,-1,-1)
    
    def readmainbatterysettings(self):
            self.sendcommand(128,59);
            min = self.readword();
            max = self.readword();
            crc = self.checksum&0x7F
            if crc==self.readbyte()&0x7F:
                    return (min,max);
            return (-1,-1);
    
    def readlogicbatterysettings(self):
            self.sendcommand(128,60);
            min = self.readword();
            max = self.readword();
            crc = self.checksum&0x7F
            if crc==self.readbyte()&0x7F:
                    return (min,max);
            return (-1,-1);
    
    def SetM1PositionConstants(self,kp,ki,kd,kimax,deadzone,min,max):
            self.sendcommand(128,61)
            self.writelong(kd)
            self.writelong(kp)
            self.writelong(ki)
            self.writelong(kimax)
            self.writelong(deadzone);
            self.writelong(min);
            self.writelong(max);
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetM2PositionConstants(self,kp,ki,kd,kimax,deadzone,min,max):
            self.sendcommand(128,62)
            self.writelong(kd)
            self.writelong(kp)
            self.writelong(ki)
            self.writelong(kimax)
            self.writelong(deadzone);
            self.writelong(min);
            self.writelong(max);
            self.writebyte(self.checksum&0x7F);
            return;
    
    def readM1PositionConstants(self):
            self.sendcommand(128,63);
            p = self.readlong();
            i = self.readlong();
            d = self.readlong();
            imax = self.readlong();
            deadzone = self.readlong();
            min = self.readlong();
            max = self.readlong();
            crc = self.checksum&0x7F
            if crc==self.readbyte()&0x7F:
                    return (p,i,d,imax,deadzone,min,max);
            return (-1,-1,-1,-1,-1,-1,-1)
    
    def readM2PositionConstants(self):
            self.sendcommand(128,64);
            p = self.readlong();
            i = self.readlong();
            d = self.readlong();
            imax = self.readlong();
            deadzone = self.readlong();
            min = self.readlong();
            max = self.readlong();
            crc = self.checksum&0x7F
            if crc==self.readbyte()&0x7F:
                    return (p,i,d,imax,deadzone,min,max);
            return (-1,-1,-1,-1,-1,-1,-1)
    
    def SetM1SpeedAccelDeccelPosition(self,accel,speed,deccel,position,buffer):
            self.sendcommand(128,65)
            self.writelong(accel)
            self.writelong(speed)
            self.writelong(deccel)
            self.writelong(position)
            self.writebyte(buffer)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetM2SpeedAccelDeccelPosition(self,accel,speed,deccel,position,buffer):
            self.sendcommand(128,66)
            self.writelong(accel)
            self.writelong(speed)
            self.writelong(deccel)
            self.writelong(position)
            self.writebyte(buffer)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def SetMixedSpeedAccelDeccelPosition(self,accel1,speed1,deccel1,position1,accel2,speed2,deccel2,position2,buffer):
            self.sendcommand(128,67)
            self.writelong(accel1)
            self.writelong(speed1)
            self.writelong(deccel1)
            self.writelong(position1)
            self.writelong(accel2)
            self.writelong(speed2)
            self.writelong(deccel2)
            self.writelong(position2)
            self.writebyte(buffer)
            self.writebyte(self.checksum&0x7F);
            return;
    
    def readtemperature(self):
            self.sendcommand(128,82);
            val = self.readword()
            crc = self.checksum&0x7F
            if crc==self.readbyte()&0x7F:
                    return val
            return -1
    
    def readerrorstate(self):
            self.sendcommand(128,90);
            val = self.readword()
            crc = self.checksum&0x7F
            if crc==self.readbyte()&0x7F:
                    return val
            return -1

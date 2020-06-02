from roboclaw import Roboclaw
import time


rc = Roboclaw("/dev/ttyS0",115200)
i = rc.Open()

ver=rc.ReadVersion(0x80)
print (ver)
ver=rc.ReadVersion(0x81)
print (ver)
ver=rc.ReadVersion(0x82)
print (ver)
rc.ForwardM1(0x81, 100)
time.sleep(3)
rc.ForwardM1(0x81,0)
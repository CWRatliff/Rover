from adafruit_servokit import ServoKit
import time
kit = ServoKit(channels=16)

pan = kit.servo[15]

pan_bias = -7
pan.angle = 90+pan_bias
time.sleep(1)
# pan.angle = 85+pan_bias
# time.sleep(1)
# pan.angle = 80+pan_bias
# time.sleep(1)
# pan.angle = 75+pan_bias
# time.sleep(1)
# pan.angle = 70+pan_bias
# time.sleep(1)
# pan.angle = 45+pan_bias
# time.sleep(1)
# pan.angle = 30+pan_bias
# time.sleep(1)
pan.angle = 140+pan_bias
time.sleep(1)
pan.angle = 37+pan_bias
time.sleep(1)
pan.angle = 140+pan_bias

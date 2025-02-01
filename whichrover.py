# definitions for rover 2.0
Rcompass_bias = 98          # for canopy table (using gyro/quat, no-mag)
Rrrbias = 100
Rrfbias = 100
Rlfbias = 100
Rlrbias = 100

Rpanbias = 83

Rd1 = 7.254         #C/L to corner wheels
Rd2 = 10.5          #mid axle to fwd axle
Rd3 = 10.5          #mid axle to rear axle
Rd4 = 10.073        #C/L to mid wheels
Rspeedfactor = 40   # 4000 counts at 100%
Rspeed = ((60 * 3/12 * 2*3.14159) / 60) / 100   # (60 rpm * 3in/12 * 2 pi) / 60 = 1.57
Rsensor = False

ServoRange = 300
ServoPulseA = 500
ServoPulseB = 2500

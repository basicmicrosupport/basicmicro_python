import time
from basicmicro import Basicmicro

#Windows comport name
ctrl = Basicmicro("COM3",115200)
#Linux comport name
#ctrl = Basicmicro("/dev/ttyACM0",115200)

ctrl.Open()

test = 0
while 1:
        #Get version string
        version = ctrl.ReadVersion(0x80)
        if version[0]==False:
                print("GETVERSION Failed")
        else:
                print(repr(version[1]))
        time.sleep(1)

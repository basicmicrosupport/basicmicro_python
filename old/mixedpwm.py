import time
from basicmicro import Basicmicro

#Windows comport name
ctrl = Basicmicro("COM3",115200)
#Linux comport name
#ctrl = Basicmicro("/dev/ttyACM0",115200)

ctrl.Open()
address = 0x80

##Deprecated command examples(uncomment when using any of the deprecated examples)
##rc.ForwardMixed(address, 0)
##rc.TurnRightMixed(address, 0)

while(1):
        ##Examples using Duty commands(recommended)
        for x in range(10):
                duties = ctrl.CalcMixed(8192,0)
                ctrl.DutyM1M2(address,duties[0],duties[1])
                time.sleep(0.5)
        for x in range(10):
                duties = ctrl.CalcMixed(-8192,0)
                ctrl.DutyM1M2(address,duties[0],duties[1])
                time.sleep(0.5)
        for x in range(10):
                duties = ctrl.CalcMixed(0,8192)
                ctrl.DutyM1M2(address,duties[0],duties[1])
                time.sleep(0.5)
        for x in range(10):
                duties = ctrl.CalcMixed(0,-8192)
                ctrl.DutyM1M2(address,duties[0],duties[1])
                time.sleep(0.5)
        ctrl.DutyM1M2(address,0,0)
        time.sleep(5)
                
	##Deprecated command examples	
##	for x in range(10):
##		ctrl.ForwardMixed(address, 32)
##		time.sleep(0.5)
##	for x in range(10):
##		ctrl.BackwardMixed(address, 32)
##		time.sleep(0.5)
##	ctrl.ForwardMixed(address, 0)
##	time.sleep(5)
##
##	for x in range(10):
##		ctrl.TurnRightMixed(address, 32)
##		time.sleep(0.5)
##	for x in range(10):
##		ctrl.TurnLeftMixed(address, 32)
##		time.sleep(0.5)
##	ctrl.TurnRightMixed(address, 0)
##	time.sleep(5)

##	duty = 16
##	for x in range(10):
##		ctrl.ForwardBackwardMixed(address, 64-duty)
##		time.sleep(0.5)
##	duty = -16
##	for x in range(10):
##		ctrl.ForwardBackwardMixed(address, 64-duty)
##		time.sleep(0.5)
##	ctrl.ForwardBackwardMixed(address, 64)
##	time.sleep(5)
##	
##	duty = 16
##	for x in range(10):
##		ctrl.LeftRightMixed(address, 64-duty)
##		time.sleep(0.5)
##	duty = -16
##	for x in range(10):
##		ctrl.LeftRightMixed(address, 64-duty)
##		time.sleep(0.5)
##	ctrl.LeftRightMixed(address, 64)
##	time.sleep(5)
	

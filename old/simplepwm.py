import time
from basicmicro import Basicmicro

#Windows comport name
ctrl = Basicmicro("COM6",115200)
#Linux comport name
#ctrl = Basicmicro("/dev/ttyACM0",115200)

ctrl.Open()
address = 0x80

while(1):
        ##NOTE: commands are sent multiple times per second to
        ##keep the default serial timeouts happy on MCP(1 second).
        ##Timeouts can be changed so this is not necessary but
        ##these timeouts are usefull to prevent runaway situations.

        ##Example using DutyAccel commands(recommened method)
	for x in range(10):
		ctrl.DutyAccelM1(address,0, 8192)
		ctrl.DutyAccelM2(address,0, 8192)
		time.sleep(0.5)
	for x in range(10):
		ctrl.DutyAccelM1(address,0, -8192)
		ctrl.DutyAccelM2(address,0, -8192)
		time.sleep(0.5)
	ctrl.DutyAccelM1(address,0, 0)
	ctrl.DutyAccelM2(address,0, 0)
	time.sleep(5)

	##Eamples using deprecated single Compatibility commands
	##These call stub functions that calculate appropriate values
	##and then calls the appropriate DutyAccel commands

	##Single Direction Compatibility commands(0 to 127 = 0 to 100% duty range)
##	for x in range(10):
##		ctrl.ForwardM1(address,32)	#1/4 power forward
##		ctrl.BackwardM2(address,32)	#1/4 power backward
##		time.sleep(0.5)
##	
##	for x in range(10):
##		ctrl.BackwardM1(address,32)	#1/4 power backward
##		ctrl.ForwardM2(address,32)	#1/4 power forward
##		time.sleep(0.5)
##
##	ctrl.BackwardM1(address,0)	#Stopped
##	ctrl.ForwardM2(address,0)		#Stopped
##	time.sleep(5)

	##Dual Direction Compatibility commands(+-63 = +-100% duty range)
##	for x in range(10):
##		m1duty = 16
##		m2duty = 16
##		ctrl.ForwardBackwardM1(address,64+m1duty)	#1/4 power forward
##		ctrl.ForwardBackwardM2(address,64+m2duty)	#1/4 power backward
##		time.sleep(0.5)
##	
##	for x in range(10):
##		m1duty = -16
##		m2duty = -16
##		ctrl.ForwardBackwardM1(address,64+m1duty)	#1/4 power backward
##		ctrl.ForwardBackwardM2(address,64+m2duty)	#1/4 power forward
##		time.sleep(0.5)
##
##	ctrl.ForwardBackwardM1(address,64)	#Stopped
##	ctrl.ForwardBackwardM2(address,64)	#Stopped
##	time.sleep(5)
	

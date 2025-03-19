#***Before using this example the motor/controller combination must be
#***tuned and the settings saved to the Roboclaw using IonMotion.
#***The Min and Max Positions must be at least 0 and 50000

import time
from basicmicro import Basicmicro

#Windows comport name
ctrl = Basicmicro("COM3",115200)
#Linux comport name
#ctrl = Basicmicro("/dev/ttyACM0",115200)

def displayspeed():
	enc1 = ctrl.ReadEncM1(address)
	enc2 = ctrl.ReadEncM2(address)
	speed1 = ctrl.ReadSpeedM1(address)
	speed2 = ctrl.ReadSpeedM2(address)

	print("Encoder1:"),
	if(enc1[0]==1):
		print enc1[1],
		print format(enc1[2],'02x'),
	else:
		print "failed",
	print "Encoder2:",
	if(enc2[0]==1):
		print enc2[1],
		print format(enc2[2],'02x'),
	else:
		print "failed " ,
	print "Speed1:",
	if(speed1[0]):
		print speed1[1],
	else:
		print "failed",
	print("Speed2:"),
	if(speed2[0]):
		print speed2[1]
	else:
		print "failed "

ctrl.Open()
address = 0x80

version = ctrl.ReadVersion(address)
if version[0]==False:
	print "GETVERSION Failed"
else:
	print repr(version[1])

while(1):
	ctrl.SpeedAccelM1(address,12000,12000)
	ctrl.SpeedAccelM2(address,12000,-12000)
	for i in range(0,200):
		displayspeed()
		time.sleep(0.01)

	ctrl.SpeedAccelM1(address,12000,-12000)
	ctrl.SpeedAccelM2(address,12000,12000)
	for i in range(0,200):
		displayspeed()
		time.sleep(0.01)
  
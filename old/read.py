import time
from basicmicro import Basicmicro

#Windows comport name
ctrl = Basicmicro("COM9",115200)
#Linux comport name
#ctlr = Basicmicro("/dev/ttyACM0",115200)

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
	displayspeed()

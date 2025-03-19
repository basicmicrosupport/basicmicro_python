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
	enc1 = ctr.ReadEncM1(address)
	enc2 = ctr.ReadEncM2(address)
	speed1 = ctr.ReadSpeedM1(address)
	speed2 = ctr.ReadSpeedM2(address)

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

ctr.Open()
address = 0x80

while(1):
	print "Pos 50000"
	ctr.SpeedAccelDeccelPositionM1(address,32000,12000,32000,50000,0)
	for i in range(0,80):
		displayspeed()
		time.sleep(0.1)

	time.sleep(2)
	
	print "Pos 0"
	ctr.SpeedAccelDeccelPositionM1(address,32000,12000,32000,0,0)
	for i in range(0,80):
		displayspeed()
		time.sleep(0.1)
  
  	time.sleep(2)

import time
from basicmicro import Basicmicro

#Windows comport name
ctrl = Basicmicro("COM3",115200)
#Linux comport name
#ctrl = Basicmicro("/dev/ttyACM0",115200)

ctrl.Open()

#Get version string
for x in range(0,127):
	value = ctrl.ReadEeprom(0x80,x)
	print "EEPROM:",
	print x,
	print " ",
	if value[0]==False:
		print "Failed:",format(value[1],"#X")
	else:
		print value[1]

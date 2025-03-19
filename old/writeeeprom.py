import time
from basicmicro import Basicmicro

#Windows comport name
ctrl = Basicmicro("COM3",115200)
#Linux comport name
#ctrl = Basicmicro("/dev/ttyACM0",115200)

ctrl.Open()

#Get version string
for x in range(0,255):
	value = ctrl.WriteEeprom(0x80,x,x*2)
	print "EEPROM:",
	print x,
	print " ",
	if value==False:
		print "Failed"
	else:
		print "Written"

from basicmicro import Basicmicro

#Windows comport name
ctrl = Basicmicro("COM3",115200)
#Linux comport name
#ctrl = Basicmicro("/dev/ttyACM0",115200)

ctrl.Open()

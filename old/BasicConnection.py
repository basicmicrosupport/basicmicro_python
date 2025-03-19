import time
import logging
from basicmicro import Basicmicro

# Enable logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

# Initialize controller
# Windows COM port
controller = Basicmicro("COM3", 115200)
# Linux COM port alternative
# controller = Basicmicro("/dev/ttyACM0", 115200)

# Open connection
if controller.Open():
    print("Connected successfully")
    
    # Read firmware version
    version = controller.ReadVersion(0x80)
    if version[0]:
        print("Firmware Version: {version[1]}")
    else:
        print("Failed to read version")
    
    # Do other operations here...
    
    # Always close the connection when done
    controller.close()
else:
    print("Failed to connect to controller")

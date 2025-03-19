# ======================================================================
# PYTHON 2 COMPATIBILITY NOTICE
# ======================================================================
# This library no longer supports Python 2.7, which reached its official
# end of life on January 1, 2020. We've made this decision for several
# important reasons:
#
# 1. Security: Python 2 no longer receives security updates
# 2. Quality: Bug fixes and improvements are only happening in Python 3
# 3. Features: We're using modern Python 3 features that improve code 
#    quality and readability (like type hints and f-strings)
# 4. Industry standard: The Python community has moved to Python 3
#
# We recommend upgrading your environment to Python 3.6 or newer.
# For assistance migrating your code, see the official guide at:
# https://docs.python.org/3/howto/pyporting.html
# ======================================================================

from __future__ import print_function
from builtins import int 
import random
import serial
import time
from typing import Tuple, List, Any

import logging
logger = logging.getLogger(__name__)

class Basicmicro:
    """
    Basicmicro Interface Class for controlling Basicmicro motor controllers.
    
    This class provides a comprehensive interface for communicating with
    Basicmicro motor controllers using the Basicmicro packet serial mode. It
    supports all major functions including motor control, encoder reading,
    and configuration settings.
    
    Basic usage:
    
        #Enable logging
        import logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )

        # Initialize the controller
        dev = Basicmicro("/dev/ttyACM0", 38400)  # Port and baud rate
        dev.Open()

        # Simple motor control
        address = 0x80  # Default address
        dev.DutyM1(address, 16384)  # Half speed forward for motor 1
        dev.DutyM2(address, -8192)  # Quarter speed backward for motor 2

        # Read encoder values
        enc1 = dev.ReadEncM1(address)
        if enc1[0]:  # Check if read was successful
            print(f"Encoder 1 count: {enc1[1]}")

        # Set velocity PID values
        dev.SetM1VelocityPID(address, kp=1.0, ki=0.5, kd=0.25, qpps=44000)
    
    See the Basicmicro user manual(s) for detailed command descriptions and parameters.
    """

    # Constants to improve readability and reduce magic numbers
    MAX_RETRY_COUNT = 3
    CRC_POLYNOMIAL = 0x1021
    SUCCESS = 1
    FAILURE = 0

    def __init__(self, comport: str, rate: int, timeout: float = 0.01, retries: int = 2, verbose: bool = False):
        """Initializes the Basicmicro interface.
        
        Args:
            comport (str): The COM port to use (e.g., 'COM3')
            rate (int): The baud rate for the serial communication
            timeout (float): The timeout for serial communication
            retries (int): The number of retries for communication
            verbose (bool): Enable detailed debug logging
        """
        # Configure logger verbosity
        if verbose and logger.level > logging.DEBUG:
            logger.setLevel(logging.DEBUG)

        logger.debug(f"Initializing Basicmicro interface: port={comport}, rate={rate}, timeout={timeout}, retries={retries}")

        self._ST_Power = -1
        self._ST_Turn = -1
        self.comport = comport
        self.rate = rate
        self.timeout = timeout
        self._trystimeout = retries
        self._crc = 0
        self._port = None  # Initialize as None to handle property access before open

        # Pre-compute CRC table for faster CRC calculations
        self._CRC_TABLE = self._initialize_crc_table()

    def _initialize_crc_table(self) -> List[int]:
        """Initialize the CRC lookup table for faster CRC calculations.
        
        Returns:
            List[int]: The pre-computed CRC table
        """
        table = [0] * 256
        for i in range(256):
            crc = i << 8
            for _ in range(8):
                crc = ((crc << 1) ^ self.CRC_POLYNOMIAL) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
            table[i] = crc
        return table

    def __enter__(self) -> 'Basicmicro':
        """
        Context manager enter method for use with 'with' statement.
    
        Returns:
            Basicmicro: The Basicmicro instance
        """
        if not self.Open():
            logger.error("Failed to open connection in context manager")
            raise RuntimeError("Failed to open serial connection")
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit method for use with 'with' statement."""
        logger.debug("Exiting context manager, closing connection")
        self.close()

    def Open(self) -> bool:
        """Opens and configures the serial connection to the controller.
    
        Returns:
            bool: True if connection successful, False otherwise
    
        Raises:
            serial.SerialException: If there are issues with the serial port
            ValueError: If port parameters are invalid
        """
        try:
            # Close port if already open
            self.close()

            # Configure and open serial port
            self._port = serial.Serial(
                port=self.comport,
                baudrate=self.rate,
                timeout=1,
                write_timeout=1,
                inter_byte_timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )

            # Ensure port is open
            if not self._port.is_open:
                self._port.open()

            logger.debug("Serial port opened, clearing buffers")

            # Clear buffers
            try:
                self._port.reset_input_buffer()
                self._port.reset_output_buffer()
            except Exception as e:
                logger.warning(f"Error while clearing buffers: {str(e)}")

            # Verify communication by reading version with multiple retries
            for retry in range(self.MAX_RETRY_COUNT):
                logger.debug(f"Verifying communication attempt {retry+1}/{self.MAX_RETRY_COUNT}")
                try:
                    success, version = self.ReadVersion(0x80)
                    if success:
                        logger.info(f"Connection established. Controller version: {version}")
                        return True
                except Exception as e:
                    logger.warning(f"Exception during communication verification (attempt {retry+1}): {str(e)}")
                time.sleep(0.1)  # Short delay between retries
        
            # If we're here, communication failed
            logger.error("Failed to verify communication after multiple attempts")
            self.close()
            return False

        except (serial.SerialException, ValueError) as e:
            logger.error(f"Error opening serial port: {str(e)}")
            self.close()
            return False
        except Exception as e:
            logger.error(f"Unexpected error opening serial port: {str(e)}")
            self.close()
            return False

    def close(self) -> None:
        """Closes the serial connection to the controller."""
        logger.info(f"Closing connection to {self.comport}")
        if hasattr(self, '_port') and self._port is not None:
            try:
                if self._port.is_open:
                    self._port.close()
            except Exception as e:
                logger.error(f"Error closing serial port: {str(e)}")

    #Command Enums
    class Cmd():
        #functions deprecated. Stubs added for backwards compatibility
        M1FORWARD = 0
        M1BACKWARD = 1
        M2FORWARD = 4
        M2BACKWARD = 5
        M17BIT = 6
        M27BIT = 7
        MIXEDFORWARD = 8
        MIXEDBACKWARD = 9
        MIXEDRIGHT = 10
        MIXEDLEFT = 11
        MIXEDFB = 12
        MIXEDLR = 13
        #end stubs

        SETTIMEOUT = 14
        GETTIMEOUT = 15
        GETM1ENC = 16
        GETM2ENC = 17
        GETM1SPEED = 18
        GETM2SPEED = 19
        RESETENC = 20
        GETVERSION = 21
        SETM1ENCCOUNT = 22
        SETM2ENCCOUNT = 23
        GETMBATT = 24
        GETLBATT = 25
        
        SETM1PID = 28
        SETM2PID = 29
        GETM1ISPEED = 30
        GETM2ISPEED = 31
        M1DUTY = 32
        M2DUTY = 33
        MIXEDDUTY = 34
        M1SPEED = 35
        M2SPEED = 36
        MIXEDSPEED = 37
        M1SPEEDACCEL = 38
        M2SPEEDACCEL = 39
        MIXEDSPEEDACCEL = 40
        M1SPEEDDIST = 41
        M2SPEEDDIST = 42
        MIXEDSPEEDDIST = 43
        M1SPEEDACCELDIST = 44
        M2SPEEDACCELDIST = 45
        MIXEDSPEEDACCELDIST = 46
        GETBUFFERS = 47
        GETPWMS = 48
        GETCURRENTS = 49
        MIXEDSPEED2ACCEL = 50
        MIXEDSPEED2ACCELDIST = 51
        M1DUTYACCEL = 52
        M2DUTYACCEL = 53
        MIXEDDUTYACCEL = 54
        READM1PID = 55
        READM2PID = 56
        SETMAINVOLTAGES = 57
        SETLOGICVOLTAGES = 58
        GETMINMAXMAINVOLTAGES = 59
        GETMINMAXLOGICVOLTAGES = 60
        SETM1POSPID = 61
        SETM2POSPID = 62
        READM1POSPID = 63
        READM2POSPID = 64
        M1SPEEDACCELDECCELPOS = 65
        M2SPEEDACCELDECCELPOS = 66
        MIXEDSPEEDACCELDECCELPOS = 67
        SETM1DEFAULTACCEL = 68
        SETM2DEFAULTACCEL = 69
        SETM1DEFAULTSPEED = 70
        SETM2DEFAULTSPEED = 71
        GETDEFAULTSPEEDS = 72
        GETSTATUS = 73
        SETPINFUNCTIONS = 74
        GETPINFUNCTIONS = 75
        SETCTRLSETTINGS	= 76
        GETCTRLSETTINGS	= 77
        GETENCODERS = 78
        GETISPEEDS = 79
        RESTOREDEFAULTS = 80
        GETDEFAULTACCELS = 81
        GETTEMP = 82
        GETTEMP2 = 83
        GETERROR = 90
        GETENCODERMODE = 91
        SETM1ENCODERMODE = 92
        SETM2ENCODERMODE = 93
        WRITENVM = 94
        READNVM = 95
        SETSERIALNUMBER = 96
        GETSERIALNUMBER = 97
        SETCONFIG = 98
        GETCONFIG = 99
        GETVOLTS = 100
        GETTEMPS = 101
        SETAUXDUTYS = 102
        GETENCSTATUS = 103
        GETAUXDUTYS = 104
        SETAUTO1 = 105
        SETAUTO2 = 106
        GETAUTOS = 107
        GETSPEEDS = 108
        SETSPEEDERRORLIMIT = 109
        GETSPEEDERRORLIMIT = 110
        GETSPEEDERRORS = 111
        SETPOSERRORLIMIT = 112
        GETPOSERRORLIMIT = 113
        GETPOSERRORS = 114
        SETOFFSETS = 115
        GETOFFSETS = 116

        M1POS = 119
        M2POS = 120
        MIXEDPOS = 121
        M1SPEEDPOS = 122
        M2SPEEDPOS = 123
        MIXEDSPEEDPOS = 124
        M1PPOS = 125
        M2PPOS = 126
        MIXEDPPOS = 127
        SETM1LR = 128
        SETM2LR = 129
        GETM1LR = 130
        GETM2LR = 131
        
        SETM1MAXCURRENT = 133
        SETM2MAXCURRENT = 134
        GETM1MAXCURRENT = 135
        GETM2MAXCURRENT = 136

        SETDOUT = 137
        GETDOUTS = 138
        SETPRIORITY = 139
        GETPRIORITY = 140
        SETADDRESSMIXED = 141
        GETADDRESSMIXED = 142
        SETSIGNAL = 143
        GETSIGNALS = 144
        SETSTREAM = 145
        GETSTREAMS = 146
        GETSIGNALSDATA = 147
                          
        SETPWMMODE = 148
        GETPWMMODE = 149

        SETNODEID = 150
        GETNODEID = 151

        SETPWMIDLE = 160
        GETPWMIDLE = 161

        CANBUFFERSTATE = 180
        CANPUTPACKET = 181
        CANGETPACKET = 182

        CANOPENWRITELOCALDICT = 190
        CANOPENREADLOCALDICT = 191

        RESETESTOP = 200
        SETESTOPLOCK = 201
        GETESTOPLOCK = 202

        SETSCRIPTAUTORUN = 246
        GETSCRIPTAUTORUN = 247
        STARTSCRIPT = 248
        STOPSCRIPT = 249

        READEEPROM = 252
        WRITEEEPROM = 253
            
    #Private Functions

    # CRC and core communication methods
    def crc_clear(self) -> None:
        """Clears the CRC value."""
        self._crc = 0
        
    def crc_update(self, data: int) -> None:
        """Updates the CRC value with the given data.
    
        Args:
            data (int): The data to update the CRC with
        """
        # Use a faster lookup table approach for CRC calculation
        self._crc = ((self._crc << 8) ^ self._CRC_TABLE[(self._crc >> 8) ^ (data & 0xFF)]) & 0xFFFF

    def _sendcommand(self, address: int, command: int) -> None:
        """Sends a command to the controller.
        
        Args:
            address (int): The address of the controller
            command (int): The command to send
        """
        logger.debug(f"Sending command: address=0x{address:02x}, command=0x{command:02x}")
        self.crc_clear()
        data = [address, command]
        self.crc_update(data[0])
        self.crc_update(data[1])
        try:
            self._port.write(bytes(data))
        except Exception as e:
            logger.error(f"Failed to send command: {str(e)}")
            raise
        
    def _readbyte(self):
        """Reads a byte from the controller.
        
        Returns:
            tuple: (success, value)
                success (bool): True if read successful
                value (int): The byte value
        """
        try:
            data = bytearray(self._port.read(1))
            if len(data)==1:
                val = data[0]&0xFF
                self.crc_update(val)
                return (1,val)  
            return (0,0)
        except Exception as e:
            logger.error(f"Error reading byte: {str(e)}")
            return (0,0)
        
    def _readword(self):
        """Reads a 16-bit word from the controller.
        
        Returns:
            tuple: (success, value)
                success (bool): True if read successful
                value (int): The word value
        """
        try:
            data = bytearray(self._port.read(2))
            if len(data)==2:
                self.crc_update(data[0])
                self.crc_update(data[1])
                return (1,(data[0]<<8)|data[1])
            return (0,0)
        except Exception as e:
            logger.error(f"Error reading word: {str(e)}")
            return (0,0)

    def _readlong(self):
        """Reads a 32-bit long value from the controller.
        
        Returns:
            tuple: (success, value)
                success (bool): True if read successful
                value (int): The long value
        """
        try:
            data = bytearray(self._port.read(4))
            if len(data)==4:
                self.crc_update(data[0])
                self.crc_update(data[1])
                self.crc_update(data[2])
                self.crc_update(data[3])
                return (1,(data[0]<<24)|(data[1]<<16)|(data[2]<<8)|data[3])
            return (0,0)
        except Exception as e:
            logger.error(f"Error reading long: {str(e)}")
            return (0,0)

    def _readslong(self):
        """Reads a signed 32-bit long value from the controller.
        
        Returns:
            tuple: (success, value)
                success (bool): True if read successful
                value (int): The signed long value
        """
        val = self._readlong()
        if val[0]:
            if val[1]&0x80000000:
                return (val[0],val[1]-0x100000000)
            return (val[0],val[1])
        return (0,0)

    def _writebyte(self,val):
        """Writes a byte to the controller.
        
        Args:
            val (int): The byte value to write
        """
        data = bytearray([val&0xFF])
        self.crc_update(data[0])
        try:
            self._port.write(data)
        except Exception as e:
            logger.error(f"Error writing byte: {str(e)}")
            raise

    def _writesbyte(self,val):
        """Writes a signed byte to the controller.
        
        Args:
            val (int): The signed byte value to write
        """
        self._writebyte(val)

    def _writeword(self,val):
        """Writes a 16-bit word to the controller.
        
        Args:
            val (int): The word value to write
        """
        data = bytearray([(val>>8)&0xFF,val&0xFF])
        self.crc_update(data[0])
        self.crc_update(data[1])
        try:
            self._port.write(data)
        except Exception as e:
            logger.error(f"Error writing word: {str(e)}")
            raise
        
    def _writesword(self,val):
        """Writes a signed 16-bit word to the controller.
        
        Args:
            val (int): The signed word value to write
        """
        self._writeword(val)

    def _writelong(self,val):
        """Writes a 32-bit long value to the controller.
        
        Args:
            val (int): The long value to write
        """
        data = bytearray([(val>>24)&0xFF,(val>>16)&0xFF,(val>>8)&0xFF,val&0xFF])
        self.crc_update(data[0])
        self.crc_update(data[1])
        self.crc_update(data[2])
        self.crc_update(data[3])
        try:
            self._port.write(data)
        except Exception as e:
            logger.error(f"Error writing long: {str(e)}")
            raise

    def _writeslong(self,val):
        """Writes a signed 32-bit long value to the controller.
        
        Args:
            val (int): The signed long value to write
        """
        self._writelong(val)

    def _write(self, address: int, cmd: int, *args, **kwargs):
        """
        Generic write method that sends a command to the controller with variable arguments.

        This method sends the address, command, optional data values, and CRC16 checksum to
        the controller. If no arguments are provided, it only sends the address, command, 
        and CRC16.

        Args:
            address (int): The address of the controller
            cmd (int): The command to send
            *args: Variable number of arguments to write
            **kwargs: Keyword arguments that specify how to write each argument
                - types (list or str): Specifies the data type for each argument
                    Supported types: 'byte', 'sbyte', 'word', 'sword', 'long', 'slong'
                - defaults to 'byte' if not specified

        Returns:
            bool: True if successful, False otherwise
        """
        logger.debug(f"Write: address=0x{address:02x}, cmd=0x{cmd:02x}, args={args}")
        # Determine the types of arguments
        arg_types = kwargs.get('types', ['byte'] * len(args))
        if isinstance(arg_types, str):
            arg_types = arg_types.split(',')

        if len(arg_types) != len(args):
            raise ValueError(f"Number of type specifications ({len(arg_types)}) must match number of arguments ({len(args)})")

        # Start retry loop
        for _ in range(self._trystimeout):
            # Send command (address and command)
            self._sendcommand(address, cmd)

            # Write each argument according to its type
            for arg, arg_type in zip(args, arg_types):
                arg_type = arg_type.lower()
                if arg_type == 'byte':
                    self._writebyte(arg)
                elif arg_type == 'sbyte':
                    self._writesbyte(arg)
                elif arg_type == 'word':
                    self._writeword(arg)
                elif arg_type == 'sword':
                    self._writesword(arg)
                elif arg_type == 'long':
                    self._writelong(arg)
                elif arg_type == 'slong':
                    self._writeslong(arg)
                else:
                    raise ValueError(f"Unsupported type: {arg_type}")

            # Write checksum and verify we received acknowledgment
            if self._writechecksum():
                return True

        logger.warning(f"Write failed after {self._trystimeout} attempts: address=0x{address:02x}, cmd=0x{cmd:02x}")
        return False

    def _writechecksum(self):
        """Writes 16-bit CRC and reads one byte (ack) from the controller.
        
        Returns:
            bool: True if successful
        """
        logger.debug(f"Writing checksum: 0x{self._crc & 0xFFFF:04x}")
        self._writeword(self._crc&0xFFFF)
        val = self._readbyte()
        if val[0]:
            return True
        logger.debug("No acknowledgment received")
        return False

    def _read(self, address: int, cmd: int, *args, **kwargs):
        """
        Generic read method that reads data from the controller based on specified types.

        This method sends a command to the controller and reads back data types
        specified in the types parameter, verifying the CRC16 checksum.

        Args:
            address (int): The address of the controller
            cmd (int): The command to send
            *args: Optional positional arguments (unused, for compatibility)
            **kwargs: Keyword arguments that specify how to read data
                - types (list or str): Specifies the data types to read
                    Supported types: 'byte', 'sbyte', 'word', 'sword', 'long', 'slong'
                - retry_on_error (bool): Whether to retry on error, defaults to True

        Returns:
            tuple: (success, *values)
                success (bool): True if read successful
                *values: The values read according to the specified types
        """
        logger.debug(f"Read: address=0x{address:02x}, cmd=0x{cmd:02x}")
        retry_on_error = kwargs.get('retry_on_error', True)
        arg_types = kwargs.get('types', [])
        if isinstance(arg_types, str):
            arg_types = arg_types.split(',')

        def read_value(arg_type):
            if arg_type == 'byte':
                return self._readbyte()
            elif arg_type == 'sbyte':
                val = self._readbyte()
                return (val[0], val[1] - 0x100 if val[0] and val[1] & 0x80 else val[1])
            elif arg_type == 'word':
                return self._readword()
            elif arg_type == 'sword':
                val = self._readword()
                return (val[0], val[1] - 0x10000 if val[0] and val[1] & 0x8000 else val[1])
            elif arg_type == 'long':
                return self._readlong()
            elif arg_type == 'slong':
                return self._readslong()
            else:
                raise ValueError(f"Unsupported type: {arg_type}")

        trys = self._trystimeout if retry_on_error else 1
        while trys:
            try:
                self._port.flushInput()
            except Exception as e:
                logger.warning(f"Error flushing input buffer: {str(e)}")
                # Continue trying despite the error
            self._sendcommand(address, cmd)
            result = [self.SUCCESS]

            for arg_type in arg_types:
                val = read_value(arg_type)
                if not val[0]:
                    break
                result.append(val[1])
            else:
                crc = self._readchecksumword()
                if crc[0] and self._crc & 0xFFFF == crc[1] & 0xFFFF:
                    return tuple(result)

            trys -= 1

        logger.warning(f"Read failed after {trys} attempts: address=0x{address:02x}, cmd=0x{cmd:02x}")
        return tuple([self.FAILURE] + [0] * len(arg_types))

    def _readchecksumword(self):
        """Reads a 16-bit checksum word from the controller.

        Returns:
            tuple: (success, checksum)
                success (bool): True if read successful
                checksum (int): The checksum value
        """
        logger.debug("Reading checksum word")
        try:
            # Try to read 2 bytes from the serial port
            data = bytearray(self._port.read(2))

            # Check if we received exactly 2 bytes (complete checksum)
            if len(data) == 2:
                # Combine the bytes into a 16-bit word (big-endian)
                # First byte is high byte, second is low byte
                checksum = ((data[0] & 0xFF) << 8) | (data[1] & 0xFF)
                return (self.SUCCESS, checksum)

            # If we didn't get 2 bytes, the read failed
            logger.debug("Failed to read checksum: incomplete data")
            return (self.FAILURE, 0)
        except (serial.SerialException, ValueError) as e:
            # Handle specific exceptions related to serial communication
            logger.error(f"Serial error while reading checksum: {str(e)}")
            return (self.FAILURE, 0)
        except Exception as e:
            # Catch any other unexpected exceptions
            logger.error(f"Unexpected error reading checksum: {str(e)}")
            return (self.FAILURE, 0)

    def _ST_Single(self, cmd, address, power):
        """Utility Function for Stubs. Sets the power for a single motor.
        
        Args:
            cmd (int): The command to send
            address (int): The address of the controller
            power (int): The power value to set
        
        Returns:
            bool: True if successful
        """
        self._ST_Power = -128
        self._ST_Turn = -128

        power = power&0x7F      
        if cmd==self.Cmd.M17BIT or cmd==self.Cmd.M27BIT:
            if power==0: power=1 ##keep Fwd/Bwd power range symetric
            power = (power*2)-128
        if cmd==self.Cmd.M1BACKWARD or cmd==self.Cmd.M2BACKWARD:
            power = -power;

        ##power = +-127 at this point
            
        duty = power*32767/127
        if cmd==self.Cmd.M1FORWARD or cmd==self.Cmd.M1BACKWARD or cmd==self.Cmd.M17BIT:
            return self.DutyAccelM1(address,0,duty)
        else:
            return self.DutyAccelM2(address,0,duty)

    def _ST_Mixed(self, cmd, address, power):
        """Utility Function for Stubs. Sets the power and turn for mixed mode.
        
        Args:
            cmd (int): The command to send
            address (int): The address of the controller
            power (int): The power value to set
        
        Returns:
            bool: True if successful
        """
        power = power&0x7F;
        if cmd<self.Cmd.MIXEDFB:
            ##Regular mode calculation
            if cmd&0x1: power = -power
        else:
            ##7bit mode calculation
            if power==0: power=1  ##keep Fwd/Bwd power range symetric(eg +-127 from center)
            power = (power*2)-128

        ##temp == +-127 at this point
            
        if cmd==self.Cmd.MIXEDRIGHT or cmd==self.Cmd.MIXEDLEFT or cmd==self.Cmd.MIXEDLR:
            self._ST_Turn = power
        else:
            self._ST_Power = power;

        if self._ST_Power!=-128 and self._ST_Turn!=-128:
            duties = self.CalcMixed(self._ST_Power*32767/127, self._ST_Turn*32767/127)
            return self.DutyM1M2(address,duties[0],duties[1])
        return False    ##Both power and turn commands must be used at least once.  Will return false until then
    
    def SendRandomData(self,cnt):
        """Sends random data to the controller. Used for testing only
    
        Args:
            cnt (int): The number of random bytes to send
        """
        try:
            for i in range(0,cnt):
                byte = random.getrandbits(8)
                self._port.write(byte&0xFF)
            return
        except Exception as e:
            logger.error(f"Error sending random data: {str(e)}")   

    def CalcMixed(self, fb, lr):
        """Utility Function for Stubs. Calculates mixed mode values.
        
        Args:
            fb (int): Forward/backward value
            lr (int): Left/right value
        
        Returns:
            list: List of mixed mode values
        """
        ##calc mixing
        if (lr^fb)<0: ##signs are different?
            if abs(lr)>abs(fb):
                out1 = -lr
            else:
                out1 = fb
            out0 = fb+lr
        else:
            if abs(fb)>abs(lr):
                out0 = fb
            else:
                out0 = lr
            out1 = fb-lr
        return [out0,out1]                
        
    def ForwardM1(self,address,val):        
        """
        Sets the power for motor 1 to move forward.

        Args:
            address (int): The address of the controller.
            val (int): The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Single(self.Cmd.M1FORWARD,address,val)

    def BackwardM1(self,address,val):
        """
        Sets the power for motor 1 to move backward.

        Args:
            address (int): The address of the controller.
            val (int): The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Single(self.Cmd.M1BACKWARD,address,val)

    def SetMinVoltageMainBattery(self,address,val):
        return False    #Deprecated

    def SetMaxVoltageMainBattery(self,address,val):
        return False    #Deprecated

    def ForwardM2(self,address,val):
        """
        Sets the power for motor 2 to move forward.

        Args:
            address (int): The address of the controller.
            val (int): The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Single(self.Cmd.M2FORWARD,address,val)

    def BackwardM2(self,address,val):
        """
        Sets the power for motor 2 to move backward.

        Args:
            address (int): The address of the controller.
            val (int): The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Single(self.Cmd.M2BACKWARD,address,val)

    def ForwardBackwardM1(self,address,val):
        """
        Sets the power for motor 1 to move forward or backward in 7-bit mode.

        Args:
            address (int): The address of the controller.
            val (int): The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Single(self.Cmd.M17BIT,address,val)

    def ForwardBackwardM2(self,address,val):
        """
        Sets the power for motor 2 to move forward or backward in 7-bit mode.

        Args:
            address (int): The address of the controller.
            val (int): The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Single(self.Cmd.M27BIT,address,val)

    def ForwardMixed(self,address,val):
        """
        Sets the power for mixed mode to move forward.

        Args:
            address (int): The address of the controller.
            val (int): The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Mixed(self.Cmd.MIXEDFORWARD,address,val)

    def BackwardMixed(self,address,val):
        """
        Sets the power for mixed mode to move backward.

        Args:
            address (int): The address of the controller.
            val (int): The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Mixed(self.Cmd.MIXEDBACKWARD,address,val)

    def TurnRightMixed(self,address,val):
        """
        Sets the power for mixed mode to turn right.

        Args:
            address (int): The address of the controller.
            val (int): The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Mixed(self.Cmd.MIXEDRIGHT,address,val)

    def TurnLeftMixed(self,address,val):
        """
        Sets the power for mixed mode to turn left.

        Args:
            address (int): The address of the controller.
            val (int): The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Mixed(self.Cmd.MIXEDLEFT,address,val)

    def ForwardBackwardMixed(self,address,val):
        """
        Sets the power for mixed mode to move forward or backward.

        Args:
            address (int): The address of the controller.
            val (int): The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Mixed(self.Cmd.MIXEDFB,address,val)

    def LeftRightMixed(self,address,val):
        """
        Sets the power for mixed mode to move left or right.

        Args:
            address (int): The address of the controller.
            val (int): The power value to set (0-127).

        Returns:
            bool: True if successful.
        """
        return self._ST_Mixed(self.Cmd.MIXEDLR,address,val)

    #Packet Serial Commands
    def SetTimeout(self,address,timeout):
        """
        Sets the timeout for motor 1 encoder.

        Args:
            address (int): The address of the controller.
            timeout (int): The timeout value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.SETTIMEOUT,int(timeout*10),types=["byte"])

    def GetTimeout(self,address):
        """
        Reads the timeout for motor 1 encoder.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, timeout)
                success (bool): True if read successful.
                timeout (int): The timeout value.
        """
        val = self._read(address,self.Cmd.GETTIMEOUT,types=["byte"])
        if val[0]:
            return (1, float(val[1])/10)
        return (0, 0)

    def ReadEncM1(self,address):
        """
        Reads the encoder count for motor 1.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, count, status)
                success (bool): True if read successful.
                count (int): The encoder count.
                status (int): The status byte.
        """
        return self._read(address,self.Cmd.GETM1ENC,types=["long", "byte"])

    def ReadEncM2(self,address):
        """
        Reads the encoder count for motor 2.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, count, status)
                success (bool): True if read successful.
                count (int): The encoder count.
                status (int): The status byte.
        """
        return self._read(address,self.Cmd.GETM2ENC,types=["long", "byte"])

    def ReadSpeedM1(self,address):
        """
        Reads the speed for motor 1.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, speed, status)
                success (bool): True if read successful.
                speed (int): The speed value.
                status (int): The status byte.
        """
        return self._read(address,self.Cmd.GETM1SPEED,types=["long", "byte"])

    def ReadSpeedM2(self,address):
        """
        Reads the speed for motor 2.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, speed, status)
                success (bool): True if read successful.
                speed (int): The speed value.
                status (int): The status byte.
        """
        return self._read(address,self.Cmd.GETM2SPEED,types=["long", "byte"])

    def ResetEncoders(self,address):
        """
        Resets the encoders for both motors.

        Args:
            address (int): The address of the controller.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.RESETENC)

    def ReadVersion(self, address):
        """
        Reads the firmware version of the controller.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, version)
                success (bool): True if read successful.
                version (str): The firmware version.
        """
        logger.debug(f"Reading firmware version from address=0x{address:02x}")
        for _ in range(self._trystimeout):
            try:
                self._port.flushInput()
            except Exception as e:
                logger.warning(f"Error flushing input buffer during version read: {str(e)}")
                # Continue trying despite the error
            self._sendcommand(address, self.Cmd.GETVERSION)
            version = []
            passed = True
            for _ in range(48):
                try:
                    data = self._port.read(1)
                    if data:
                        self.crc_update(data[0])
                        if data[0] == 0:
                            break
                        version.append(chr(data[0]))
                    else:
                        logger.debug("Timeout while reading version string")
                        passed = False
                        break
                except Exception as e:
                    logger.debug(f"Error reading version character: {str(e)}")
                    passed = False
                    break
            if passed:
                crc = self._readchecksumword()
                if crc[0] and self._crc & 0xFFFF == crc[1] & 0xFFFF:
                    return (1, ''.join(version))
                else:
                    logger.debug(f"CRC check failed: received=0x{crc[1]:04x}, calculated=0x{self._crc & 0xFFFF:04x}")

            logger.debug("Retrying version read after short delay")
            time.sleep(0.01)
    
        logger.warning(f"Failed to read version from address=0x{address:02x} after {self._trystimeout} attempts")
        return (0, "")

    def SetEncM1(self,address,cnt):
        """
        Sets the encoder count for motor 1.

        Args:
            address (int): The address of the controller.
            cnt (int): The encoder count to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.SETM1ENCCOUNT,cnt,types=["long"])

    def SetEncM2(self,address,cnt):
        """
        Sets the encoder count for motor 2.

        Args:
            address (int): The address of the controller.
            cnt (int): The encoder count to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.SETM2ENCCOUNT,cnt,types=["long"])

    def ReadMainBatteryVoltage(self,address):
        """
        Reads the main battery voltage.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, voltage)
                success (bool): True if read successful.
                voltage (int): The main battery voltage.
        """
        return self._read(address,self.Cmd.GETMBATT,types=["word"])

    def ReadLogicBatteryVoltage(self,address):
        """
        Reads the logic battery voltage.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, voltage)
                success (bool): True if read successful.
                voltage (int): The logic battery voltage.
        """
        return self._read(address,self.Cmd.GETLBATT,types=["word"])

    def SetMinVoltageLogicBattery(self,address,val):
        """
        Deprecated: Sets the minimum voltage for the logic battery.

        Args:
            address (int): The address of the controller.
            val (int): The minimum voltage value to set.

        Returns:
            bool: Always returns False.
        """
        return False #deprecated

    def SetMaxVoltageLogicBattery(self,address,val):
        """
        Deprecated: Sets the maximum voltage for the logic battery.

        Args:
            address (int): The address of the controller.
            val (int): The maximum voltage value to set.

        Returns:
            bool: Always returns False.
        """
        return False #deprecated

    def SetM1VelocityPID(self,address,p,i,d,qpps):
        """
        Sets the velocity PID constants for motor 1.

        Args:
            address (int): The address of the controller.
            p (float): The proportional constant.
            i (float): The integral constant.
            d (float): The derivative constant.
            qpps (int): The speed in quadrature pulses per second.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.SETM1PID,int(d*65536),int(p*65536),int(i*65536),qpps,types=["long","long","long","long"])

    def SetM2VelocityPID(self,address,p,i,d,qpps):
        """
        Sets the velocity PID constants for motor 2.

        Args:
            address (int): The address of the controller.
            p (float): The proportional constant.
            i (float): The integral constant.
            d (float): The derivative constant.
            qpps (int): The speed in quadrature pulses per second.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.SETM2PID,int(d*65536),int(p*65536),int(i*65536),qpps,types=["long","long","long","long"])

    def ReadISpeedM1(self,address):
        """
        Reads the instantaneous speed for motor 1.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, speed, status)
                success (bool): True if read successful.
                speed (int): The instantaneous speed value.
                status (int): The status byte.
        """
        return self._read(address,self.Cmd.GETM1ISPEED,types=["long", "byte"])

    def ReadISpeedM2(self,address):
        """
        Reads the instantaneous speed for motor 2.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, speed, status)
                success (bool): True if read successful.
                speed (int): The instantaneous speed value.
                status (int): The status byte.
        """
        return self._read(address,self.Cmd.GETM2ISPEED,types=["long", "byte"])

    def DutyM1(self,address,val):
        """
        Sets the duty cycle for motor 1.

        Args:
            address (int): The address of the controller.
            val (int): The duty cycle value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.M1DUTY,val,types=["sword"])

    def DutyM2(self,address,val):
        """
        Sets the duty cycle for motor 2.

        Args:
            address (int): The address of the controller.
            val (int): The duty cycle value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.M2DUTY,val,types=["sword"])

    def DutyM1M2(self,address,m1,m2):
        """
        Sets the duty cycle for both motors.

        Args:
            address (int): The address of the controller.
            m1 (int): The duty cycle value for motor 1.
            m2 (int): The duty cycle value for motor 2.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.MIXEDDUTY,m1,m2,types=["sword","sword"])

    def SpeedM1(self,address,val):
        """
        Sets the speed for motor 1.

        Args:
            address (int): The address of the controller.
            val (int): The speed value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.M1SPEED,val,types=["slong"])

    def SpeedM2(self,address,val):
        """
        Sets the speed for motor 2.

        Args:
            address (int): The address of the controller.
            val (int): The speed value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.M2SPEED,val,types=["slong"])

    def SpeedM1M2(self,address,m1,m2):
        """
        Sets the speed for both motors.

        Args:
            address (int): The address of the controller.
            m1 (int): The speed value for motor 1.
            m2 (int): The speed value for motor 2.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.MIXEDSPEED,m1,m2,types=["slong","slong"])

    def SpeedAccelM1(self,address,accel,speed):
        """
        Sets the acceleration and speed for motor 1.

        Args:
            address (int): The address of the controller.
            accel (int): The acceleration value to set.
            speed (int): The speed value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.M1SPEEDACCEL,accel,speed,types=["long","slong"])

    def SpeedAccelM2(self,address,accel,speed):
        """
        Sets the acceleration and speed for motor 2.

        Args:
            address (int): The address of the controller.
            accel (int): The acceleration value to set.
            speed (int): The speed value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.M2SPEEDACCEL,accel,speed,types=["long","slong"])

    def SpeedAccelM1M2(self,address,accel,speed1,speed2):
        """
        Sets the acceleration and speed for both motors.

        Args:
            address (int): The address of the controller.
            accel (int): The acceleration value to set.
            speed1 (int): The speed value for motor 1.
            speed2 (int): The speed value for motor 2.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.MIXEDSPEEDACCEL,accel,speed1,speed2,types=["long","slong","slong"])

    def SpeedDistanceM1(self,address,speed,distance,buffer):
        """
        Sets the speed and distance for motor 1.

        Args:
            address (int): The address of the controller.
            speed (int): The speed value to set.
            distance (int): The distance value to set.
            buffer (int): The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.M1SPEEDDIST,speed,distance,buffer,types=["slong","long","byte"])

    def SpeedDistanceM2(self,address,speed,distance,buffer):
        """
        Sets the speed and distance for motor 2.

        Args:
            address (int): The address of the controller.
            speed (int): The speed value to set.
            distance (int): The distance value to set.
            buffer (int): The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.M2SPEEDDIST,speed,distance,buffer,types=["slong","long","byte"])

    def SpeedDistanceM1M2(self,address,speed1,distance1,speed2,distance2,buffer):
        """
        Sets the speed and distance for both motors.

        Args:
            address (int): The address of the controller.
            speed1 (int): The speed value for motor 1.
            distance1 (int): The distance value for motor 1.
            speed2 (int): The speed value for motor 2.
            distance2 (int): The distance value for motor 2.
            buffer (int): The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.MIXEDSPEEDDIST,speed1,distance1,speed2,distance2,buffer,types=["slong","long","slong","long","byte"])

    def SpeedAccelDistanceM1(self,address,accel,speed,distance,buffer):
        """
        Sets the acceleration, speed, and distance for motor 1.

        Args:
            address (int): The address of the controller.
            accel (int): The acceleration value to set.
            speed (int): The speed value to set.
            distance (int): The distance value to set.
            buffer (int): The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.M1SPEEDACCELDIST,accel,speed,distance,buffer,types=["long","slong","long","byte"])

    def SpeedAccelDistanceM2(self,address,accel,speed,distance,buffer):
        """
        Sets the acceleration, speed, and distance for motor 2.

        Args:
            address (int): The address of the controller.
            accel (int): The acceleration value to set.
            speed (int): The speed value to set.
            distance (int): The distance value to set.
            buffer (int): The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.M2SPEEDACCELDIST,accel,speed,distance,buffer,types=["long","slong","long","byte"])

    def SpeedAccelDistanceM1M2(self,address,accel,speed1,distance1,speed2,distance2,buffer):
        """
        Sets the acceleration, speed, and distance for both motors.

        Args:
            address (int): The address of the controller.
            accel (int): The acceleration value to set.
            speed1 (int): The speed value for motor 1.
            distance1 (int): The distance value for motor 1.
            speed2 (int): The speed value for motor 2.
            distance2 (int): The distance value for motor 2.
            buffer (int): The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.MIXEDSPEEDACCELDIST,accel,speed1,distance1,speed2,distance2,buffer,types=["long","slong","long","slong","long","byte"])

    def ReadBuffers(self,address):
        """
        Reads the buffer status.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, buffer1, buffer2)
                success (bool): True if read successful.
                buffer1 (int): The status of buffer 1.
                buffer2 (int): The status of buffer 2.
        """
        return self._read(address,self.Cmd.GETBUFFERS,types=["byte","byte"])

    def ReadPWMs(self,address):
        """
        Reads the PWM values.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, pwm1, pwm2)
                success (bool): True if read successful.
                pwm1 (int): The PWM value for motor 1.
                pwm2 (int): The PWM value for motor 2.
        """
        val = self._read(address,self.Cmd.GETPWMS,types=["word","word"])
        if val[0]:
            pwm1 = val[1]
            pwm2 = val[2]
            if pwm1&0x8000:
                pwm1-=0x10000
            if pwm2&0x8000:
                pwm2-=0x10000
            return (1,pwm1,pwm2)
        return (0,0,0)

    def ReadCurrents(self,address):
        """
        Reads the current values.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, current1, current2)
                success (bool): True if read successful.
                current1 (int): The current value for motor 1.
                current2 (int): The current value for motor 2.
        """
        val = self._read(address,self.Cmd.GETCURRENTS,types=["word","word"])
        if val[0]:
            cur1 = val[1]
            cur2 = val[2]
            if cur1&0x8000:
                cur1-=0x10000
            if cur2&0x8000:
                cur2-=0x10000
            return (1,cur1,cur2)
        return (0,0,0)

    def SpeedAccelM1M2_2(self,address,accel1,speed1,accel2,speed2):
        """
        Sets the acceleration and speed for both motors with different accelerations.

        Args:
            address (int): The address of the controller.
            accel1 (int): The acceleration value for motor 1.
            speed1 (int): The speed value for motor 1.
            accel2 (int): The acceleration value for motor 2.
            speed2 (int): The speed value for motor 2.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.MIXEDSPEED2ACCEL,accel1,speed1,accel2,speed2,types=["long","slong","long","slong"])

    def SpeedAccelDistanceM1M2_2(self,address,accel1,speed1,distance1,accel2,speed2,distance2,buffer):
        """
        Sets the acceleration, speed, and distance for both motors with different accelerations.

        Args:
            address (int): The address of the controller.
            accel1 (int): The acceleration value for motor 1.
            speed1 (int): The speed value for motor 1.
            distance1 (int): The distance value for motor 1.
            accel2 (int): The acceleration value for motor 2.
            speed2 (int): The speed value for motor 2.
            distance2 (int): The distance value for motor 2.
            buffer (int): The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.MIXEDSPEED2ACCELDIST,accel1,speed1,distance1,accel2,speed2,distance2,buffer,types=["long","slong","long","slong","long","long","byte"])

    def DutyAccelM1(self,address,accel,duty):
        """
        Sets the acceleration and duty cycle for motor 1.

        Args:
            address (int): The address of the controller.
            accel (int): The acceleration value to set.
            duty (int): The duty cycle value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.M1DUTYACCEL,duty,accel,types=["sword","long"])

    def DutyAccelM2(self,address,accel,duty):
        """
        Sets the acceleration and duty cycle for motor 2.

        Args:
            address (int): The address of the controller.
            accel (int): The acceleration value to set.
            duty (int): The duty cycle value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.M2DUTYACCEL,duty,accel,types=["sword","long"])

    def DutyAccelM1M2(self,address,accel1,duty1,accel2,duty2):
        """
        Sets the acceleration and duty cycle for both motors.

        Args:
            address (int): The address of the controller.
            accel1 (int): The acceleration value for motor 1.
            duty1 (int): The duty cycle value for motor 1.
            accel2 (int): The acceleration value for motor 2.
            duty2 (int): The duty cycle value for motor 2.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.MIXEDDUTYACCEL,duty1,accel1,duty2,accel2,types=["sword","long","sword","long"])
        
    def ReadM1VelocityPID(self,address):
        """
        Reads the velocity PID constants for motor 1.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, p, i, d, qpps)
                success (bool): True if read successful.
                p (float): The proportional constant.
                i (float): The integral constant.
                d (float): The derivative constant.
                qpps (int): The speed in quadrature pulses per second.
        """
        data = self._read(address,self.Cmd.READM1PID,types=["long","long","long","long"])
        if data[0]:
            data[1]/=65536.0
            data[2]/=65536.0
            data[3]/=65536.0
            return data
        return (0,0,0,0,0)

    def ReadM2VelocityPID(self,address):
        """
        Reads the velocity PID constants for motor 2.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, p, i, d, qpps)
                success (bool): True if read successful.
                p (float): The proportional constant.
                i (float): The integral constant.
                d (float): The derivative constant.
                qpps (int): The speed in quadrature pulses per second.
        """
        data = self._read(address,self.Cmd.READM2PID,types=["long","long","long","long"])
        if data[0]:
            data[1]/=65536.0
            data[2]/=65536.0
            data[3]/=65536.0
            return data
        return (0,0,0,0,0)

    def SetMainVoltages(self,address,min, max, auto_offset):
        """
        Sets the main battery voltage limits.

        Args:
            address (int): The address of the controller.
            min (int): The minimum voltage value to set.
            max (int): The maximum voltage value to set.
            auto_offset (int): The auto offset value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.SETMAINVOLTAGES,min,max,auto_offset,types=["word","word","byte"])
        
    def SetLogicVoltages(self,address,min, max):
        """
        Sets the logic battery voltage limits.

        Args:
            address (int): The address of the controller.
            min (int): The minimum voltage value to set.
            max (int): The maximum voltage value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.SETLOGICVOLTAGES,min,max,types=["word","word"])
        
    def ReadMinMaxMainVoltages(self,address):
        """
        Reads the main battery voltage limits.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, min, max, auto_offset)
                success (bool): True if read successful.
                min (int): The minimum voltage value.
                max (int): The maximum voltage value.
                auto_offset (int): The auto offset value.
        """
        return self._read(address,self.Cmd.GETMINMAXMAINVOLTAGES,types=["word","word","byte"])

    def ReadMinMaxLogicVoltages(self,address):
        """
        Reads the logic battery voltage limits.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, min, max)
                success (bool): True if read successful.
                min (int): The minimum voltage value.
                max (int): The maximum voltage value.
        """
        return self._read(address,self.Cmd.GETMINMAXLOGICVOLTAGES,types=["word","word"])

    def SetM1PositionPID(self,address,kp,ki,kd,kimax,deadzone,min,max):
        """
        Sets the position PID constants for motor 1.

        Args:
            address (int): The address of the controller.
            kp (float): The proportional constant.
            ki (float): The integral constant.
            kd (float): The derivative constant.
            kimax (int): The integral limit.
            deadzone (int): The deadzone value.
            min (int): The minimum position value.
            max (int): The maximum position value.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.SETM1POSPID,int(kd*1024),int(kp*1024),int(ki*1024),kimax,deadzone,min,max,types=["long","long","long","long","long","long","long"])

    def SetM2PositionPID(self,address,kp,ki,kd,kimax,deadzone,min,max):
        """
        Sets the position PID constants for motor 2.

        Args:
            address (int): The address of the controller.
            kp (float): The proportional constant.
            ki (float): The integral constant.
            kd (float): The derivative constant.
            kimax (int): The integral limit.
            deadzone (int): The deadzone value.
            min (int): The minimum position value.
            max (int): The maximum position value.

        Returns:
            bool: True if successful.
        """
        return self._write(address,self.Cmd.SETM2POSPID,int(kd*1024),int(kp*1024),int(ki*1024),kimax,deadzone,min,max,types=["long","long","long","long","long","long","long"])

    def ReadM1PositionPID(self, address):
        """
        Reads the position PID constants for motor 1.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, kp, ki, kd, kimax, deadzone, min, max)
                success (bool): True if read successful.
                kp (float): The proportional constant.
                ki (float): The integral constant.
                kd (float): The derivative constant.
                kimax (int): The integral limit.
                deadzone (int): The deadzone value.
                min (int): The minimum position value.
                max (int): The maximum position value.
        """
        data = self._read(address, self.Cmd.READM1POSPID, types=["long", "long", "long", "long", "long", "long", "long"])
        if data[0]:
            return (data[0], data[1] / 1024.0, data[2] / 1024.0, data[3] / 1024.0, data[4], data[5], data[6], data[7])
        return (0, 0, 0, 0, 0, 0, 0, 0)
        
    def ReadM2PositionPID(self,address):
        """
        Reads the position PID constants for motor 2.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, kp, ki, kd, kimax, deadzone, min, max)
                success (bool): True if read successful.
                kp (float): The proportional constant.
                ki (float): The integral constant.
                kd (float): The derivative constant.
                kimax (int): The integral limit.
                deadzone (int): The deadzone value.
                min (int): The minimum position value.
                max (int): The maximum position value.
        """
        data = self._read(address,self.Cmd.READM2POSPID,types=["long","long","long","long","long","long","long"])
        if data[0]:
            data[1]/=1024.0
            data[2]/=1024.0
            data[3]/=1024.0
            return data
        return (0,0,0,0,0,0,0,0)

    def SpeedAccelDeccelPositionM1(self, address, accel, speed, deccel, position, buffer):
        """
        Sets the acceleration, speed, deceleration, and position for motor 1.

        Args:
            address (int): The address of the controller.
            accel (int): The acceleration value to set.
            speed (int): The speed value to set.
            deccel (int): The deceleration value to set.
            position (int): The position value to set.
            buffer (int): The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, self.Cmd.M1SPEEDACCELDECCELPOS, accel, speed, deccel, position, buffer,types=["long","long","long","long","byte"])

    def SpeedAccelDeccelPositionM2(self, address, accel, speed, deccel, position, buffer):
        """
        Sets the acceleration, speed, deceleration, and position for motor 2.

        Args:
            address (int): The address of the controller.
            accel (int): The acceleration value to set.
            speed (int): The speed value to set.
            deccel (int): The deceleration value to set.
            position (int): The position value to set.
            buffer (int): The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, self.Cmd.M2SPEEDACCELDECCELPOS, accel, speed, deccel, position, buffer,types=["long","long","long","long","byte"])

    def SpeedAccelDeccelPositionM1M2(self, address, accel1, speed1, deccel1, position1, accel2, speed2, deccel2, position2, buffer):
        """
        Sets the acceleration, speed, deceleration, and position for both motors.

        Args:
            address (int): The address of the controller.
            accel1 (int): The acceleration value for motor 1.
            speed1 (int): The speed value for motor 1.
            deccel1 (int): The deceleration value for motor 1.
            position1 (int): The position value for motor 1.
            accel2 (int): The acceleration value for motor 2.
            speed2 (int): The speed value for motor 2.
            deccel2 (int): The deceleration value for motor 2.
            position2 (int): The position value for motor 2.
            buffer (int): The buffer value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, self.Cmd.MIXEDSPEEDACCELDECCELPOS, accel1, speed1, deccel1, position1, accel2, speed2, deccel2, position2, buffer,types=["long","long","long","long","long","long","long","long","byte"])

    def SetM1DefaultAccel(self, address, accel):
        """
        Sets the default acceleration for motor 1.

        Args:
            address (int): The address of the controller.
            accel (int): The default acceleration value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, self.Cmd.SETM1DEFAULTACCEL,accel,types=["long"])

    def SetM2DefaultAccel(self, address, accel):
        """
        Sets the default acceleration for motor 2.

        Args:
            address (int): The address of the controller.
            accel (int): The default acceleration value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, self.Cmd.SETM2DEFAULTACCEL,accel,types=["long"])

    def GetDefaultSpeeds(self, address):
        """
        Reads the default speeds for both motors.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, default_speed1, default_speed2)
                success (bool): True if read successful.
                default_speed1 (int): The default speed for motor 1.
                default_speed2 (int): The default speed for motor 2.
        """
        return self._read(address, self.Cmd.GETDEFAULTSPEEDS,types=["word","word"])

    def GetStatus(self,address):
        """
        Reads the status of the controller.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, tick, state, temp1, temp2, mbat, lbat, pwm1, pwm2, cur1, cur2, enc1, enc2, speedS1, speedS2, speed1, speed2, speederror1, speederror2, poserror1, poserror2)
                success (bool): True if read successful.
                tick (int): The tick value.
                state (int): The state value.
                temp1 (int): The temperature value 1.
                temp2 (int): The temperature value 2.
                mbat (int): The main battery voltage.
                lbat (int): The logic battery voltage.
                pwm1 (int): The PWM value for motor 1.
                pwm2 (int): The PWM value for motor 2.
                cur1 (int): The current value for motor 1.
                cur2 (int): The current value for motor 2.
                enc1 (int): The encoder value for motor 1.
                enc2 (int): The encoder value for motor 2.
                speedS1 (int): The speed setpoint for motor 1.
                speedS2 (int): The speed setpoint for motor 2.
                speed1 (int): The speed value for motor 1.
                speed2 (int): The speed value for motor 2.
                speederror1 (int): The speed error for motor 1.
                speederror2 (int): The speed error for motor 2.
                poserror1 (int): The position error for motor 1.
                poserror2 (int): The position error for motor 2.
        """
        return self._read(address,self.Cmd.GETSTATUS,types=["long","long","word","word","word","word","word","word","word","word","long","long","long","long","long","long","word","word","word","word"])

    def SetPinFunctions(self, address, S3mode, S4mode, S5mode):
        """
        Sets the functions of pins S3, S4, and S5.

        Args:
            address (int): The address of the controller.
            S3mode (int): The mode to set for pin S3.
            S4mode (int): The mode to set for pin S4.
            S5mode (int): The mode to set for pin S5.

        Returns:
            bool: True if successful.
        """
        return self._write(address, self.Cmd.SETPINFUNCTIONS, S3mode, S4mode, S5mode,types=["byte","byte","byte"])

    def ReadPinFunctions(self, address):
        """
        Reads the functions of pins S3, S4, and S5.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, S3mode, S4mode, S5mode)
                success (bool): True if read successful.
                S3mode (int): The mode of pin S3.
                S4mode (int): The mode of pin S4.
                S5mode (int): The mode of pin S5.
        """
        return self._read(address, self.Cmd.GETPINFUNCTIONS, types=["byte", "byte", "byte"])

    def SetDeadBand(self, address, min, max):
        """
        Sets the deadband values.

        Args:
            address (int): The address of the controller.
            min (int): The minimum deadband value.
            max (int): The maximum deadband value.

        Returns:
            bool: True if successful.
        """
        return self._write(address, self.Cmd.SETDEADBAND, min, max,types=["byte","byte"])

    def GetDeadBand(self, address):
        """
        Reads the deadband values.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, min, max)
                success (bool): True if read successful.
                min (int): The minimum deadband value.
                max (int): The maximum deadband value.
        """
        return self._read(address, self.Cmd.GETDEADBAND,types=["byte","byte"])

    def GetEncoders(self, address):
        """
        Reads the encoder values for both motors.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, enc1, enc2)
                success (bool): True if read successful.
                enc1 (int): The encoder value for motor 1.
                enc2 (int): The encoder value for motor 2.
        """
        return self._read(address, self.Cmd.GETENCODERS,types=["long","long"])

    def GetISpeeds(self, address):
        """
        Reads the instantaneous speeds for both motors.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, speed1, speed2)
                success (bool): True if read successful.
                speed1 (int): The instantaneous speed for motor 1.
                speed2 (int): The instantaneous speed for motor 2.
        """
        return self._read(address, self.Cmd.GETISPEEDS,types=["long","long"])
     
    def RestoreDefaults(self, address):
        """
        Restores the default settings.

        Args:
            address (int): The address of the controller.

        Returns:
            bool: True if successful.
        """
        return self._write(address, self.Cmd.RESTOREDEFAULTS,0xE22EAB7A,types=["long"])

    def GetDefaultAccels(self, address):
        """
        Reads the default accelerations for both motors.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, accel1, accel2, accel3, accel4)
                success (bool): True if read successful.
                accel1 (int): The default acceleration for motor 1.
                accel2 (int): The default acceleration for motor 2.
                accel3 (int): The default acceleration for motor 3.
                accel4 (int): The default acceleration for motor 4.
        """
        return self._read(address, self.Cmd.GETDEFAULTACCELS,types=["long","long","long","long"])

    def ReadTemp(self, address):
        """
        Reads the temperature from the first sensor.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, temperature)
                success (bool): True if read successful.
                temperature (int): The temperature value.
        """
        return self._read(address, self.Cmd.GETTEMP,types=["word"])

    def ReadTemp2(self, address):
        """
        Reads the temperature from the second sensor.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, temperature)
                success (bool): True if read successful.
                temperature (int): The temperature value.
        """
        return self._read(address, self.Cmd.GETTEMP2,types=["word"])

    def ReadError(self, address):
        """
        Reads the error status.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, error)
                success (bool): True if read successful.
                error (int): The error status.
        """
        return self._read(address, self.Cmd.GETERROR,types=["long"])

    def ReadEncoderModes(self, address):
        """
        Reads the encoder modes for both motors.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, mode1, mode2)
                success (bool): True if read successful.
                mode1 (int): The encoder mode for motor 1.
                mode2 (int): The encoder mode for motor 2.
        """
        return self._read(address, self.Cmd.GETENCODERMODE,types=["byte","byte"])

    def SetM1EncoderMode(self, address, mode):
        """
        Sets the encoder mode for motor 1.

        Args:
            address (int): The address of the controller.
            mode (int): The encoder mode to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, self.Cmd.SETM1ENCODERMODE, mode,types=["byte"])

    def SetM2EncoderMode(self, address, mode):
        """
        Sets the encoder mode for motor 2.

        Args:
            address (int): The address of the controller.
            mode (int): The encoder mode to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, self.Cmd.SETM2ENCODERMODE, mode,types=["byte"])

    def WriteNVM(self, address):
        """
        Saves the active settings to non-volatile memory (NVM).

        Args:
            address (int): The address of the controller.

        Returns:
            bool: True if successful.
        """
        return self._write(address, self.Cmd.WRITENVM, 0xE22EAB7A,types=["long"])

    def ReadNVM(self, address):
        """
        Restores the settings from non-volatile memory (NVM).

        Args:
            address (int): The address of the controller.

        Returns:
            bool: True if successful.
        """
        return self._write(address, self.Cmd.READNVM)

    def SetSerialNumber(self, address, serial_number):
        """Sets the device serial number (36 bytes).

        Args:
            address (int): controller address (0x80 to 0x87)
            serial_number (str): Serial number string (36 characters max)

        Returns:
            bool: True if successful, False otherwise

        Note: Serial number will be padded with nulls if less than 36 bytes
        """
        logger.info(f"Setting serial number for address 0x{address:02x} to '{serial_number}'")

        if not isinstance(serial_number, str):
            logger.error("Serial number must be a string")
            return False

        # Pad or truncate to exactly 36 bytes
        serial_bytes = serial_number.encode('ascii').ljust(36, b'\0')

        for _ in range(self._trystimeout):
            self._sendcommand(address, self.Cmd.SETSERIALNUMBER)

            try:
                # Write the count of characters and all 36 bytes in one go
                self._port.write(bytes([len(serial_number)]) + serial_bytes)
            except (serial.SerialException, ValueError) as e:
                logger.error(f"Exception during serial number setting: {str(e)}")

            if self._writechecksum():
                return True
    
        logger.error("Failed to set serial number after multiple attempts")
        return False

    def GetSerialNumber(self, address):
        """Reads the device serial number (36 bytes).

        Args:
            address (int): Controller address (0x80 to 0x87)

        Returns:
            tuple: (success, serial_number)
                success (bool): True if read successful
                serial_number (str): Serial number string
        """
        logger.info(f"Reading serial number for address 0x{address:02x}")

        for _ in range(self._trystimeout):
            try:
                self._port.flushInput()
            except Exception as e:
                logger.warning(f"Error flushing input buffer during serial number read: {str(e)}")
                # Continue trying despite the error
            self._sendcommand(address, self.Cmd.GETSERIALNUMBER)

            # Read 1 byte for the count of characters used by the serial number
            cnt = self._readbyte()
            if not cnt[0]:
                logger.debug("Failed to read count byte")
                continue

            # Read 36 bytes for the serial number
            serial_bytes = bytearray()
            for _ in range(36):
                val = self._readbyte()
                if not val[0]:
                    logger.debug("Failed to read serial byte")
                    break
                serial_bytes.append(val[1])

            if len(serial_bytes) == 36:
                crc = self._readchecksumword()
                if crc[0] and self._crc & 0xFFFF == crc[1] & 0xFFFF:
                    # Convert the count of characters into a string
                    serial_str = serial_bytes[:cnt[1]].decode('ascii').rstrip('\0')
                    return (1, serial_str)
                else:
                    logger.debug(f"CRC check failed: received=0x{crc[1]:04x}, calculated=0x{self._crc & 0xFFFF:04x}")
    
        logger.error(f"Failed to read serial number after {self._trystimeout} attempts")
        return (0, '')

    #Warning(TTL Serial): If control mode is changed from packet serial mode when setting config communications will be lost!
    #Warning(TTL Serial): If baudrate of packet serial mode is changed communications will be lost!
    def SetConfig(self, address, config):
        """
        Sets the configuration.

        Args:
            address (int): The address of the controller.
            config (int): The configuration value to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, self.Cmd.SETCONFIG, config,types=["word"])

    def GetConfig(self, address):
        """
        Reads the configuration.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, config)
                success (bool): True if read successful.
                config (int): The configuration value.
        """
        return self._read(address, self.Cmd.GETCONFIG,types=["word"])

    def GetEncStatus(self, address):
        """Reads encoder error statuses.
            
        Args:
            address (int): Controller address (0x80 to 0x87)
                
        Returns:
            tuple: (success, enc1status, enc2status)
                success (bool): True if read successful
                enc1status (int): Encoder 1 error flags
                enc2status (int): Encoder 2 error flags
        """
        return self._read(address, self.Cmd.GETENCSTATUS,types=["byte","byte"])

    def SetAuto1(self, address, value):
        """Sets auto mode 1 value.
            
        Args:
            address (int): Controller address (0x80 to 0x87)
            value (int): Auto mode configuration
                
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.SETAUTO1, value,types=["long"])

    def SetAuto2(self, address, value):
        """Sets auto mode 2 value.
            
        Args:
            address (int): Controller address (0x80 to 0x87)
            value (int): Auto mode configuration
                
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.SETAUTO2, value,types=["long"])

    def GetAutos(self, address):
        """Reads auto mode values.
            
        Args:
            address (int): Controller address (0x80 to 0x87)
                
        Returns:
            tuple: (success, auto1, auto2)
                success (bool): True if read successful
                auto1 (int): Auto mode 1 value
                auto2 (int): Auto mode 2 value
        """
        return self._read(address, self.Cmd.GETAUTOS,types=["long","long"])

    def GetSpeeds(self, address):
        """Reads current speed values for both motors.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
        
        Returns:
            tuple: (success, speed1, speed2)
                success (bool): True if read successful
                speed1 (int): Current speed of motor 1 (32-bit)
                speed2 (int): Current speed of motor 2 (32-bit)
        """
        return self._read(address, self.Cmd.GETSPEEDS,types=["long","long"])

    def SetSpeedErrorLimit(self, address, limit1, limit2):
        """Sets speed error limits.
            
        Args:
            address (int): Controller address (0x80 to 0x87)
            limit1 (int): Motor 1 speed error limit
            limit2 (int): Motor 2 speed error limit
                
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.SETSPEEDERRORLIMIT, limit1, limit2,types=["word","word"])

    def GetSpeedErrorLimit(self, address):
        """Reads speed error limits.
            
        Args:
            address (int): Controller address (0x80 to 0x87)
                
        Returns:
            tuple: (success, limit1, limit2)
        """
        return self._read(address, self.Cmd.GETSPEEDERRORLIMIT,types=["word","word"])

    def GetSpeedErrors(self, address):
        """Reads current speed errors.
            
        Args:
            address (int): Controller address (0x80 to 0x87)
                
        Returns:
            tuple: (success, error1, error2)
        """
        return self._read(address, self.Cmd.GETSPEEDERRORS,types=["word","word"])

    def M1Position(self, address, position, buffer):
        """Commands motor 1 to absolute position.
            
        Args:
            address (int): Controller address (0x80 to 0x87)
            position (int): Target position value
            buffer (int): The buffer value to set.
                
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.M1POS, position, buffer,types=["long","byte"])

    def M2Position(self, address, position, buffer):
        """Commands motor 2 to absolute position.
            
        Args:
            address (int): Controller address (0x80 to 0x87)
            position (int): Target position value
            buffer (int): The buffer value to set.
                
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.M2POS, position, buffer,types=["long","byte"])

    def MixedPosition(self, address, position1, position2, buffer):
        """Commands both motors to positions simultaneously.
            
        Args:
            address (int): Controller address (0x80 to 0x87)
            position1 (int): Motor 1 target position
            position2 (int): Motor 2 target position
            buffer (int): The buffer value to set.
                
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.MIXEDPOS, position1, position2, buffer,types=["long","long","byte"])

    def M1SpeedPosition(self, address, speed, position, buffer):
        """Commands motor 1 position with speed.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
            speed (int): Maximum speed
            position (int): Target position
            buffer (int): The buffer value to set.
        
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.M1SPEEDPOS, speed, position, buffer,types=["long","long","byte"])

    def M2SpeedPosition(self, address, speed, position, buffer):
        """Commands motor 2 position with speed.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
            speed (int): Maximum speed (32-bit)
            position (int): Target position (32-bit)
            buffer (int): The buffer value to set.
        
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.M2SPEEDPOS, speed, position, buffer,types=["long","long","byte"])

    def MixedSpeedPosition(self, address, speed1, position1, speed2, position2, buffer):
        """Commands both motors with speed and position.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
            speed1 (int): Motor 1 speed
            position1 (int): Motor 1 position
            speed2 (int): Motor 2 speed
            position2 (int): Motor 2 position
            buffer (int): The buffer value to set.
        
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.MIXEDSPEEDPOS, speed1, position1, speed2, position2, buffer,types=["long","long","long","long","byte"])

    def M1PercentPosition(self, address, position, buffer):
        """Commands motor 1 to a percent position.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
            position (int): Target position as percentage (-32768 to +32767)
            buffer (int): The buffer value to set.
        
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.M1PPOS, position, buffer,types=["sword","byte"])

    def M2PercentPosition(self, address, position, buffer):
        """Commands motor 2 to a percent position.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
            position (int): Target position as percentage (-32768 to +32767)
            buffer (int): The buffer value to set.
        
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.M2PPOS, position, buffer,types=["sword","byte"])

    def MixedPercentPosition(self, address, position1, position2, buffer):
        """Commands both motors to percent positions.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
            position1 (int): Motor 1 target position percentage (-32768 to +32767)
            position2 (int): Motor 2 target position percentage (-32768 to +32767)
            buffer (int): The buffer value to set.
        
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.MIXEDPPOS, position1, position2, buffer,types=["sword","sword","byte"])
    
    def SetPosErrorLimit(self, address, limit1, limit2):
        """Sets position error limits for both motors.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
            limit1 (int): Motor 1 position error limit (0 to 65535)
            limit2 (int): Motor 2 position error limit (0 to 65535)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.SETPOSERRORLIMIT, limit1, limit2,types=["word","word"])

    def GetPosErrorLimit(self, address):
        """Reads position error limits.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
        
        Returns:
            tuple: (success, limit1, limit2)
                success (bool): True if read successful
                limit1 (int): Motor 1 position error limit
                limit2 (int): Motor 2 position error limit
        """
        return self._read(address, self.Cmd.GETPOSERRORLIMIT,types=["word","word"])

    def GetPosErrors(self, address):
        """Reads current position errors.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
        
        Returns:
            tuple: (success, error1, error2)
                success (bool): True if read successful
                error1 (int): Motor 1 position error
                error2 (int): Motor 2 position error
        """
        return self._read(address, self.Cmd.GETPOSERRORS,types=["word","word"])

    def SetOffsets(self, address, offset1, offset2):
        """Sets encoder offsets.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
            offset1 (int): Motor 1 encoder offset (0 to 255)
            offset2 (int): Motor 2 encoder offset (0 to 255)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.SETOFFSETS, offset1, offset2,types=["byte","byte"])

    def GetOffsets(self, address):
        """Reads encoder offsets.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
        
        Returns:
            tuple: (success, offset1, offset2)
                success (bool): True if read successful
                offset1 (int): Motor 1 encoder offset
                offset2 (int): Motor 2 encoder offset
        """
        return self._read(address, self.Cmd.GETOFFSETS,types=["byte","byte"])

    def SetM1LR(self, address, L, R):
        """Sets motor 1 Inductance/Resistance.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
            L (int): Inductance in Henries (32-bit)
            R (int): Resistance in Ohms (32-bit)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.SETM1LR, int(L*0x1000000), int(R*0x1000000),types=["long","long"])

    def SetM2LR(self, address, L, R):
        """Sets motor 2 Inductance/Resistance.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
            L (int): Inductance in Henries (32-bit)
            R (int): Resistance in Ohms (32-bit)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.SETM2LR, int(L*0x1000000), int(R*0x1000000),types=["long","long"])

    def GetM1LR(self, address):
        """Reads motor 1 Inductance/Resistance.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
        
        Returns:
            tuple: (success, left_pos, right_pos)
                success (bool): True if read successful
                L (int): Inductance in Henries (32-bit)
                R (int): Resistance in Ohms (32-bit)
        """
        data = self._read(address, self.Cmd.GETM1LR,types=["long","long"])
        if data[0]:
            return (1, float(data[1])/0x1000000, float(data[2])/0x1000000)
        return (0, 0, 0)

    def GetM2LR(self, address):
        """Reads motor 2 Inductance/Resistance.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
        
        Returns:
            tuple: (success, left_pos, right_pos)
                success (bool): True if read successful
                L (int): Inductance in Henries (32-bit)
                R (int): Resistance in Ohms (32-bit)
        """
        data = self._read(address, self.Cmd.GETM2LR,types=["long","long"])
        if data[0]:
            return (1, float(data[1])/0x1000000, float(data[2])/0x1000000)
        return (0, 0, 0)

    def GetVolts(self, address):
        """Reads main and logic battery voltages.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
        
        Returns:
            tuple: (success, mbat, lbat)
                success (bool): True if read successful
                mbat (int): Main battery voltage in tenths of a volt
                lbat (int): Logic battery voltage in tenths of a volt
        """
        val = self._read(address, self.Cmd.GETVOLTS, types=["word", "word"])
        if val[0]:
            return (1, val[1], val[2])
        return (0, 0, 0)

    def GetTemps(self, address):
        """Reads temperature sensor values.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
        
        Returns:
            tuple: (success, temp1, temp2)
                success (bool): True if read successful
                temp1 (int): Temperature sensor 1 in tenth degrees Celsius
                temp2 (int): Temperature sensor 2 in tenth degrees Celsius
        """
        return self._read(address, self.Cmd.GETTEMPS, types=["word", "word"])

    def SetAuxDutys(self, address, duty1, duty2, duty3, duty4, duty5):
        """Sets auxiliary PWM duty cycles.
        
        Args:
            address (int): Controller address (0x80 to 0x87)
            duty1-duty5 (int): Duty cycle values (-32768 to +32767)
            
        Returns:
            bool: True if successful, False otherwise
        """
        return self._write(address, self.Cmd.SETAUXDUTYS, duty1, duty2, duty3, duty4, duty5,types=["word","word","word","word","word"])

    def GetAuxDutys(self, address):
        """Reads auxiliary PWM duty cycles.
        
        Args:
            address (int): Controller address (0x80 to 0x87)
            
        Returns:
            tuple: (success, duty1, duty2, duty3, duty4, duty5)
                success (bool): True if read successful
                duty1-5 (int): Current duty cycle values (-32768 to +32767)
        """
        return self._read(address, self.Cmd.GETAUXDUTYS,types=["word","word","word","word","word"])

    def SetM1MaxCurrent(self, address, maxi, mini):
        """
        Sets the maximum and minimum current limits for motor 1.

        Args:
            address (int): The address of the controller.
            maxi (int): The maximum current limit.
            mini (int): The minimum current limit.

        Returns:
            bool: True if successful.
        """
        return self._write(address, self.Cmd.SETM1MAXCURRENT, maxi, mini,types=["long","long"])

    def SetM2MaxCurrent(self, address, maxi, mini):
        """
        Sets the maximum and minimum current limits for motor 2.

        Args:
            address (int): The address of the controller.
            maxi (int): The maximum current limit.
            mini (int): The minimum current limit.

        Returns:
            bool: True if successful.
        """
        return self._write(address, self.Cmd.SETM2MAXCURRENT, maxi, mini,types=["long","long"])

    def ReadM1MaxCurrent(self, address):
        """
        Reads the maximum and minimum current limits for motor 1.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, maxi, mini)
                success (bool): True if read successful.
                maxi (int): The maximum current limit.
                mini (int): The minimum current limit.
        """
        return self._read(address, self.Cmd.GETM1MAXCURRENT,types=["long","long"])

    def ReadM2MaxCurrent(self, address):
        """
        Reads the maximum and minimum current limits for motor 2.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, maxi, mini)
                success (bool): True if read successful.
                maxi (int): The maximum current limit.
                mini (int): The minimum current limit.
        """
        return self._read(address, self.Cmd.GETM2MAXCURRENT,types=["long","long"])

    def SetDOUT(self, address, index, action):
        """Sets the digital output.
        
        Args:
            address (int): Controller address (0x80 to 0x87)
            index (int): Output index (0 to 255)
            action (int): Action to perform (0 to 255)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.SETDOUT, index, action,types=["byte","byte"])

    def GetDOUTS(self, address):
        """Gets the digital outputs.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
    
        Returns:
            tuple: (success, count, actions)
                success (bool): True if read successful
                count (int): Number of actions
                actions (list): List of actions performed
        """
        logger.info(f"Reading digital outputs for address 0x{address:02x}")

        for _ in range(self._trystimeout):
            try:
                self._port.flushInput()
            except Exception as e:
                logger.warning(f"Error flushing input buffer during DOUTS read: {str(e)}")
                # Continue trying despite the error
            self._sendcommand(address, self.Cmd.GETDOUTS)
        
            count = self._readbyte()
            if not count[0]:
                logger.debug("Failed to read count byte")
                continue
        
            actions = [self._readbyte()[1] for _ in range(count[1]) if self._readbyte()[0]]
        
            if len(actions) != count[1]:
                logger.debug("Failed to read all action bytes")
                continue
        
            crc = self._readchecksumword()
            if crc[0] and self._crc & 0xFFFF == crc[1] & 0xFFFF:
                return (1, count[1], actions)
    
        logger.error(f"Failed to read digital outputs after {self._trystimeout} attempts")
        return (0, 0, [])

    def SetPriority(self, address, priority1, priority2, priority3):
        """Sets the priority levels.
        
        Args:
            address (int): Controller address (0x80 to 0x87)
            priority1 (int): Priority level 1 (0 to 255)
            priority2 (int): Priority level 2 (0 to 255)
            priority3 (int): Priority level 3 (0 to 255)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.SETPRIORITY, priority1, priority2, priority3,types=["byte","byte","byte"])

    def GetPriority(self, address):
        """Gets the priority levels.
        
        Args:
            address (int): Controller address (0x80 to 0x87)
        
        Returns:
            tuple: (success, priority1, priority2, priority3)
                success (bool): True if read successful
                priority1 (int): Priority level 1
                priority2 (int): Priority level 2
                priority3 (int): Priority level 3
        """
        return self._read(address, self.Cmd.GETPRIORITY,types=["byte","byte","byte"])

    def SetAddressMixed(self, address, new_address, enable_mixing):
        """Sets the mixed address.
        
        Args:
            address (int): Controller address (0x80 to 0x87)
            new_address (int): New address (0 to 255)
            enable_mixing (int): Enable mixing (0 or 1)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.SETADDRESSMIXED, new_address, enable_mixing,types=["byte","byte"])

    def GetAddressMixed(self, address):
        """Gets the mixed address.
        
        Args:
            address (int): Controller address (0x80 to 0x87)
        
        Returns:
            tuple: (success, new_address, mixed)
                success (bool): True if read successful
                new_address (int): New address
                mixed (int): Mixing enabled (0 or 1)
        """
        return self._read(address, self.Cmd.GETADDRESSMIXED,types=["byte","byte"])

    def SetSignal(self, address, index, signal_type, mode, target, min_action, max_action, lowpass, timeout, loadhome, min_val, max_val, center, deadband, powerexp, minout, maxout, powermin, potentiometer):
        """Sets the signal parameters.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
            index (int): Signal index (0 to 255)
            signal_type (int): Signal type (0 to 255)
            mode (int): Mode (0 to 255)
            target (int): Target (0 to 255)
            min_action (int): Minimum action (0 to 65535)
            max_action (int): Maximum action (0 to 65535)
            lowpass (int): Lowpass filter (0 to 255)
            timeout (int): Timeout (0 to 4294967295)
            loadhome (int): Load home position (-2147483648 to 2147483647)
            min_val (int): Minimum value (-2147483648 to 2147483647)
            max_val (int): Maximum value (-2147483648 to 2147483647)
            center (int): Center value (-2147483648 to 2147483647)
            deadband (int): Deadband (0 to 4294967295)
            powerexp (int): Power exponent (0 to 4294967295)
            minout (int): Minimum output (0 to 4294967295)
            maxout (int): Maximum output (0 to 4294967295)
            powermin (int): Minimum power (0 to 4294967295)
            potentiometer (int): Potentiometer value (0 to 4294967295)
    
        Returns:
            bool: True if successful
        """
        logger.info(f"Setting signal parameters for address 0x{address:02x}, index={index}")
        logger.debug(f"Signal params: type={signal_type}, mode={mode}, target={target}, min_action={min_action}, max_action={max_action}, lowpass={lowpass}")
    
        params = [
            (index, "byte"), (signal_type, "byte"), (mode, "byte"), (target, "byte"),
            (min_action, "word"), (max_action, "word"), (lowpass, "byte"), (timeout, "long"),
            (loadhome, "slong"), (min_val, "slong"), (max_val, "slong"), (center, "slong"),
            (deadband, "long"), (powerexp, "long"), (minout, "long"), (maxout, "long"),
            (powermin, "long"), (potentiometer, "long")
        ]

        for _ in range(self._trystimeout):
            self._sendcommand(address, self.Cmd.SETSIGNAL)
            for value, dtype in params:
                if dtype == "byte":
                    self._writebyte(value)
                elif dtype == "word":
                    self._writeword(value)
                elif dtype == "long":
                    self._writelong(value)
                elif dtype == "slong":
                    self._writeslong(value)
        
            if self._writechecksum():
                return True
    
        logger.error(f"Failed to set signal parameters for address 0x{address:02x}, index={index} after {self._trystimeout} attempts")
        return False

    def GetSignals(self, address):
        """Gets the signal parameters.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
    
        Returns:
            tuple: (success, count, signals)
                success (bool): True if read successful
                count (int): Number of signals
                signals (list): List of signal parameters
        """
        logger.info(f"Reading signal parameters for address 0x{address:02x}")

        for _ in range(self._trystimeout):
            try:
                self._port.flushInput()
            except Exception as e:
                logger.warning(f"Error flushing input buffer during signals read: {str(e)}")
                # Continue trying despite the error
            self._sendcommand(address, self.Cmd.GETSIGNALS)
            count = self._readbyte()
            if not count[0]:
                logger.debug("Failed to read signal count byte")
                continue

            signals = []
            read_error = False

            for _ in range(count[1]):
                signal = {
                    'type': self._readbyte(),
                    'mode': self._readbyte(),
                    'target': self._readbyte(),
                    'min_action': self._readword(),
                    'max_action': self._readword(),
                    'lowpass': self._readbyte(),
                    'timeout': self._readlong(),
                    'loadhome': self._readslong(),
                    'min_val': self._readslong(),
                    'max_val': self._readslong(),
                    'center': self._readslong(),
                    'deadband': self._readlong(),
                    'powerexp': self._readlong(),
                    'minout': self._readlong(),
                    'maxout': self._readlong(),
                    'powermin': self._readlong(),
                    'potentiometer': self._readlong()
                }

                if not all([v[0] for v in signal.values()]):
                    logger.debug(f"Failed to read complete signal data for signal {i+1}")
                    read_error = True
                    break

                signals.append({k: v[1] for k, v in signal.items()})

            if read_error:
                continue

            crc = self._readchecksumword()
            if crc[0] and self._crc & 0xFFFF == crc[1] & 0xFFFF:
                return (1, count[1], signals)
            else:
                logger.debug(f"CRC check failed: received=0x{crc[1]:04x}, calculated=0x{self._crc & 0xFFFF:04x}")

        logger.error(f"Failed to read signals from address 0x{address:02x} after {self._trystimeout} attempts")
        return (0, 0, [])

    def SetStream(self, address, index, stream_type, baudrate, timeout):
        """Sets the stream parameters.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
            index (int): Stream index (0 to 255)
            stream_type (int): Stream type (0 to 255)
            baudrate (int): Baudrate (0 to 4294967295)
            timeout (int): Timeout (0 to 4294967295)
    
        Returns:
            bool: True if successful
        """
        logger.info(f"Setting stream parameters for address 0x{address:02x}, index={index}")
        logger.debug(f"Stream params: type={stream_type}, baudrate={baudrate}, timeout={timeout}")

        for _ in range(self._trystimeout):
            self._sendcommand(address, self.Cmd.SETSTREAM)
            self._writebyte(index)
            self._writebyte(stream_type)
            self._writelong(baudrate)
            self._writelong(timeout)
            if self._writechecksum():
                return True

        logger.error(f"Failed to set stream parameters for address 0x{address:02x}, index={index} after {self._trystimeout} attempts")
        return False

    def GetStreams(self, address):
        """Gets the stream parameters.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
    
        Returns:
            tuple: (success, count, streams)
                success (bool): True if read successful
                count (int): Number of streams
                streams (list): List of stream parameters
        """
        logger.info(f"Reading stream parameters for address 0x{address:02x}")

        for _ in range(self._trystimeout):
            try:
                self._port.flushInput()
            except Exception as e:
                logger.warning(f"Error flushing input buffer during streams read: {str(e)}")
                # Continue trying despite the error
            self._sendcommand(address, self.Cmd.GETSTREAMS)
            count = self._readbyte()
            if not count[0]:
                logger.debug("Failed to read stream count byte")
                continue

            streams = []
            read_error = False

            for _ in range(count[1]):
                stream = {
                    'type': self._readbyte(),
                    'baudrate': self._readlong(),
                    'timeout': self._readlong()
                }

                if not all([v[0] for v in stream.values()]):
                    logger.debug(f"Failed to read complete stream data for stream {i+1}")
                    read_error = True
                    break

                streams.append({k: v[1] for k, v in stream.items()})

            if read_error:
                continue

            crc = self._readchecksumword()
            if crc[0] and self._crc & 0xFFFF == crc[1] & 0xFFFF:
                return (1, count[1], streams)
            else:
                logger.debug(f"CRC check failed: received=0x{crc[1]:04x}, calculated=0x{self._crc & 0xFFFF:04x}")

        logger.error(f"Failed to read streams from address 0x{address:02x} after {self._trystimeout} attempts")
        return (0, 0, [])

    def GetSignalsData(self, address):
        """Gets the signals data.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
    
        Returns:
            tuple: (success, count, signals_data)
                success (bool): True if read successful
                count (int): Number of signals data
                signals_data (list): List of signals data
        """
        logger.info(f"Reading signals data from address 0x{address:02x}")
    
        for _ in range(self._trystimeout):
            try:
                self._port.flushInput()
            except Exception as e:
                logger.warning(f"Error flushing input buffer during signals data read: {str(e)}")
                # Continue trying despite the error
            self._sendcommand(address, self.Cmd.GETSIGNALSDATA)
            count = self._readbyte()
            if not count[0]:
                logger.debug("Failed to read signal data count byte")
                continue

            signals_data = []
            read_error = False
        
            for _ in range(count[1]):
                signal_data = {}
                signal_data['command'] = self._readlong()
                signal_data['position'] = self._readlong()
                signal_data['percent'] = self._readlong()
                signal_data['speed'] = self._readlong()
                signal_data['speeds'] = self._readlong()

                if not all([v[0] for v in signal_data.values()]):
                    logger.debug(f"Failed to read complete data for signal {i+1}")
                    read_error = True
                    break

                signals_data.append({k: v[1] for k, v in signal_data.items()})

            if read_error:
                continue

            crc = self._readchecksumword()
            if crc[0] and self._crc & 0xFFFF == crc[1] & 0xFFFF:
                return (1, count[1], signals_data)
            else:
                logger.debug(f"CRC check failed: received=0x{crc[1]:04x}, calculated=0x{self._crc & 0xFFFF:04x}")

        logger.error(f"Failed to read signals data from address 0x{address:02x} after {self._trystimeout} attempts")
        return (0, 0, [])
    
    def SetNodeID(self, address, nodeid):
        """Sets the node ID.
        
        Args:
            address (int): Controller address (0x80 to 0x87)
            nodeid (int): Node ID (0 to 255)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.SETNODEID, nodeid,types=["byte"])

    def GetNodeID(self, address):
        """Gets the node ID.
        
        Args:
            address (int): Controller address (0x80 to 0x87)
        
        Returns:
            tuple: (success, nodeid)
                success (bool): True if read successful
                nodeid (int): Node ID
        """
        return self._read(address, self.Cmd.GETNODEID,types=["byte"])

    def SetPWMIdle(self, address, idledelay1, idlemode1, idledelay2, idlemode2):
        """Sets the PWM idle parameters.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
            idledelay1 (int): Idle delay 1 (0 to 127)
            idlemode1 (int): Idle mode 1
            idledelay2 (int): Idle delay 2 (0 to 127)
            idlemode2 (int): Idle mode 2
    
        Returns:
            bool: True if successful
        """
        byte1 = (int(idledelay1 * 10) & 0x7F) | (0x80 if idlemode1 else 0x00)
        byte2 = (int(idledelay2 * 10) & 0x7F) | (0x80 if idlemode2 else 0x00)   
        return self._write(address, self.Cmd.SETPWMIDLE, byte1, byte2, types=["byte", "byte"])
        
    def GetPWMIdle(self, address):
        """Gets the PWM idle parameters.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
    
        Returns:
            tuple: (success, idledelay1, idlemode1, idledelay2, idlemode2)
                success (bool): True if read successful
                idledelay1 (int): Idle delay 1
                idlemode1 (int): Idle mode 1
                idledelay2 (int): Idle delay 2
                idlemode2 (int): Idle mode 2
        """
        result = self._read(address, self.Cmd.GETPWMIDLE, types=["byte", "byte"])
    
        if result[0]:
            val1 = result[1]
            val2 = result[2]
            idledelay1 = float(val1 & 0x7F) / 10
            idlemode1 = bool(val1 & 0x80)
            idledelay2 = float(val2 & 0x7F) / 10
            idlemode2 = bool(val2 & 0x80)
            return (1, idledelay1, idlemode1, idledelay2, idlemode2)    
        return (0, 0, False, 0, False)

    def CANBufferState(self, address):
        """Gets the count of available CAN packets.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
    
        Returns:
            tuple: (success, count)
                success (bool): True if read successful
                count (int): Number of available CAN packets
        """
        return self._read(address, self.Cmd.CANBUFFERSTATE, types=["byte"])

    def CANPutPacket(self, address, cob_id, RTR, data):
        """Sends a CAN packet.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
            cob_id (int): CAN object identifier (0 to 255)
            RTR (int): Remote Transmission Request (0 or 1)
            data (list): List of data bytes (length must be <= 8 bytes)
    
        Returns:
            bool: True if successful
        """
        length = len(data)
        if length > 8:
            raise ValueError("Data length must be no more than 8")

        # Pad data to 8 bytes with 0s
        padded_data = data.copy()
        padded_data.extend([0] * (8 - length))
    
        # First the command-specific arguments (cob_id, RTR, length)
        # Then all 8 bytes of data (padded with zeros if needed)
        return self._write(address, self.Cmd.CANPUTPACKET, cob_id, RTR, length, padded_data[0], padded_data[1], 
                           padded_data[2], padded_data[3], padded_data[4], padded_data[5], padded_data[6], padded_data[7],
                           types=["word", "byte", "byte", "byte", "byte", "byte", "byte", "byte", "byte", "byte", "byte"]
                          )

    def CANGetPacket(self, address):
        """Reads a CAN packet.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
    
        Returns:
            tuple: (success, cob_id, RTR, length, data)
                success (bool): True if read successful
                cob_id (int): CAN object identifier
                RTR (int): Remote Transmission Request
                length (int): Length of the data
                data (list): List of data bytes
        """
        val = self._read(address, self.Cmd.CANGETPACKET, types=["byte", "word", "byte", "byte", "byte", "byte", "byte", "byte", "byte", "byte", "byte", "byte"])
        if val[0] and val[1] == 0xFF:  # First byte should be 0xFF for a valid packet
            cob_id = val[2]
            RTR = val[3]
            length = val[4]
            # Extract data bytes(all 8)
            data = [val[i+5] for i in range(8)]
            return (1, cob_id, RTR, length, data)
    
        return (0, 0, 0, 0, [])

    def CANOpenWriteLocalDict(self, address, wIndex, bSubindex, lValue, bSize):
        """Writes to the local CANopen dictionary.
    
        Args:
            address (int): Controller address (0x80 to 0x87)
            wIndex (int): Index in the dictionary
            bSubindex (int): Subindex in the dictionary
            lValue (int): Value to write
            bSize (int): Size of the value
    
        Returns:
            lResult (int): Result of the write operation
        """
        logger.debug(f"Writing to CANopen dictionary at address=0x{address:02x}, index=0x{wIndex:04x}, subindex=0x{bSubindex:02x}, value={lValue}, size={bSize}")

        for _ in range(self._trystimeout):
            self._sendcommand(address, self.Cmd.CANOPENWRITELOCALDICT)
            self._writeword(wIndex)
            self._writebyte(bSubindex)
            self._writelong(lValue)
            self._writebyte(bSize)
        
            if self._writechecksum():
                self.crc_clear()
                lResult = self._readlong()
                if lResult[0]:
                    crc = self._readchecksumword()
                    if crc[0] and self._crc & 0xFFFF == crc[1] & 0xFFFF:
                        return (1, lResult[1])
                    else:
                        logger.debug(f"CRC check failed: received=0x{crc[1]:04x}, calculated=0x{self._crc & 0xFFFF:04x}")
                else:
                    logger.debug("Failed to read result value")
            else:
                logger.debug("Failed to write checksum")

        logger.error(f"Failed to write to CANopen dictionary at address=0x{address:02x}, index=0x{wIndex:04x}, subindex=0x{bSubindex:02x} after {self._trystimeout} attempts")
        return False

    def CANOpenReadLocalDict(self, address, wIndex, bSubindex):
        """Reads from the local CANopen dictionary.
        
        Args:
            address (int): Controller address (0x80 to 0x87)
            wIndex (int): Index in the dictionary
            bSubindex (int): Subindex in the dictionary
        
        Returns:
            tuple: (success, lValue, bSize, bType, lResult)
                success (bool): True if read successful
                lValue (int): Value read
                bSize (int): Size of the value
                bType (int): Type of the value
                lResult (int): Result of the read operation
        """
        logger.debug(f"Reading from CANopen dictionary at address=0x{address:02x}, index=0x{wIndex:04x}, subindex=0x{bSubindex:02x}")

        for _ in range(self._trystimeout):
            self._sendcommand(address, self.Cmd.CANOPENREADLOCALDICT)
            self._writeword(wIndex)
            self._writebyte(bSubindex)
        
            lValue = self._readlong()
            if not lValue[0]:
                logger.debug("Failed to read lValue")
                continue

            bSize = self._readbyte()
            if not bSize[0]:
                logger.debug("Failed to read bSize")
                continue

            bType = self._readbyte()
            if not bType[0]:
                logger.debug("Failed to read bType")
                continue

            lResult = self._readlong()
            if not lResult[0]:
                logger.debug("Failed to read lResult")
                continue
                
            crc = self._readchecksumword()
            if crc[0] and self._crc & 0xFFFF == crc[1] & 0xFFFF:
                return (1, lValue[1], bSize[1], bType[1], lResult[1])
    
            else:
                logger.debug(f"CRC check failed: received=0x{crc[1]:04x}, calculated=0x{self._crc & 0xFFFF:04x}")
    
        logger.error(f"Failed to read from CANopen dictionary after {self._trystimeout} attempts: address=0x{address:02x}, index=0x{wIndex:04x}, subindex=0x{bSubindex:02x}")
        return (0, 0, 0, 0, 0)

    def ResetEStop(self, address):
        """Resets the emergency stop.
        
        Args:
            address (int): Controller address (0x80 to 0x87)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.RESETESTOP)

    def SetEStopLock(self, address, state):
        """Sets the emergency stop lock state.
        
        Args:
            address (int): Controller address (0x80 to 0x87)
            state (int): State value (0x55 for automatic reset, 0xAA for software reset, 0 for hardware reset)
        
        Returns:
            bool: True if successful
        """
        if state not in [0x55, 0xAA, 0]:
            raise ValueError("Invalid state value. Must be 0x55, 0xAA, or 0.")
        
        return self._write(address, self.Cmd.SETESTOPLOCK, state,types=["byte"])

    def GetEStopLock(self, address):
        """Gets the emergency stop lock state.
        
        Args:
            address (int): Controller address (0x80 to 0x87)
        
        Returns:
            tuple: (success, state)
                success (bool): True if read successful
                state (int): State value (0x55 for automatic reset, 0xAA for software reset, 0 for hardware reset)
        """
        return self._read(address, self.Cmd.GETESTOPLOCK,types=["byte"])

    def SetScriptAutoRun(self, address, scriptauto_time):
        """Sets the script auto run time.
        
        Args:
            address (int): Controller address (0x80 to 0x87)
            scriptauto_time (int): Auto run time in milliseconds. Values less than 100 will be set to 0 (script does not autorun).
        
        Returns:
            bool: True if successful
        """
        if scriptauto_time < 100 and scriptauto_time != 0:
            raise ValueError("Scriptauto_time value is below 100! Script will not autorun.")
        
        return self._write(address, self.Cmd.SETSCRIPTAUTORUN, scriptauto_time,types=["long"])

    def GetScriptAutoRun(self, address):
        """Gets the script auto run time.
        
        Args:
            address (int): Controller address (0x80 to 0x87)
        
        Returns:
            tuple: (success, scriptauto_time)
                success (bool): True if read successful
                scriptauto_time (int): Auto run time in milliseconds
        """
        return self._read(address, self.Cmd.GETSCRIPTAUTORUN,types=["long"])

    def StartScript(self, address):
        """Starts the script.
        
        Args:
            address (int): Controller address (0x80 to 0x87)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.STARTSCRIPT)

    def StopScript(self, address):
        """Stops the script.
        
        Args:
            address (int): Controller address (0x80 to 0x87)
        
        Returns:
            bool: True if successful
        """
        return self._write(address, self.Cmd.STOPSCRIPT)

    def SetPWMMode(self, address, mode):
        """
        Sets the PWM mode.

        Args:
            address (int): The address of the controller.
            mode (int): The PWM mode to set.

        Returns:
            bool: True if successful.
        """
        return self._write(address, self.Cmd.SETPWMMODE, mode,types=["byte"])

    def ReadPWMMode(self, address):
        """
        Reads the PWM mode.

        Args:
            address (int): The address of the controller.

        Returns:
            tuple: (success, mode)
                success (bool): True if read successful.
                mode (int): The PWM mode.
        """
        return self._read(address, self.Cmd.GETPWMMODE,types=["byte"])

    def ReadEeprom(self, address, ee_address):
        """
        Reads a word from the EEPROM.

        Args:
            address (int): The address of the controller.
            ee_address (int): The EEPROM address to read from.

        Returns:
            tuple: (success, value)
                success (bool): True if read successful.
                value (int): The word value read from EEPROM.
        """
        # Define error codes as constants for better readability
        CRC_MISMATCH = -0x20000
        TIMEOUT_ERROR = -0x10000
    
        logger.info(f"Reading EEPROM from controller address=0x{address:02x}, EEPROM address=0x{ee_address:02x}")
    
        for _ in range(self._trystimeout):
            try:
                self._port.flushInput()
            except Exception as e:
                logger.warning(f"Error flushing input buffer during EEPROM read: {str(e)}")
                # Continue trying despite the error
            self._sendcommand(address, self.Cmd.READEEPROM)
            self.crc_update(ee_address)

            try:
                self._port.write(bytes([ee_address & 0xFF]))
            except (serial.SerialException, ValueError) as e:
                logger.error(f"Serial communication error while writing EEPROM address: {str(e)}")
                return (self.FAILURE, WRITE_ERROR)
            except Exception as e:
                logger.error(f"Unexpected error during EEPROM address write: {str(e)}")
                return (self.FAILURE, WRITE_ERROR)
        
            # Read response
            val = self._readword()
            if not val[0]:
                logger.debug("Failed to read value from EEPROM")
                continue
            
            # Validate checksum
            crc = self._readchecksumword()
            if not crc[0]:
                logger.debug("Failed to read CRC checksum")
                continue
            
            # Check if CRC matches
            if self._crc & 0xFFFF == crc[1] & 0xFFFF:
                return (self.SUCCESS, val[1])
            else:
                logger.debug(f"CRC mismatch: calculated=0x{self._crc & 0xFFFF:04x}, received=0x{crc[1]:04x}")
                return (self.FAILURE, CRC_MISMATCH)
    
        logger.error(f"Failed to read from EEPROM after {self._trystimeout} attempts: controller address=0x{address:02x}, EEPROM address=0x{ee_address:02x}")
        return (self.FAILURE, TIMEOUT_ERROR)

    def WriteEeprom(self, address, ee_address, ee_word):
        """
        Writes a word to the EEPROM.

        Args:
            address (int): The address of the controller.
            ee_address (int): The EEPROM address to write to.
            ee_word (int): The word value to write to EEPROM.

        Returns:
            bool: True if successful.
        """
        return self._write(address, self.Cmd.WRITEEEPROM, ee_address, ee_word >> 8, ee_word & 0xFF,types=["byte","byte","byte"])

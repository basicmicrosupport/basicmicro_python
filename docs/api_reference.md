# Basicmicro API Reference

This document provides detailed information about the Basicmicro library API for controlling Basicmicro motor controllers.

## Table of Contents

- [Main Controller Class](#main-controller-class)
- [Connection Management](#connection-management)
- [Basic Motor Control](#basic-motor-control)
- [Duty Cycle Control](#duty-cycle-control)
- [Speed Control](#speed-control)
- [Position Control](#position-control)
- [PID Configuration](#pid-configuration)
- [Encoder Functions](#encoder-functions)
- [Status and Diagnostic Functions](#status-and-diagnostic-functions)
- [Configuration Functions](#configuration-functions)
- [CAN Bus Functions](#can-bus-functions)
- [Advanced Functions](#advanced-functions)
- [PWM Functions](#pwm-functions)
- [Digital I/O Functions](#digital-io-functions)
- [Signaling and Stream Functions](#signaling-and-stream-functions)
- [Error Handling](#error-handling)

## Main Controller Class

### `Basicmicro`

The main interface class for controlling Basicmicro motor controllers.

```python
controller = Basicmicro(comport, rate, timeout=0.01, retries=2, verbose=False)
```

Parameters:
- `comport` (str): The COM port to use (e.g., 'COM3', '/dev/ttyACM0')
- `rate` (int): The baud rate for the serial communication
- `timeout` (float, optional): The timeout for serial communication in seconds. Default is 0.01.
- `retries` (int, optional): The number of retries for communication. Default is 2.
- `verbose` (bool, optional): Enable detailed debug logging. Default is False.

## Connection Management

### `Open()`

Opens and configures the serial connection to the controller.

```python
success = controller.Open()
```

Returns:
- `bool`: True if connection successful, False otherwise

### `close()`

Closes the serial connection to the controller.

```python
controller.close()
```

### Context Manager Support

The `Basicmicro` class supports the Python context manager pattern:

```python
with Basicmicro("/dev/ttyACM0", 38400) as controller:
    # Work with controller
    # Connection is automatically closed when leaving the block
```

## Basic Motor Control

### Legacy-Style Commands (0-127 values)

These commands use a 0-127 value range for compatibility with older versions:

- `ForwardM1(address, val)`: Motor 1 forward
- `BackwardM1(address, val)`: Motor 1 backward
- `ForwardM2(address, val)`: Motor 2 forward
- `BackwardM2(address, val)`: Motor 2 backward
- `ForwardBackwardM1(address, val)`: Motor 1 forward/backward (7-bit)
- `ForwardBackwardM2(address, val)`: Motor 2 forward/backward (7-bit)

For mixed mode (differential drive):
- `ForwardMixed(address, val)`: Move forward
- `BackwardMixed(address, val)`: Move backward
- `TurnRightMixed(address, val)`: Turn right
- `TurnLeftMixed(address, val)`: Turn left
- `ForwardBackwardMixed(address, val)`: Move forward/backward (7-bit)
- `LeftRightMixed(address, val)`: Turn left/right (7-bit)

Parameters:
- `address` (int): Controller address (0x80 to 0x87)
- `val` (int): Power value (0-127)

Returns:
- `bool`: True if successful

## Duty Cycle Control

Modern control using 16-bit duty cycle values (-32767 to +32767):

### `DutyM1(address, val)`

Sets the duty cycle for motor 1 (-32767 to +32767).

```python
success = controller.DutyM1(address, 16384)  # 50% forward
```

### `DutyM2(address, val)`

Sets the duty cycle for motor 2 (-32767 to +32767).

```python
success = controller.DutyM2(address, -8192)  # 25% backward
```

### `DutyM1M2(address, m1, m2)`

Sets the duty cycle for both motors simultaneously.

```python
success = controller.DutyM1M2(address, 16384, -8192)  # M1 forward, M2 backward
```

### `DutyAccelM1(address, accel, duty)`

Sets acceleration and duty cycle for motor 1.

```python
success = controller.DutyAccelM1(address, 500, 16384)  # Accelerate to 50%
```

### `DutyAccelM2(address, accel, duty)`

Sets acceleration and duty cycle for motor 2.

### `DutyAccelM1M2(address, accel1, duty1, accel2, duty2)`

Sets acceleration and duty cycle for both motors.

## Speed Control

Commands for velocity control (requires encoders):

### `SpeedM1(address, val)`

Sets the speed for motor 1 in encoder counts per second.

```python
success = controller.SpeedM1(address, 1000)  # 1000 counts/sec forward
```

### `SpeedM2(address, val)`

Sets the speed for motor 2 in encoder counts per second.

### `SpeedM1M2(address, m1, m2)`

Sets the speed for both motors.

```python
success = controller.SpeedM1M2(address, 1000, -800)  # Different speeds
```

### With Acceleration Control

- `SpeedAccelM1(address, accel, speed)`
- `SpeedAccelM2(address, accel, speed)`
- `SpeedAccelM1M2(address, accel, speed1, speed2)`
- `SpeedAccelM1M2_2(address, accel1, speed1, accel2, speed2)` - Different accelerations

Example:
```python
# Accelerate Motor 1 to 1000 counts/sec at rate of 500 counts/sec²
success = controller.SpeedAccelM1(address, 500, 1000)
```

### With Distance Control

- `SpeedDistanceM1(address, speed, distance, buffer)`
- `SpeedDistanceM2(address, speed, distance, buffer)`
- `SpeedDistanceM1M2(address, speed1, distance1, speed2, distance2, buffer)`

Example:
```python
# Move Motor 1 for 1000 counts at 500 counts/sec, immediate execution
success = controller.SpeedDistanceM1(address, 500, 1000, 0)
```

### With Acceleration and Distance

- `SpeedAccelDistanceM1(address, accel, speed, distance, buffer)`
- `SpeedAccelDistanceM2(address, accel, speed, distance, buffer)`
- `SpeedAccelDistanceM1M2(address, accel, speed1, distance1, speed2, distance2, buffer)`
- `SpeedAccelDistanceM1M2_2(address, accel1, speed1, distance1, accel2, speed2, distance2, buffer)` - Different accelerations

## Position Control

Commands for position control (requires encoders):

### `M1Position(address, position, buffer)`

Commands motor 1 to move to absolute position.

```python
# Move Motor 1 to position 5000, immediate execution
success = controller.M1Position(address, 5000, 0)
```

### `M2Position(address, position, buffer)`

Commands motor 2 to move to absolute position.

### `MixedPosition(address, position1, position2, buffer)`

Commands both motors to specific positions.

```python
# Move both motors to positions, immediate execution
success = controller.MixedPosition(address, 5000, 3000, 0)
```

### Speed-Controlled Position Commands

- `M1SpeedPosition(address, speed, position, buffer)`
- `M2SpeedPosition(address, speed, position, buffer)`
- `MixedSpeedPosition(address, speed1, position1, speed2, position2, buffer)`

Example:
```python
# Move M1 to position 5000 at speed 1000, immediate execution
success = controller.M1SpeedPosition(address, 1000, 5000, 0)
```

### Speed-Accel-Decel-Controlled Position Comamnds

- `SpeedAccelDeccelPositionM1(address, accel, speed, deccel, position, buffer)`
- `SpeedAccelDeccelPositionM2(address, accel, speed, deccel, position, buffer)`
- `SpeedAccelDeccelPositionM1M2(address, accel1, speed1, deccel1, position1, accel2, speed2, deccel2, position2, buffer)`

Example:
```python
# Move M1 to position 5000 at speed 1000, immediate execution
success = controller.SpeedAccelDecelPositionM1(address, 1000, 1000, 1000, 5000, 0)
```

### Percentage Position Commands

- `M1PercentPosition(address, position, buffer)`
- `M2PercentPosition(address, position, buffer)`
- `MixedPercentPosition(address, position1, position2, buffer)`

Example:
```python
# Move Motor 1 to 50% of its range, immediate execution
success = controller.M1PercentPosition(address, 16384, 0)  # 50% = 16384
```

## PID Configuration

### Velocity PID

- `SetM1VelocityPID(address, p, i, d, qpps)`
- `SetM2VelocityPID(address, p, i, d, qpps)`
- `ReadM1VelocityPID(address)`
- `ReadM2VelocityPID(address)`

Example:
```python
# Set velocity PID for motor 1
success = controller.SetM1VelocityPID(address, 1.0, 0.5, 0.25, 44000)

# Read back the settings
success, p, i, d, qpps = controller.ReadM1VelocityPID(address)
```

### Position PID

- `SetM1PositionPID(address, kp, ki, kd, kimax, deadzone, min_pos, max_pos)`
- `SetM2PositionPID(address, kp, ki, kd, kimax, deadzone, min_pos, max_pos)`
- `ReadM1PositionPID(address)`
- `ReadM2PositionPID(address)`

Example:
```python
# Set position PID for motor 1
success = controller.SetM1PositionPID(address, 10.0, 0.5, 1.0, 50, 10, -1000000, 1000000)
```

### Motor Electrical Properties

- `SetM1LR(address, L, R)` - Set motor 1 inductance/resistance
- `SetM2LR(address, L, R)` - Set motor 2 inductance/resistance
- `GetM1LR(address)` - Get motor 1 inductance/resistance
- `GetM2LR(address)` - Get motor 2 inductance/resistance

Example:
```python
# Set motor 1 L/R values
success = controller.SetM1LR(address, 0.001, 0.5)  # 1mH, 0.5Ω

# Read motor 1 L/R values
success, L, R = controller.GetM1LR(address)
```

## Encoder Functions

### Reading Encoders

- `ReadEncM1(address)`: Read motor 1 encoder
- `ReadEncM2(address)`: Read motor 2 encoder
- `GetEncoders(address)`: Read both encoders
- `ReadISpeedM1(address)`: Read instantaneous speed for motor 1
- `ReadISpeedM2(address)`: Read instantaneous speed for motor 2
- `GetISpeeds(address)`: Read instantaneous speeds for both motors
- `ReadSpeedM1(address)`: Read filtered speed for motor 1
- `ReadSpeedM2(address)`: Read filtered speed for motor 2
- `GetSpeeds(address)`: Read speeds for both motors
- `GetEncStatus(address)`: Read encoder error statuses

Example:
```python
# Read Motor 1 encoder
success, count, status = controller.ReadEncM1(address)

# Read both encoders
success, enc1, enc2 = controller.GetEncoders(address)
```

### Encoder Management

- `ResetEncoders(address)`: Reset both encoders to zero
- `SetEncM1(address, cnt)`: Set motor 1 encoder value
- `SetEncM2(address, cnt)`: Set motor 2 encoder value
- `SetM1EncoderMode(address, mode)`: Set motor 1 encoder mode
- `SetM2EncoderMode(address, mode)`: Set motor 2 encoder mode
- `ReadEncoderModes(address)`: Read encoder modes

Example:
```python
# Reset encoders to zero
success = controller.ResetEncoders(address)
```

## Status and Diagnostic Functions

### Basic Status Information

- `ReadVersion(address)`: Read firmware version
- `ReadMainBatteryVoltage(address)`: Read main battery voltage
- `ReadLogicBatteryVoltage(address)`: Read logic battery voltage
- `GetVolts(address)`: Read both battery voltages
- `ReadTemp(address)`: Read temperature from sensor 1
- `ReadTemp2(address)`: Read temperature from sensor 2
- `GetTemps(address)`: Read both temperature sensors
- `ReadError(address)`: Read error status

Example:
```python
# Read battery voltage
success, voltage = controller.ReadMainBatteryVoltage(address)
voltage_volts = voltage / 10.0  # Convert to volts

# Read temperature
success, temp = controller.ReadTemp(address)
temp_celsius = temp / 10.0  # Convert to degrees Celsius
```

### Detailed Status

- `GetStatus(address)`: Read comprehensive status information
- `ReadCurrents(address)`: Read motor current values
- `ReadPWMs(address)`: Read PWM duty cycle values
- `ReadBuffers(address)`: Read command buffer status
- `GetSpeedErrors(address)`: Read speed error values
- `GetPosErrors(address)`: Read position error values
- `GetSpeedErrorLimit(address)`: Read speed error limits
- `GetPosErrorLimit(address)`: Read position error limits
- `SetSpeedErrorLimit(address, limit1, limit2)`: Set speed error limits
- `SetPosErrorLimit(address, limit1, limit2)`: Set position error limits

Example:
```python
# Read current values
success, current1, current2 = controller.ReadCurrents(address)

# Read buffer status
success, buffer1, buffer2 = controller.ReadBuffers(address)
```

### Timeout Control

- `SetTimeout(address, timeout)`: Set communication timeout
- `GetTimeout(address)`: Read communication timeout

Example:
```python
# Set timeout to 2 seconds
success = controller.SetTimeout(address, 2.0)
```

## Configuration Functions

### Controller Configuration

- `SetConfig(address, config)`: Set controller configuration
- `GetConfig(address)`: Read controller configuration
- `RestoreDefaults(address)`: Restore factory default settings
- `WriteNVM(address)`: Save settings to non-volatile memory
- `ReadNVM(address)`: Load settings from non-volatile memory
- `SetSerialNumber(address, serial_number)`: Set controller serial number
- `GetSerialNumber(address)`: Read controller serial number
- `SetNodeID(address, nodeid)`: Set CAN node ID
- `GetNodeID(address)`: Get CAN node ID

Example:
```python
# Read current configuration
success, config = controller.GetConfig(address)

# Save settings to non-volatile memory
success = controller.WriteNVM(address)
```

### Motor Configuration

- `SetM1DefaultAccel(address, accel, decel)`: Set default acceleration and deceleration for motor 1
- `SetM2DefaultAccel(address, accel, decel)`: Set default acceleration and deceleration for motor 2
- `SetM1DefaultSpeed(address, speed)`: Set default speed for motor 1
- `SetM2DefaultSpeed(address, speed)`: Set default speed for motor 2
- `GetDefaultSpeeds(address)`: Read default speeds
- `GetDefaultAccels(address)`: Read default accelerations
- `SetM1MaxCurrent(address, maxi, mini)`: Set maximum and minimum current limits for motor 1
- `SetM2MaxCurrent(address, maxi, mini)`: Set maximum and minimum current limits for motor 2
- `ReadM1MaxCurrent(address)`: Read maximum and minimum current limits for motor 1
- `ReadM2MaxCurrent(address)`: Read maximum and minimum current limits for motor 2

Example:
```python
# Set default acceleration and deceleration for motor 1
success = controller.SetM1DefaultAccel(address, 500, 1000)

# Set default speed for motor 1
success = controller.SetM1DefaultSpeed(address, 1000)
```

### Voltage Configuration

- `SetMainVoltages(address, min_voltage, max_voltage, auto_offset)`: Set main battery voltage limits
- `SetLogicVoltages(address, min_voltage, max_voltage)`: Set logic battery voltage limits
- `ReadMinMaxMainVoltages(address)`: Read main battery voltage limits
- `ReadMinMaxLogicVoltages(address)`: Read logic battery voltage limits
- `SetOffsets(address, offset1, offset2)`: Set voltage reading offsets
- `GetOffsets(address)`: Read voltage reading offsets

Example:
```python
# Set main battery voltage limits
success = controller.SetMainVoltages(address, 100, 140, 0)  # 10.0V to 14.0V
```

## CAN Bus Functions

Functions for CAN bus communication:

- `CANBufferState(address)`: Get the count of available raw CAN packets
- `CANPutPacket(address, cob_id, RTR, data)`: Send a raw CAN packet
- `CANGetPacket(address)`: Read a raw CAN packet
- `CANOpenWriteLocalDict(address, wIndex, bSubindex, lValue, bSize)`: Write to CANopen dictionary
- `CANOpenReadLocalDict(address, wIndex, bSubindex)`: Read from CANopen dictionary

Example:
```python
# Send a CAN packet
data = [0x01, 0x02, 0x03, 0x04]
success = controller.CANPutPacket(address, 0x123, 0, data)

# Read a CAN packet
success, can_id, rtr, length, data = controller.CANGetPacket(address)
```

## Advanced Functions

### Script Control

- `StartScript(address)`: Start the onboard script
- `StopScript(address)`: Stop the onboard script
- `GetScriptAutoRun(address)`: Get script autorun setting
- `SetScriptAutoRun(address, scriptauto_time)`: Set script autorun

Example:
```python
# Start the script
success = controller.StartScript(address)

# Set script to autorun after 500ms
success = controller.SetScriptAutoRun(address, 500)
```

### Emergency Stop

- `ResetEStop(address)`: Reset emergency stop
- `SetEStopLock(address, state)`: Set emergency stop lock state
- `GetEStopLock(address)`: Get emergency stop lock state

Example:
```python
# Reset emergency stop
success = controller.ResetEStop(address)

# Configure for software reset mode
success = controller.SetEStopLock(address, controller.ESTOP_SW_RESET)
```

## PWM Functions

### PWM Mode Configuration

- `SetPWMMode(address, mode1, mode2)`: Set PWM modes for both motors
- `ReadPWMMode(address)`: Read PWM modes
- `SetPWMIdle(address, idledelay1, idlemode1, idledelay2, idlemode2)`: Set PWM idle parameters
- `GetPWMIdle(address)`: Get PWM idle parameters

Example:
```python
# Set PWM idle parameters
success = controller.SetPWMIdle(address, 5.0, True, 5.0, True)
```

### Auxiliary PWM Control

- `SetAuxDutys(address, duty1, duty2, duty3, duty4, duty5)`: Set auxiliary PWM duty cycles
- `GetAuxDutys(address)`: Read auxiliary PWM duty cycles

Example:
```python
# Set auxiliary PWM duty cycles
success = controller.SetAuxDutys(address, 8192, -8192, 16384, 0, 0)
```

## Digital I/O Functions

### Digital Output Control

- `SetDOUT(address, index, action)`: Set digital output
- `GetDOUTS(address)`: Get digital outputs status

Example:
```python
# Set digital output
success = controller.SetDOUT(address, 0, 1)  # Set output 0 to ON

# Read all digital outputs
success, count, actions = controller.GetDOUTS(address)
```

### Pin Configuration

- `SetPinFunctions(address, S3mode, S4mode, S5mode)`: Set functions of pins S3, S4, and S5
- `ReadPinFunctions(address)`: Read functions of pins

Example:
```python
# Set pin functions
success = controller.SetPinFunctions(address, 1, 2, 3)
```

## Signaling and Stream Functions

### Signal Configuration

- `SetSignal(address, index, signal_type, mode, target, min_action, max_action, lowpass, timeout, loadhome, min_val, max_val, center, deadband, powerexp, minout, maxout, powermin, potentiometer)`: Configure signal parameters
- `GetSignals(address)`: Get configured signals
- `GetSignalsData(address)`: Get signal data

Example:
```python
# Get all configured signals
success, count, signals = controller.GetSignals(address)
```

### Stream Configuration

- `SetStream(address, index, stream_type, baudrate, timeout)`: Configure stream parameters
- `GetStreams(address)`: Get stream configurations

Example:
```python
# Configure a UART stream (type 1) at 9600 baud
success = controller.SetStream(address, 0, 1, 9600, 1000)
```

### Miscellaneous Configuration

- `SetAddressMixed(address, new_address, enable_mixing)`: Set mixed address and enable/disable mixing
- `GetAddressMixed(address)`: Get mixed address and mixing state
- `SetPriority(address, priority1, priority2, priority3)`: Set priority levels
- `GetPriority(address)`: Get priority levels

## Error Handling

The library includes custom exceptions for error handling:

- `BasicmicroError`: Base exception for all Basicmicro-related errors
- `CommunicationError`: Exception for communication errors
- `CommandError`: Exception for invalid commands
- `ResponseError`: Exception for invalid responses
- `ChecksumError`: Exception for checksum errors
- `TimeoutError`: Exception for timeout errors

Example:
```python
from basicmicro.exceptions import BasicmicroError, CommunicationError

try:
    controller.Open()
    # ... operations ...
except CommunicationError as e:
    print(f"Communication error: {e}")
except BasicmicroError as e:
    print(f"Controller error: {e}")
finally:
    controller.close()
```

## Utility Functions

The package includes utility functions in the `utils` module:

- `initialize_crc_table(polynomial)`: Initialize a CRC lookup table
- `calc_mixed(fb, lr)`: Calculate mixed mode drive values for differential steering

---

For complete details on each function, refer to the docstrings in the source code or consult the Basicmicro controller documentation.
# Basicmicro API Reference

This document provides detailed information about the Basicmicro library API.

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

## Encoder Functions

### Reading Encoders

- `ReadEncM1(address)`: Read motor 1 encoder
- `ReadEncM2(address)`: Read motor 2 encoder
- `GetEncoders(address)`: Read both encoders

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
- `ReadTemp(address)`: Read temperature from sensor 1
- `ReadTemp2(address)`: Read temperature from sensor 2
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

Example:
```python
# Read current values
success, current1, current2 = controller.ReadCurrents(address)

# Read buffer status
success, buffer1, buffer2 = controller.ReadBuffers(address)
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

Example:
```python
# Read current configuration
success, config = controller.GetConfig(address)

# Save settings to non-volatile memory
success = controller.WriteNVM(address)
```

### Motor Configuration

- `SetM1EncoderMode(address, mode)`: Set motor 1 encoder mode
- `SetM2EncoderMode(address, mode)`: Set motor 2 encoder mode
- `ReadEncoderModes(address)`: Read encoder modes
- `SetM1DefaultAccel(address, accel)`: Set default acceleration for motor 1
- `SetM2DefaultAccel(address, accel)`: Set default acceleration for motor 2
- `GetDefaultAccels(address)`: Read default accelerations

Example:
```python
# Set default acceleration for motor 1
success = controller.SetM1DefaultAccel(address, 500)

# Read encoder modes
success, mode1, mode2 = controller.ReadEncoderModes(address)
```

## CAN Bus Functions

Functions for CAN bus communication:

- `CANBufferState(address)`: Get the count of available CAN packets
- `CANPutPacket(address, cob_id, RTR, data)`: Send a CAN packet
- `CANGetPacket(address)`: Read a CAN packet
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

# Get script autorun setting
success, autorun_time = controller.GetScriptAutoRun(address)
```

### Emergency Stop

- `ResetEStop(address)`: Reset emergency stop
- `SetEStopLock(address, state)`: Set emergency stop lock state
- `GetEStopLock(address)`: Get emergency stop lock state

Example:
```python
# Reset emergency stop
success = controller.ResetEStop(address)
```

---

For complete details on each function, refer to the docstrings in the source code or consult the Basicmicro controller documentation.
##SMALdog2 Wire Protocol
This document describes the protocol used for the ethernet connection between
the computer and the Dynamixel/IMU/power board.

For the most part, we are following the Dynamixel protocol for ease. This
includes the ability to pass packets through to the bus, ability to query a
special board device (253), implementation of sync_read, and a special full sync
instruction which takes in new positions for the 12 joints, and then returns
the full system state.

###Ethernet Comms
Every packet starts with a magic header containing four bytes: 'SMAL' followed
by any number of concatenated Dynamixel packets.

##FULL_SYNC
The full_sync command is 0x85. The packet includes the 12 joint commands, and
causes a sync_read to be done onboard the controller. The controller then returns
the full system table.

####Dynamixel Data (28 Bytes)
There are 14 servos connected to the control board. Servos 1-12 are the leg
servos, while 13 and 14 are for the laser scanner. Each servo is given 2 bytes
of data representing the desired position. This value is signed, and if it is
-1, the servo is sent a torque off command. These values will be sent to the
servos with SYNC_WRITE command.

###Return Packet
####Packet Type (1 Byte)
As with the command packet, a return packet has a type immediately following
the magic number. For a return packet, this is 0xf1.

####Dynamixel Data (28 Bytes)
After a command packet is recieved, the control board will forward the values
using a SYNC_WRITE, and then read back the current position of all servos. The
return packet gives the read position of each servo. A -1 signifies that the
read failed.

####IMU Data (12 Bytes)
The IMU data includes the current accelerometer and gyro data. Format: TBD.

####Current Measurements (12 Bytes)
For each current sensor, a 16-bit signed integer representing current in 100mA
steps is returned. The order is:
 * Power inlet current sense
 * Computer power current sense
 * Left front leg
 * Right rear leg
 * Right front leg
 * Left rear leg

####Present Voltage
This is an 8-bit unsigned integer representing the battery voltage level in
100mV steps.

####Foot Touch Sensors (4 Bytes)
For each foot, we get an 8-bit value representing the force on the leg:
 * Left front foot
 * Right rear foot
 * Right front foot
 * Left rear foot

####Run Stop Status (1 Byte)
Returns >0 if runstop has been pressed.

###Other packets
The firmware will also accept Dynamixel bus commands, pass them through to the
bus, and forward the return packet to the PC. this is not used in the drivers,
but can be used by other scripts.

## Notes:
 * Leg data is always in the same order as a standard crawl gait: LF, RR, RF, LR

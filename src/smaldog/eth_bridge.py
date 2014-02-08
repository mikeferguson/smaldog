#!/usr/bin/env python

# Copyright (c) 2008-2013, Michael E. Ferguson. All right reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

## @file eth_bridge.py Low-level code to control an EthBridge.

from __future__ import print_function
import socket, time
from ax12 import *

## @brief This connects to the stm32 eth/xbee/ax/rx bridge over Ethernet.
class EthBridge:
    magic = 'SMAL'

    ## @brief Constructs an EthBridge instance and creates the ethernet socket.
    ##
    ## @param ip The IP address of the EthBridge.
    ##
    ## @param port The port number to transmit/recieve from.
    ##
    ## @param timeout How long to wait for packets to come back.
    def __init__(self, ip="192.168.0.42", port = 6707, timeout = 1.0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind( ("",0) )
        self.sock.setblocking(0)
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.p_id = 0

    ## @brief Send packet(s) over Ethernet.
    ##
    ## @param packets A list of integers that constitutes the packet(s).
    def send(self, packets):
        msg = self.magic + "".join([chr(m) for m in packets])
        self.sock.sendto(msg, 0, (self.ip, self.port))

    ## @brief Read packet(s) over Ethernet.   
    def recv(self):
        t = time.time()
        while(True):
            if time.time() > t + self.timeout:
                return None
            try:
                data = self.sock.recv(1024)
                # TODO: parse it
                if data[0:4] != self.magic:
                    print("Invalid Header")
                    continue
                return data[4:]
            except:
                pass

    def clear_recv(self):
        try:
            while(True):
                self.sock.recv(1024)
                time.sleep(0.001)
        except:
            pass

    ## @brief Read a dynamixel return packet.
    ##
    ## @param mode This should be 0 to start reading packet. 
    ##
    ## @return The parameters of the packet.
    def getPacket(self):
        data = self.recv()
        if data == None:
            return None
        else:
            # TODO: more error processing/checking here
            return [ord(b) for b in data[5:-1]]

    ## @brief Create a buffer for this packet
    ##
    ## @param index The servo ID.
    ##
    ## @param ins The instruction to send.
    ##
    ## @param params The parameters for the packet.
    ##
    ## @return A list of integers that can be sent over the port.
    def makePacket(self, index, ins, params, p_id = 0):
        length = 2 + len(params)
        checksum = 255 - ((index + length + ins + sum(params))%256)
        return [0xff, 0xff, index, length, ins] + params + [checksum, ]

    ## @brief Write values to registers.
    ##
    ## @param index The ID of the servo.
    ##
    ## @param start The starting register address to begin writing to.
    ##
    ## @param values The data to write, in a list.
    ##
    ## @return The error level.
    def write(self, index, start, values):
        self.send(self.makePacket(index, AX_WRITE_DATA, [start] + values))

    ## @brief Read values of registers.
    ##
    ## @param index The ID of the servo.
    ## 
    ## @param start The starting register address to begin the read at.
    ##
    ## @param length The number of bytes to read.
    ##
    ## @return A list of the bytes read, or -1 if failure.
    def read(self, index, start, length):
        self.clear_recv()
        self.send(self.makePacket(index, AX_READ_DATA, [start, length]))
        values = self.getPacket()
        if values == None:
            return -1        
        else:
            return values

    ## @brief Set the status of the LED on a servo.
    ##
    ## @param index The ID of the device to write.
    ##
    ## @param value 0 to turn the LED off, >0 to turn it on
    ##
    ## @return The error level.
    def setLed(self, index, value):
        return self.write(index, P_LED, [value])

    ## @brief Set the position of a servo.
    ##
    ## @param index The ID of the device to write.
    ##
    ## @param value The position to go to in, in servo ticks.
    ##
    ## @return The error level.
    def setPosition(self, index, value):
        return self.write(index, P_GOAL_POSITION_L, [value%256, value>>8])

    ## @brief Set the speed of a servo.
    ##
    ## @param index The ID of the device to write.
    ##
    ## @param value The speed to write.
    ##
    ## @return The error level.
    def setSpeed(self, index, value):
        return self.write(index, P_GOAL_SPEED_L, [value%256, value>>8])

    ## @brief Get the position of a servo.
    ##
    ## @param index The ID of the device to read.
    ##
    ## @return The servo position.
    def getPosition(self, index):
        values = self.read(index, P_PRESENT_POSITION_L, 2)
        try:
            return int(values[0]) + (int(values[1])<<8)
        except:
            return -1

    ## @brief Get the voltage of a device.
    ##
    ## @param index The ID of the device to read.
    ##
    ## @return The voltage, in Volts.
    def getVoltage(self, index):
        try:
            return int(self.read(index, P_PRESENT_VOLTAGE, 1)[0])/10.0
        except:
            return -1

    ## @brief Get the current of a device.
    ##
    ## @param index The ID of the device to read.
    ##
    ## @return The current, in Amps.
    def getCurrent(self, index):
        values = self.read(index, P_CURRENT_L, 2)
        try:
            return (int(values[0]) + (int(values[1])<<8)-2048)*0.0045
        except:
            return -1

if __name__=="__main__":
    eth = EthBridge()

    # blink AX-12 with ID1
    eth.setLed(1,1)
    time.sleep(0.5)
    eth.setLed(1,0)

    # set position, read it
    eth.setPosition(1,512)
    print(eth.getPosition(1))
    print(eth.getVoltage(1))

    # read from eth-bridge itself
    print(eth.getVoltage(253))
    print(eth.getCurrent(253))
    
    eth.recv()


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

## @file ax12.py Definitions of AX-12 control table.

# Control Table Symbolic Constants - EEPROM AREA
P_MODEL_NUMBER_L = 0
P_MODEL_NUMBER_H = 1
P_VERSION = 2
P_ID = 3
P_BAUD_RATE = 4
P_RETURN_DELAY_TIME = 5
P_CW_ANGLE_LIMIT_L = 6
P_CW_ANGLE_LIMIT_H = 7
P_CCW_ANGLE_LIMIT_L = 8
P_CCW_ANGLE_LIMIT_H = 9
P_SYSTEM_DATA2 = 10
P_LIMIT_TEMPERATURE = 11
P_DOWN_LIMIT_VOLTAGE = 12
P_UP_LIMIT_VOLTAGE = 13
P_MAX_TORQUE_L = 14
P_MAX_TORQUE_H = 15
P_RETURN_LEVEL = 16
P_ALARM_LED = 17
P_ALARM_SHUTDOWN = 18
P_OPERATING_MODE = 19
P_DOWN_CALIBRATION_L = 20
P_DOWN_CALIBRATION_H = 21
P_UP_CALIBRATION_L = 22
P_UP_CALIBRATION_H = 23
# Control Table Symbolic Constants - RAM AREA
P_TORQUE_ENABLE = 24
P_LED = 25
P_CW_COMPLIANCE_MARGIN = 26
P_CCW_COMPLIANCE_MARGIN = 27
P_CW_COMPLIANCE_SLOPE = 28
P_CCW_COMPLIANCE_SLOPE = 29
P_GOAL_POSITION_L = 30
P_GOAL_POSITION_H = 31
P_GOAL_SPEED_L = 32
P_GOAL_SPEED_H = 33
P_TORQUE_LIMIT_L = 34
P_TORQUE_LIMIT_H = 35
P_PRESENT_POSITION_L = 36
P_PRESENT_POSITION_H = 37
P_PRESENT_SPEED_L = 38
P_PRESENT_SPEED_H = 39
P_PRESENT_LOAD_L = 40
P_PRESENT_LOAD_H = 41
P_PRESENT_VOLTAGE = 42
P_PRESENT_TEMPERATURE = 43
P_REGISTERED_INSTRUCTION = 44
P_PAUSE_TIME = 45
P_MOVING = 46
P_LOCK = 47
P_PUNCH_L = 48
P_PUNCH_H = 49
# NEWER DEVICES ONLY!!!
P_CURRENT_L = 68
P_CURRENT_H = 69
P_TORQUE_MODE = 70
P_GOAL_TORQUE_L = 71
P_GOAL_TORQUE_H = 72
P_GOAL_ACCEL = 73

# Status Return Levels
AX_RETURN_NONE = 0
AX_RETURN_READ = 1
AX_RETURN_ALL = 2

# Instruction Set
AX_PING = 1
AX_READ_DATA = 2
AX_WRITE_DATA = 3
AX_REG_WRITE = 4
AX_ACTION = 5
AX_RESET = 6
AX_SYNC_WRITE = 131
AX_SYNC_READ = 132


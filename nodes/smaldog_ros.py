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

## @file smaldog_ros.py ROS interface for SMALdog.

from math import pi
from smaldog.eth_bridge import *
from smaldog.leg_ik import *
from smaldog.utilities import *
from smaldog.robot_defs import *

import rospy
from sensor_msgs.msg import JointState

class SMALdogROS:

    def __init__(self, execute=False):
        self.robot = SMALdog()
        if execute:
            self.conn = EthBridge()
        else:
            self.conn = None

        self.zero_stance = [[defs.X_COXA, -0.04, 0.85],   # Right Front
                            [-defs.X_COXA, -0.04, 0.85],  # Right Rear
                            [defs.X_COXA, 0.04, 0.85],    # Left Front
                            [-defs.X_COXA, 0.04, 0.85]]   # Left Rear

        self.joint_state_pub = rospy.Publisher('joint_states', JointState)

# TODO: command velocity
# TODO: tf between body_link and base_link
# TODO: odometry between base_link and odom
    
    def run(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            new_stance = gait.next(0.075, 0, 0)
            next_pose = solver.fullIK(new_stance)
            for pose in interpolate(start_pose, next_pose, 10):
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                for i in range(12):
                    msg.name.append(self.robot.names[i])
                    msg.position.append(pose[i])
                pub.publish(msg)
                r.sleep()
            start_pose = next_pose

if __name__=="__main__":
    rospy.init_node("smaldog_ros")
    robot = SMALdogROS()
    robot.run()


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
from smaldog.robot_defs import *
import smaldog.ax12

from smaldog.controllers.muybridge import *

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Twist

from tf.broadcaster import TransformBroadcaster

class SMALdogROS:

    def __init__(self, execute=False):
        self.robot = SMALdog()
        if execute:
            self.conn = EthBridge()
        else:
            self.conn = None

        self.zero_stance = [[self.robot.X_SHOULDER, -0.06, -0.1],   # Right Front
                            [-self.robot.X_SHOULDER, -0.06, -0.1],  # Right Rear
                            [self.robot.X_SHOULDER, 0.06, -0.1],    # Left Front
                            [-self.robot.X_SHOULDER, 0.06, -0.1]]   # Left Rear

        self.joint_state_pub = rospy.Publisher('joint_states', JointState)
        self.odom_broadcaster = TransformBroadcaster()

        self.x = 0
        self.y = 0

        rospy.Subscriber("cmd_vel", Twist, self.cmdCb)
    
    def run(self):
        controller = MuybridgeGaitController(self.robot, self.zero_stance)
        old_pose = self.robot.getIK(self.zero_stance)
        old_pose["x"] = 0.0
        old_pose["y"] = 0.0
        old_pose["r"] = 0.0

        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            # get stance from controller TODO: this really should be a motion plan
            new_stance = controller.next(self.x, 0, 0) # TODO: add y/r
            t = new_stance[0]
            # do IK
            new_pose = self.robot.getIK(new_stance[1])
            new_pose["x"] = controller.x
            new_pose["y"] = controller.y
            new_pose["r"] = controller.r

            # interpolate
            for pose in self.interpolate(old_pose, new_pose, int(t/0.02)):
                # publish
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                for name in self.robot.names:
                    msg.name.append(name)
                    msg.position.append(pose[name])
                    if pose[name] == float('nan'):
                        print "WARN", name, "is nan"
                self.joint_state_pub.publish(msg)

                # TF
                quaternion = Quaternion()
                quaternion.x = 0.0
                quaternion.y = 0.0
                quaternion.z = sin(pose["r"]/2)
                quaternion.w = cos(pose["r"]/2)
                self.odom_broadcaster.sendTransform(
                    (pose["x"], pose["y"], 0.095),
                    (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                    rospy.Time.now(),
                    "body_link",
                    "world"
                    )

                if self.conn:
                    packet = makeSyncWritePacket(convertToAX12(pose, self.robot))
                    self.conn.send(self.conn.makePacket(254, ax12.AX_SYNC_WRITE, packet))

                r.sleep()

            old_pose = new_pose

    def interpolate(self, start_pose, end_pose, iterations):
        diffs = dict()
        for name in start_pose.keys():
            diffs[name] = (end_pose[name] - start_pose[name])/iterations
        for i in range(iterations):
            pose = dict()
            for name in start_pose.keys():
                pose[name] = start_pose[name] + (diffs[name]*i)
            yield pose

    def cmdCb(self, msg):
        # TODO: add y/r
        if msg.linear.x > 0.05:
            self.x = 0.05
        else:
            self.x = msg.linear.x

if __name__=="__main__":
    rospy.init_node("smaldog_ros")
    robot = SMALdogROS(True)
    robot.run()


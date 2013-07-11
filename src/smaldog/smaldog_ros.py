#!/usr/bin/env python

from math import pi
from eth_bridge import *
from ik import *
from interpolation import *
from utilities import *
from robot_defs import *
import ax12

from gaits import *

import rospy
from sensor_msgs.msg import JointState

defs = SMALdog()
solver = MammalIK(defs)
#eth = EthBridge()

#zero_stance = [[defs.X_COXA+.035, -defs.Y_COXA-defs.L_COXA/2.0, 0.0],    # Right Front
#               [-defs.X_COXA-.035, -defs.Y_COXA-defs.L_COXA/2.0, 0.0],   # Right Rear
#               [defs.X_COXA+.035, defs.Y_COXA+defs.L_COXA/2.0, 0.0],     # Left Front
#               [-defs.X_COXA-.035, defs.Y_COXA+defs.L_COXA/2.0, 0.0]]    # Left Rear

zero_stance = [[defs.X_COXA, -0.04, 0.0],    # Right Front
               [-defs.X_COXA, -0.04, 0.0],   # Right Rear
               [defs.X_COXA, 0.04, 0.0],     # Left Front
               [-defs.X_COXA, 0.04, 0.0]]    # Left Rear

solver.bodyPosZ = (defs.L_FEMUR + defs.L_TIBIA)*0.8

start_pose = solver.fullIK(zero_stance)
print start_pose

rospy.init_node("smaldog_ros")
pub = rospy.Publisher('joint_states', JointState)

joint_names = ["rf_coxa_joint", "rf_femur_joint", "rf_tibia_joint",
               "rr_coxa_joint", "rr_femur_joint", "rr_tibia_joint",
               "lf_coxa_joint", "lf_femur_joint", "lf_tibia_joint",
               "lr_coxa_joint", "lr_femur_joint", "lr_tibia_joint"]

gait = RippleGait(zero_stance)

r = rospy.Rate(20)
while not rospy.is_shutdown():
    new_stance = gait.next(0.075, 0, 0)
    next_pose = solver.fullIK(new_stance)
    for pose in interpolate(start_pose, next_pose, 10):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        for i in range(12):
            msg.name.append(joint_names[i])
            msg.position.append(pose[i])
        pub.publish(msg)
        r.sleep()
    start_pose = next_pose

# TODO: command velocity
# TODO: tf between body_link and base_link
# TODO: odometry between base_link and odom

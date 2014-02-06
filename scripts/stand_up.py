#!/usr/bin/env python

# Really hacky standup, this will be tossed at some point

import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

from smaldog.robot_defs import *

robot = SMALdog()
stance = [[robot.X_SHOULDER+0.01, -0.04, -0.08],   # Right Front
          [-robot.X_SHOULDER+.06, 0.06, -0.08],   # Left Rear
          [robot.X_SHOULDER+.01, 0.06, -0.08],    # Left Front
          [-robot.X_SHOULDER+.01, -0.06, -0.08]]  # Right Rear

stance = [[robot.X_SHOULDER-.06, -0.07, -0.04],   # Right Front
          [-robot.X_SHOULDER+.06, 0.07, -0.04],   # Left Rear
          [robot.X_SHOULDER-.06, 0.07, -0.04],    # Left Front
          [-robot.X_SHOULDER+.06, -0.07, -0.04]]  # Right Rear

rospy.init_node("stand")

rospy.loginfo('Connecting to body_controller...')
client = actionlib.SimpleActionClient('body_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
client.wait_for_server()
rospy.loginfo('Connected.')

goal = FollowJointTrajectoryGoal()
goal.trajectory.joint_names = robot.names

goal.trajectory.points.append(JointTrajectoryPoint())
stance = [[robot.X_SHOULDER-.06, -0.07, -0.04],   # Right Front
          [-robot.X_SHOULDER+.06, 0.07, -0.04],   # Left Rear
          [robot.X_SHOULDER-.06, 0.07, -0.04],    # Left Front
          [-robot.X_SHOULDER+.06, -0.07, -0.04]]  # Right Rear
pose = robot.getIK(stance)
goal.trajectory.points[0].positions = [pose[name] for name in robot.names]
goal.trajectory.points[0].time_from_start = rospy.Duration(0.0)
print [pose[name] for name in robot.names]

for i in range(4):
    goal.trajectory.points.append(JointTrajectoryPoint())
    stance[0][2] -= 0.01
    stance[1][2] -= 0.01
    stance[2][2] -= 0.01
    stance[3][2] -= 0.01
    pose = robot.getIK(stance)
    goal.trajectory.points[-1].positions = [pose[name] for name in robot.names]
    goal.trajectory.points[-1].time_from_start = goal.trajectory.points[-2].time_from_start + rospy.Duration(0.5)
    #print [pose[name] for name in robot.names]

goal.trajectory.header.stamp = rospy.Time.now()
client.send_goal(goal)
client.wait_for_result()

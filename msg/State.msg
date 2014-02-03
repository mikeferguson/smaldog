# This message carries the remainder of the robot state.
# For a full system state, the user should subscribe to
# the joint_states, imu, and robot_state topics.

# Battery voltage as measured by the controller board
float32 battery_voltage

# Is the run-stop pressed?
bool run_stopped

# Total current measured
float32 current_total

# Current supplied to the computer (this is on the 19V rail)
float32 current_computer

# Current supplied to each leg
float32 current_left_front_leg
float32 current_right_front_leg
float32 current_left_rear_leg
float32 current_right_rear_leg

# Status of the leg sensors. Currently these are just
# 0 (not touching ground) or 1 (touching ground)
uint8 status_left_front_foot
uint8 status_right_front_foot
uint8 status_left_rear_foot
uint8 status_right_rear_foot

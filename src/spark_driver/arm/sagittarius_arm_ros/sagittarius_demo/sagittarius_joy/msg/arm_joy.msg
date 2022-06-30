
# This message is used specifically in the sagittarius_joy package
#
# Maps raw 'joy' commands to more specific ones to control an sagittarius

# enum values that define the joystick controls for the robot

#########################################################################################################

# Control the motion of the virtual 'ee_gripper_link' or end effector using the modern_robotics_ik engine
# Position Control
int8 EE_X_INC = 1
int8 EE_X_DEC = 2
int8 EE_Y_INC = 3
int8 EE_Y_DEC = 4
int8 EE_Z_INC = 5
int8 EE_Z_DEC = 6

# Orientation Control
int8 EE_ROLL_CCW = 7
int8 EE_ROLL_CW = 8
int8 EE_PITCH_UP = 9
int8 EE_PITCH_DOWN = 10
int8 EE_YAW_LEFT = 11
int8 EE_YAW_RIGHT = 12

#########################################################################################################

# Control the motion of independent joints on the Arm or send predefined robot poses
# Pose Control
int8 UP_POSE = 13
int8 HOME_POSE = 14
int8 SLEEP_POSE = 15

# Position and Orientation reset
int8 POSITION_RESET = 16
int8 ORIENTATION_RESET = 17

#########################################################################################################

# Customize configurations for the Sagittarius Arm
# Inc/Dec Joint speed
int8 SPEED_INC = 18
int8 SPEED_DEC = 19

# Inc/Dec Gripper spacing
int8 GRIPPER_SPACING_INC = 20
int8 GRIPPER_SPACING_DEC = 21

# Torque On/Off all servos
int8 TORQUE_ON = 22
int8 TORQUE_OFF = 23

# Torque Inc/Dec all servos
int8 TORQUE_INC = 24
int8 TORQUE_DEC = 25

#########################################################################################################

# Other function for the Sagittarius Arm
# Super Key
int8 SUPER_KEY = 26

#########################################################################################################

# Control the motion of the Sagittarius Arm
int8 ee_x_cmd
int8 ee_y_cmd
int8 ee_z_cmd
int8 ee_roll_cmd
int8 ee_pitch_cmd
int8 ee_yaw_cmd

# Independent Joint/Pose Control
int8 pose_cmd
int8 reset_cmd

# Misc. Configs
int8 speed_cmd
int8 gripper_spacing_cmd
int8 torque_cmd
int8 torque_lev_cmd

# Super key
int8 super_cmd

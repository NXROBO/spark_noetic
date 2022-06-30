#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "sagittarius_joy/arm_joy.h"

// Xbox 360 Controller button mappings
static const std::map<std::string, int> xbox360 = {{"SLEEP_POSE", 0}, // buttons start here
                                                   {"HOME_POSE", 1},
                                                   {"SUPER_KEY", 2},
                                                   {"UP_POSE", 3},
                                                   {"EE_Z_DEC", 4},
                                                   {"EE_Z_INC", 5},
                                                   {"EE_ROLL_CCW", 6},
                                                   {"EE_ROLL_CW", 7},
                                                   {"TORQUE_ONOFF", 8},
                                                   {"POSITION_RESET", 9},
                                                   {"ORIENTATION_RESET", 10},
                                                   {"EE_YAW", 0}, // axes start here
                                                   {"EE_X", 1},
                                                   {"GRIPPER_SPACING_INC", 2},
                                                   {"EE_Y", 3},
                                                   {"EE_PITCH", 4},
                                                   {"GRIPPER_SPACING_DEC", 5},
                                                   {"TORQUE", 6},
                                                   {"SPEED", 7}};

// PS4 Controller button mappings
static const std::map<std::string, int> ps4 = {{"SLEEP_POSE", 0}, // buttons start here
                                               {"HOME_POSE", 1},
                                               {"UP_POSE", 2},
                                               {"SUPER_KEY", 3},
                                               {"EE_Z_DEC", 4},
                                               {"EE_Z_INC", 5},
                                               {"null_1", 6},
                                               {"null_2", 7},
                                               {"EE_ROLL_CCW", 8},
                                               {"EE_ROLL_CW", 9},
                                               {"TORQUE_ONOFF", 10},
                                               {"POSITION_RESET", 11},
                                               {"ORIENTATION_RESET", 12},
                                               {"EE_YAW", 0}, // axes start here
                                               {"EE_X", 1},
                                               {"GRIPPER_SPACING_INC", 2},
                                               {"EE_Y", 3},
                                               {"EE_PITCH", 4},
                                               {"GRIPPER_SPACING_DEC", 5},
                                               {"TORQUE", 6},
                                               {"SPEED", 7}};

ros::Publisher pub_joy_cmd;            // ROS Publisher to publish ArmJoy messages
sagittarius_joy::arm_joy prev_joy_cmd; // Keep track of the previously commanded ArmJoy message so that only unique messages are published
std::map<std::string, int> cntlr;      // Holds the controller button mappings
std::string controller_type;           // Holds the name of the controller received from the ROS Parameter server
double threshold;                      // Joystick sensitivity threshold

/// @brief Joystick callback to create custom ArmJoy messages to control the Arm
/// @param msg - raw sensor_msgs::Joy data
void joy_state_cb(const sensor_msgs::Joy &msg)
{
  static double time_start;
  static bool timer_started = false;
  static bool torque_cmd_last_state = true;
  sagittarius_joy::arm_joy joy_cmd;

  // Check the ee_x_cmd
  if (msg.axes.at(cntlr["EE_X"]) >= threshold)
    joy_cmd.ee_x_cmd = sagittarius_joy::arm_joy::EE_X_INC;
  else if (msg.axes.at(cntlr["EE_X"]) <= -threshold)
    joy_cmd.ee_x_cmd = sagittarius_joy::arm_joy::EE_X_DEC;

  // Check the ee_y_cmd
  if (msg.axes.at(cntlr["EE_Y"]) >= threshold)
    joy_cmd.ee_y_cmd = sagittarius_joy::arm_joy::EE_Y_INC;
  else if (msg.axes.at(cntlr["EE_Y"]) <= -threshold)
    joy_cmd.ee_y_cmd = sagittarius_joy::arm_joy::EE_Y_DEC;

  // Check the ee_z_cmd
  if (msg.buttons.at(cntlr["EE_Z_INC"]) == 1)
    joy_cmd.ee_z_cmd = sagittarius_joy::arm_joy::EE_Z_INC;
  else if (msg.buttons.at(cntlr["EE_Z_DEC"]) == 1)
    joy_cmd.ee_z_cmd = sagittarius_joy::arm_joy::EE_Z_DEC;

  // Check the ee_roll_cmd
  if (msg.buttons.at(cntlr["EE_ROLL_CW"]) == 1)
    joy_cmd.ee_roll_cmd = sagittarius_joy::arm_joy::EE_ROLL_CW;
  else if (msg.buttons.at(cntlr["EE_ROLL_CCW"]) == 1)
    joy_cmd.ee_roll_cmd = sagittarius_joy::arm_joy::EE_ROLL_CCW;

  // Check the ee_pitch_cmd
  if (msg.axes.at(cntlr["EE_PITCH"]) >= threshold)
    joy_cmd.ee_pitch_cmd = sagittarius_joy::arm_joy::EE_PITCH_DOWN;
  else if (msg.axes.at(cntlr["EE_PITCH"]) <= -threshold)
    joy_cmd.ee_pitch_cmd = sagittarius_joy::arm_joy::EE_PITCH_UP;

  // Check the ee_yaw_cmd
  if (msg.axes.at(cntlr["EE_YAW"]) >= threshold)
    joy_cmd.ee_yaw_cmd = sagittarius_joy::arm_joy::EE_YAW_LEFT;
  else if (msg.axes.at(cntlr["EE_YAW"]) <= -threshold)
    joy_cmd.ee_yaw_cmd = sagittarius_joy::arm_joy::EE_YAW_RIGHT;

  // Check the pose_cmd
  if (msg.buttons.at(cntlr["HOME_POSE"]) == 1)
    joy_cmd.pose_cmd = sagittarius_joy::arm_joy::HOME_POSE;
  else if (msg.buttons.at(cntlr["SLEEP_POSE"]) == 1)
    joy_cmd.pose_cmd = sagittarius_joy::arm_joy::SLEEP_POSE;
  else if (msg.buttons.at(cntlr["UP_POSE"]) == 1)
    joy_cmd.pose_cmd = sagittarius_joy::arm_joy::UP_POSE;

  // Check the reset_cmd
  if (msg.buttons.at(cntlr["POSITION_RESET"]) == 1)
    joy_cmd.reset_cmd = sagittarius_joy::arm_joy::POSITION_RESET;
  else if (msg.buttons.at(cntlr["ORIENTATION_RESET"]) == 1)
    joy_cmd.reset_cmd = sagittarius_joy::arm_joy::ORIENTATION_RESET;

  // Check the gripper_cmd
  if (msg.axes.at(cntlr["GRIPPER_SPACING_INC"]) <= -0.2)
    joy_cmd.gripper_spacing_cmd = sagittarius_joy::arm_joy::GRIPPER_SPACING_INC;
  else if (msg.axes.at(cntlr["GRIPPER_SPACING_DEC"]) <= -0.2)
    joy_cmd.gripper_spacing_cmd = sagittarius_joy::arm_joy::GRIPPER_SPACING_DEC;

  // Check the speed_cmd
  if (msg.axes.at(cntlr["SPEED"]) == 1)
    joy_cmd.speed_cmd = sagittarius_joy::arm_joy::SPEED_INC;
  else if (msg.axes.at(cntlr["SPEED"]) == -1)
    joy_cmd.speed_cmd = sagittarius_joy::arm_joy::SPEED_DEC;

  // Check the torque_cmd
  if (msg.buttons.at(cntlr["TORQUE_ONOFF"]) == 1)
  {
    time_start = ros::Time::now().toSec();
    timer_started = true;
  }
  else if (msg.buttons.at(cntlr["TORQUE_ONOFF"]) == 0)
  {
    if (timer_started && ros::Time::now().toSec() - time_start > 0)
      if (torque_cmd_last_state == true)
      {
        joy_cmd.torque_cmd = sagittarius_joy::arm_joy::TORQUE_OFF;
        torque_cmd_last_state = false;
      }
      else
      {
        joy_cmd.torque_cmd = sagittarius_joy::arm_joy::TORQUE_ON;
        torque_cmd_last_state = true;
      }
    timer_started = false;
  }

  // Check the torque_lev_cmd
  if (msg.axes.at(cntlr["TORQUE"]) == 1)
    joy_cmd.torque_lev_cmd = sagittarius_joy::arm_joy::TORQUE_INC;
  else if (msg.axes.at(cntlr["TORQUE"]) == -1)
    joy_cmd.torque_lev_cmd = sagittarius_joy::arm_joy::TORQUE_DEC;

  // Check the super_cmd
  if (msg.buttons.at(cntlr["SUPER_KEY"]) == 1)
    joy_cmd.super_cmd = sagittarius_joy::arm_joy::SUPER_KEY;

  // Only publish a ArmJoy message if any of the following fields have changed.
  if (!(prev_joy_cmd.ee_x_cmd == joy_cmd.ee_x_cmd &&
        prev_joy_cmd.ee_y_cmd == joy_cmd.ee_y_cmd &&
        prev_joy_cmd.ee_z_cmd == joy_cmd.ee_z_cmd &&
        prev_joy_cmd.ee_roll_cmd == joy_cmd.ee_roll_cmd &&
        prev_joy_cmd.ee_pitch_cmd == joy_cmd.ee_pitch_cmd &&
        prev_joy_cmd.ee_yaw_cmd == joy_cmd.ee_yaw_cmd &&
        prev_joy_cmd.pose_cmd == joy_cmd.pose_cmd &&
        prev_joy_cmd.reset_cmd == joy_cmd.reset_cmd &&
        prev_joy_cmd.speed_cmd == joy_cmd.speed_cmd &&
        prev_joy_cmd.gripper_spacing_cmd == joy_cmd.gripper_spacing_cmd &&
        prev_joy_cmd.torque_cmd == joy_cmd.torque_cmd &&
        prev_joy_cmd.torque_lev_cmd == joy_cmd.torque_lev_cmd &&
        prev_joy_cmd.super_cmd == joy_cmd.super_cmd))
    pub_joy_cmd.publish(joy_cmd);
  prev_joy_cmd = joy_cmd;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sagittarius_joy");
  ros::NodeHandle n;
  ros::param::get("~threshold", threshold);
  ros::param::get("~controller", controller_type);
  if (controller_type == "xbox360")
    cntlr = xbox360;
  else if (controller_type == "ps4")
    cntlr = ps4;
  // else
  //   cntlr = ps4;
  ros::Subscriber sub_joy_raw = n.subscribe("commands/joy_raw", 10, joy_state_cb);
  pub_joy_cmd = n.advertise<sagittarius_joy::arm_joy>("commands/joy_processed", 10);
  ros::spin();
  return 0;
}

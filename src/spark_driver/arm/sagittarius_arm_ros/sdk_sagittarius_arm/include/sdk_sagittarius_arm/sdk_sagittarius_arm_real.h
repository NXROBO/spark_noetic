#ifndef SDK_SAGITTARIUS_ARM_REAL_
#define SDK_SAGITTARIUS_ARM_REAL_

#include <ros/ros.h>
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <urdf/model.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sdk_sagittarius_arm/ArmRadControl.h>
#include "sdk_sagittarius_arm/SingleRadControl.h"
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_common_serial.h>
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_parser.h>
#include <sdk_sagittarius_arm/ArmInfo.h>
#include <sdk_sagittarius_arm/ServoRtInfo.h>

struct Servo
{
    std::string name;                                                     // 舵机所在的关节名
    uint8_t servo_id;                                                     // 舵机ID。未启用
};



namespace sdk_sagittarius_arm
{
    class SagittariusArmReal
    {
    public:
        SagittariusArmReal(ros::NodeHandle nh, ros::NodeHandle pnh, const std::string robot_name, const std::string robot_model);
        virtual ~SagittariusArmReal();
        void JointStatesCb(const sensor_msgs::JointState& cmd_arm);
        void arm_joint_trajectory_msg_callback(const trajectory_msgs::JointTrajectory &msg);
        void arm_joint_trajectory_action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
        void arm_gripper_trajectory_action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
        void arm_gripper_trajectory_msg_callback(const trajectory_msgs::JointTrajectory &msg);
        void arm_execute_joint_trajectory(const ros::TimerEvent&);
        short arm_calculate_gripper_degree_position(const float dist);
        void arm_set_gripper_linear_position(const float dist);
        void arm_set_single_joint_degree_position(short g_degree);
        void arm_execute_gripper_trajectory(const ros::TimerEvent&);
        void arm_set_joint_positions(const double joint_positions[], double diff_time);
        void arm_write_joint_commands(const sdk_sagittarius_arm::ArmRadControl &msg);
        void arm_write_gripper_command(const std_msgs::Float64 &msg);
        void ControlTorque(const std_msgs::String::ConstPtr &msg);
        void arm_get_servo_configs(void);
        bool arm_get_servo_info(sdk_sagittarius_arm::ServoRtInfo::Request & req, sdk_sagittarius_arm::ServoRtInfo::Response & res);
        bool arm_get_robot_info(sdk_sagittarius_arm::ArmInfo::Request & req, sdk_sagittarius_arm::ArmInfo::Response & res);
        bool GetAndSetServoAcceleration(sdk_sagittarius_arm::CSDarmCommon *pt);
        bool GetAndSetServoVelocity(sdk_sagittarius_arm::CSDarmCommon *pt);
        bool GetAndSetServoTorque(sdk_sagittarius_arm::CSDarmCommon *pt);
    private:
        std::vector<Servo> arm_joints;
        bool execute_joint_traj;
        bool execute_gripper_traj;
        bool torque_status;
        size_t joint_num_write;
        trajectory_msgs::JointTrajectory jnt_tra_msg,gripper_tra_msg;                                   	
        actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> *joint_action_server;
        actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> *gripper_action_server; 
        sensor_msgs::JointState motionPlan_position;
        sdk_sagittarius_arm::CSDarmCommon *pSDKarm;
        sdk_sagittarius_arm::CSDarmCommon *pTest;
        sdk_sagittarius_arm::CSDKarmParser *pParser;
        ros::ServiceServer srv_get_robot_info;                                                    // Service to get information about the robot info
        ros::ServiceServer srv_get_servo_info;                                                    // Service to get information about the servo info
        ros::Subscriber sub_js, sub_joint_commands, sub_gripper_command, sub_ct;
        float angle[10];
        ros::Timer tmr_joint_traj;
        ros::Timer tmr_gripper_traj;
        sensor_msgs::JointState joint_states;
        double joint_start_time;
        bool rviz_control;
        bool servo_control_trajectory;                                  
        double gripper_start_time;                                  // 爪子控制的开始时间
        std::vector<double> home_positions;                         
        std::vector<double> sleep_positions;   
        uint8_t *joint_ids_read;                                    
        uint8_t *joint_ids_write;        
        std::vector<Servo> all_joints;                              
        const std::string robot_name;                               
        const std::string robot_model;                              

    };
} // sdk_sagittarius_arm

#endif // SDK_SAGITTARIUS_ARM_REAL_

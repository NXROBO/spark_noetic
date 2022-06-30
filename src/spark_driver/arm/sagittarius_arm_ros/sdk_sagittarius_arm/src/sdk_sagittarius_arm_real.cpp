/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2021, NXROBO.
 * All rights reserved.
 * Author: litian.zhuang  <litian.zhuang@nxrobo.com>   
 */
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_real.h>
namespace sdk_sagittarius_arm
{
    SagittariusArmReal::SagittariusArmReal(ros::NodeHandle nh, ros::NodeHandle pnh, const std::string robot_name, const std::string robot_model)
        : robot_name(robot_name), robot_model(robot_model)
    {
        /*Check whether serialname is provided*/
        bool isSerialConnection = false;
        bool exit_free_torque = false;
    
        std::string strSerialName;
        std::string strBaudrate;
        bool rviz_coqntrol;
        rviz_control = false;
        servo_control_trajectory = false;
        execute_joint_traj = false;
        execute_gripper_traj = false;
        torque_status = true;

        if(pnh.getParam("just_rviz_control", rviz_control))
        {
            ROS_WARN("rviz_control is %d",rviz_coqntrol);
        }
        if(pnh.getParam("servo_control_trajectory", servo_control_trajectory))  //直接使用舵机的规划。
        {
            ROS_WARN("servo_control_trajectory is %d",servo_control_trajectory);
        }
        if(pnh.getParam("exit_free_torque", exit_free_torque))  //程序退出时，是否释放舵机扭矩。
        {
            ROS_WARN("exit_free_torque is %d",exit_free_torque);
        }


        if(pnh.getParam("serialname", strSerialName))
        {
            isSerialConnection = true;
        }
        if(pnh.getParam("baudrate", strBaudrate))
        {
            isSerialConnection = true;
        }
        /*Get configured time limit*/
        int iTimeLimit = 5;
        pnh.param("timelimit", iTimeLimit, 5);
        pSDKarm = NULL;
        int result = sdk_sagittarius_arm::ExitError;

        arm_get_servo_configs();
        srv_get_robot_info = nh.advertiseService("get_robot_info", &SagittariusArmReal::arm_get_robot_info, this);

        srv_get_servo_info = nh.advertiseService("get_servo_info", &SagittariusArmReal::arm_get_servo_info, this);

/*        pTest = new sdk_sagittarius_arm::CSDarmCommonSerial("/dev/ttyUSB0", "115200", iTimeLimit, nh, pnh); //just for test
        result = pTest->Init();     //just for test*/
        pSDKarm = new sdk_sagittarius_arm::CSDarmCommonSerial(strSerialName, strBaudrate, iTimeLimit, nh, pnh, exit_free_torque);
        result = pSDKarm->Init();
        sub_ct = nh.subscribe("control_torque", 1, &SagittariusArmReal::ControlTorque, this);

        if(rviz_control)
        {
            sub_js = nh.subscribe("joint_states", 1, &SagittariusArmReal::JointStatesCb, this);
            //motionPlan_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",1);
        }
        else
        {
            pSDKarm->StartReceiveSerail();
            /*Device has been initliazed successfully*/
            //

            tmr_joint_traj = nh.createTimer(ros::Duration(1/100), &SagittariusArmReal::arm_execute_joint_trajectory, this);
            tmr_gripper_traj = nh.createTimer(ros::Duration(1/100), &SagittariusArmReal::arm_execute_gripper_trajectory, this);
            //
            joint_action_server = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>
                    (nh, "sagittarius_arm_controller/follow_joint_trajectory", boost::bind(&SagittariusArmReal::arm_joint_trajectory_action_callback,  this, _1),false);

            gripper_action_server = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>
                    (nh, "sagittarius_gripper_controller/follow_joint_trajectory", boost::bind(&SagittariusArmReal::arm_gripper_trajectory_action_callback, this, _1),false);
            joint_action_server->start();
            gripper_action_server->start();
        }
        GetAndSetServoVelocity(pSDKarm);
        sleep(2);
        GetAndSetServoAcceleration(pSDKarm);        
        sleep(2);
        GetAndSetServoTorque(pSDKarm);
        sleep(2);

        if (!rviz_control)
        {
            std::string yaml_file;
            ros::param::get("~servo_configs", yaml_file);
            yaml_file += robot_model + ".yaml";
            YAML::Node sag_config = YAML::LoadFile(yaml_file.c_str());
            if (sag_config.IsNull())
            {
                ROS_ERROR("Config file not found.");
            }
            else
            {
                sensor_msgs::JointState joint_state;
                joint_state = pSDKarm->joint_states;
                bool joint_2_3_outrange_flag = false;
                double angel_check = 0;
                angel_check = sag_config["joint2"]["Max_Position_Limit"].as<double>() / 1800.0 * PI;
                if (joint_state.position[1] > angel_check)
                {
                    ROS_WARN("joint2 is out of range.");
                    joint_state.position[1] = angel_check;
                    joint_2_3_outrange_flag = true;
                }
                angel_check = sag_config["joint3"]["Min_Position_Limit"].as<double>() / 1800.0 * PI;
                if (pSDKarm->joint_states.position[2] < angel_check)
                {
                    ROS_WARN("joint3 is out of range.");
                    joint_state.position[2] = angel_check;
                    joint_2_3_outrange_flag = true;
                }
                if (joint_2_3_outrange_flag)
                    pSDKarm->SendArmAllServerCB(
                        (float)joint_state.position[0],
                        (float)joint_state.position[1],
                        (float)joint_state.position[2],
                        (float)joint_state.position[3],
                        (float)joint_state.position[4],
                        (float)joint_state.position[5]);
            }
        }

        sub_joint_commands = nh.subscribe("joint/commands", 100, &SagittariusArmReal::arm_write_joint_commands, this);
    	sub_gripper_command = nh.subscribe("gripper/command", 100, &SagittariusArmReal::arm_write_gripper_command, this);

    }

    SagittariusArmReal::~SagittariusArmReal()
    {
        if(pSDKarm != NULL)
        {
            delete pSDKarm;
        }

    }

    bool SagittariusArmReal::GetAndSetServoVelocity(sdk_sagittarius_arm::CSDarmCommon *pt)
    {
        std::string str_arm_vel;
        int arm_vel;
        ros::param::param<int>("~arm_velocity", arm_vel, 1000);
        ROS_WARN("arm_vel is %d",arm_vel);
        if(pt != NULL)
            pt->SetArmVel(arm_vel);     
       return true;

    }

    bool SagittariusArmReal::GetAndSetServoAcceleration(sdk_sagittarius_arm::CSDarmCommon *pt)
    {
        std::string str_arm_acc;
        int arm_acc;
        ros::param::param<int>("~arm_acceleration", arm_acc, 0);
        ROS_WARN("arm_acceleration is %d",arm_acc);
        if(pt != NULL)
            pt->SetArmAcc(arm_acc);
        return true;
    }

    bool SagittariusArmReal::GetAndSetServoTorque(sdk_sagittarius_arm::CSDarmCommon *pt)
    {
        std::string str_arm_acc;
        int arm_torque[7];
        ros::param::param<int>("~servo_torque1", arm_torque[0], 1000);
        ROS_WARN("servo_torque1 is %d",arm_torque[0]);
        ros::param::param<int>("~servo_torque2", arm_torque[1], 1000);
        ROS_WARN("servo_torque2 is %d",arm_torque[1]);
        ros::param::param<int>("~servo_torque3", arm_torque[2], 1000);
        ROS_WARN("servo_torque3 is %d",arm_torque[2]);
        ros::param::param<int>("~servo_torque4", arm_torque[3], 1000);
        ROS_WARN("servo_torque4 is %d",arm_torque[3]);
        ros::param::param<int>("~servo_torque5", arm_torque[4], 1000);
        ROS_WARN("servo_torque5 is %d",arm_torque[4]);
        ros::param::param<int>("~servo_torque6", arm_torque[5], 1000);
        ROS_WARN("servo_torque6 is %d",arm_torque[5]);
        ros::param::param<int>("~servo_torque7", arm_torque[6], 1000);
        ROS_WARN("servo_torque7 is %d",arm_torque[6]);
        if(pt != NULL)
            pt->SetArmTorque(arm_torque);
        return true;
    }  

    void SagittariusArmReal::arm_get_servo_configs(void)
    {
        std::string yaml_file;
        ros::param::get("~servo_configs", yaml_file);
        yaml_file += robot_model + ".yaml";
        YAML::Node sag_config = YAML::LoadFile(yaml_file.c_str());
        if (sag_config.IsNull())
        {
            ROS_ERROR("Config file not found.");
            return;
        }

        // Define the home and sleep positions for the arm joints
        YAML::Node sleep_node = sag_config["sleep"];
        for (auto const& value: sleep_node)
        {
            sleep_positions.push_back(value.as<double>());
            home_positions.push_back(0);
        }
        YAML::Node order_node = sag_config["order"];
        YAML::Node singles_node = sag_config["singles"];
        std::vector <YAML::Node> nodes {order_node, singles_node};
        joint_num_write = 0;
        if (order_node.size() > 0)
            ROS_INFO("size is >0");
        for (auto const& node: nodes)
        {
            for(size_t i{0}; i < node.size(); i++)
            {
                std::string name = node[i].as<std::string>();
                YAML::Node item = sag_config[name];
                int32_t id = item["ID"].as<int32_t>();
                int32_t secondary_id = item["Secondary_ID"].as<int32_t>();
                Servo motor = {name, (uint8_t) id};
                std::vector<uint8_t> shadow_list {(uint8_t) id};
                if (node == sag_config["order"] && name != "joint_gripper_left" && secondary_id == 255)
                    joint_num_write++;
            }
        }
        joint_ids_write = new uint8_t[joint_num_write];
        size_t cntr = 0;
        for (auto const& node: nodes)
        {
            for (size_t i{0}; i < node.size(); i++)
            {
                std::string name = node[i].as<std::string>();
                YAML::Node item = sag_config[name];
                int32_t id = item["ID"].as<int32_t>();

                for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
                {
                    std::string item_name = it_item->first.as<std::string>();
                    int32_t value = it_item->second.as<int32_t>();

                    if (item_name == "ID")
                        continue;
                    else if (item_name == "Secondary_ID" && value == 255)
                    {
                        Servo joint = {name, (uint8_t) id};
                        all_joints.push_back(joint);

                        if (node == sag_config["order"])
                        {
                            arm_joints.push_back(joint);
                            if (name != "joint_gripper_left")
                            {
                                joint_ids_write[cntr] = (uint8_t) id;
                                cntr++;
                            }
                        }
                    }
                }
            }
        }
    }

    bool SagittariusArmReal::arm_get_servo_info(sdk_sagittarius_arm::ServoRtInfo::Request & req, sdk_sagittarius_arm::ServoRtInfo::Response & res)
    {
        int t_cnt = 500;
        if (req.servo_id>0)
        {
            pSDKarm->servo_state.flag = 0;
            pSDKarm->SendGetServoRealTimeInfo(req.servo_id);
        }    
        else
        {
            ROS_ERROR("the servo id must be more than 0");
            return false;
        }    
        while((pSDKarm->servo_state.flag == 0)&&(t_cnt--))
        {
            usleep(1000*2);
        }
        if(pSDKarm->servo_state.flag)
        {
            res.speed = pSDKarm->servo_state.speed;
            res.voltage = pSDKarm->servo_state.voltage;
            res.current = pSDKarm->servo_state.current;
            res.payload = pSDKarm->servo_state.payload;
            return true;
        }
        else
        {
            ROS_ERROR("get the servo state: timeout");
            return false;
        }   


    }

    bool SagittariusArmReal::arm_get_robot_info(sdk_sagittarius_arm::ArmInfo::Request & req, sdk_sagittarius_arm::ArmInfo::Response & res)
    {
        // Parse the urdf model to get joint position and velocity limits
        if (!ros::param::has("robot_description"))
        {
            ROS_ERROR("The sagittarius/robot_description parameter was not found!");
            return false;
        }
        urdf::Model model;
        model.initParam(robot_name + "/robot_description");
        for (auto const &joint : all_joints)
        {   
            urdf::JointConstSharedPtr ptr;
            if (joint.name == "joint_gripper_left")
            {
                ptr = model.getJoint("joint_gripper_left");
                res.lower_gripper_limit = ptr->limits->lower;
                res.upper_gripper_limit = ptr->limits->upper;
                res.use_gripper = true;
            }
            ptr = model.getJoint(joint.name);
            res.lower_joint_limits.push_back(ptr->limits->lower);
            res.upper_joint_limits.push_back(ptr->limits->upper);
            res.velocity_limits.push_back(ptr->limits->velocity);
            res.joint_names.push_back(joint.name);
            res.joint_ids.push_back(joint.servo_id);

        }
        res.home_pos = home_positions;
        res.sleep_pos = sleep_positions;
        res.num_joints = joint_num_write;
        res.num_single_joints = all_joints.size();
        return true;
    }

    void SagittariusArmReal::ControlTorque(const std_msgs::String::ConstPtr &msg)
    {
        ROS_INFO("ControlTorque:%s.\n",msg->data.c_str());
        if (msg->data == "open")
        {
            torque_status = true;
            pSDKarm->SendArmLockOrFree(1);
        }
        else
        {
            torque_status = false;
            pSDKarm->SendArmLockOrFree(0);
        }
    }

    void SagittariusArmReal::JointStatesCb(const sensor_msgs::JointState& cmd_arm)
    {
        if(pSDKarm!=NULL)
        {
            angle[0] = cmd_arm.position[0];
            angle[1] = cmd_arm.position[1];
            angle[2] = cmd_arm.position[2];
            angle[3] = cmd_arm.position[3];
            angle[4] = cmd_arm.position[4];
            angle[5] = cmd_arm.position[5];
            if(torque_status)
            {
                pSDKarm->SendArmAllServerCB(angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]);			//插补
                arm_set_gripper_linear_position(cmd_arm.position[6]*2);
                //pSDKarm->SendArmAllServer(angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]);
                //ROS_INFO("[%f,%f,%f,%f,%f,%f]", angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]);
            }
        }

    }

    void SagittariusArmReal::arm_joint_trajectory_msg_callback(const trajectory_msgs::JointTrajectory &msg)
    {
        if (execute_joint_traj == false)
        {
            std::map<std::string, uint8_t> joint_order;
            uint8_t cntr = 0;

            for (auto const& name:msg.joint_names)
            {
                joint_order[name] = cntr;
                cntr++;
            }
            jnt_tra_msg.joint_names.clear();
            jnt_tra_msg.points.clear();
            jnt_tra_msg.header = msg.header;
            for (auto const& joint:arm_joints)
                if (joint.name != "joint_gripper_left")
                    jnt_tra_msg.joint_names.push_back(joint.name);

            cntr = 0;
            size_t pos_size = msg.points.at(0).positions.size();
            size_t vel_size = msg.points.at(0).velocities.size();
            size_t accel_size = msg.points.at(0).accelerations.size();
            while(cntr < msg.points.size())
            {
                trajectory_msgs::JointTrajectoryPoint jnt_tra_point_msg;
                jnt_tra_point_msg.time_from_start = msg.points.at(cntr).time_from_start;
                for (auto const& joint:arm_joints)
                {
                    if (joint.name != "joint_gripper_left")
                    {
                        ///ROS_WARN("position =%lf",msg.points.at(cntr).positions.at(joint_order[joint.name])) ;
                        if (pos_size != 0)
                            jnt_tra_point_msg.positions.push_back(msg.points.at(cntr).positions.at(joint_order[joint.name]));
                        if (vel_size != 0)
                            jnt_tra_point_msg.velocities.push_back(msg.points.at(cntr).velocities.at(joint_order[joint.name]));
                        if (accel_size != 0)
                            jnt_tra_point_msg.accelerations.push_back(msg.points.at(cntr).accelerations.at(joint_order[joint.name]));
                    }
                }
                jnt_tra_msg.points.push_back(jnt_tra_point_msg);
                cntr++;
            }
            size_t itr = 0;
            joint_states = pSDKarm->joint_states;
            for (auto const& joint:arm_joints)
            {
                if (joint.name != "joint_gripper_left")
                {
                    //ROS_WARN("joint.name != gripper,positions.at(%d)=%d",itr, jnt_tra_msg.points[0].positions.at(itr));
                    if (!(fabs(jnt_tra_msg.points[0].positions.at(itr) - joint_states.position.at(itr)) < 0.1))
                    {
                        ROS_WARN("%s Servo is not at the correct initial state.", joint_states.name.at(itr).c_str());
                        ROS_WARN("Expected state: %f, Actual State: %f.", jnt_tra_msg.points.at(0).positions.at(itr), joint_states.position.at(itr));
                    }
                    itr++;
                }
            }
            ROS_INFO("Succeeded to get joint trajectory!");
            joint_start_time = ros::Time::now().toSec();
            execute_joint_traj = true;
        }
        else
        {
            ROS_WARN("Arm joints are still moving");
        }
    }


    void SagittariusArmReal::arm_joint_trajectory_action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
    {
        control_msgs::FollowJointTrajectoryResult result;
        if (goal->trajectory.points.size() < 2)
        {
            result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
            joint_action_server->setSucceeded(result);
            return;
        }
        arm_joint_trajectory_msg_callback(goal->trajectory);
        ros::Rate r(100);
        while (execute_joint_traj)
        {
            if (joint_action_server->isPreemptRequested())
            {
                execute_joint_traj = false;
                joint_action_server->setPreempted();
                ROS_INFO("Joint trajectory server preempted by client");
                return;
            }
            r.sleep();
        }
        result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
        joint_action_server->setSucceeded(result);
        //ROS_WARN("RobotArm::arm_joint_trajectory_action_callback go out");
    }

    void SagittariusArmReal::arm_gripper_trajectory_action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
    {
        control_msgs::FollowJointTrajectoryResult result;
        //ROS_WARN("arm_gripper_trajectory_action_callback");
        if (goal->trajectory.points.size() < 2)
        {
            result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
            gripper_action_server->setSucceeded(result);
            return;
        }
        ROS_WARN("arm_gripper_trajectory_msg_callback");
        arm_gripper_trajectory_msg_callback(goal->trajectory);
        ros::Rate r(100);
        while (execute_gripper_traj)
        {
            if (gripper_action_server->isPreemptRequested())
            {
                execute_gripper_traj = false;
                gripper_action_server->setPreempted();
                ROS_INFO("Gripper trajectory server preempted by client");
                return;
            }
            r.sleep();
        }
        result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
        //ROS_INFO("gripper_action_server setSucceeded");
        gripper_action_server->setSucceeded(result);
    }

    void SagittariusArmReal::arm_gripper_trajectory_msg_callback(const trajectory_msgs::JointTrajectory &msg)
    {
        if (execute_gripper_traj == false)
        {
            gripper_tra_msg.joint_names.clear();
            gripper_tra_msg.points.clear();
            gripper_tra_msg = msg;
            if (!(fabs(gripper_tra_msg.points.at(0).positions.at(0) - joint_states.position.at(arm_joints.size())) < 0.1))
            {
                ROS_WARN("Gripper Servo is not at the correct initial state.");
                ROS_WARN("Expected state: %f, Actual State: %f.", gripper_tra_msg.points.at(0).positions.at(0), joint_states.position.at(arm_joints.size()));
            }

            ROS_INFO("Succeeded to get gripper trajectory!");
            gripper_start_time = ros::Time::now().toSec();
            execute_gripper_traj = true;
        }
        else
        {
            ROS_WARN("Gripper is still moving");
        }
    }
    /// @brief arm_execute_joint_trajectory 关节的轨迹跟踪定时器
    /// @param - TimerEvent 事件定时器参数
    void SagittariusArmReal::arm_execute_joint_trajectory(const ros::TimerEvent&)
    {
        static uint8_t cntr = 0;
        static int flag = 1;
        if (!execute_joint_traj)
        {
            if (cntr != 0)
            {
                ROS_INFO("Joint Trajectory stopped.");
                cntr = 0;
            }
            flag = 1;
            return;
        }
        int traj_size = jnt_tra_msg.points.size();
        double time_now = ros::Time::now().toSec() - joint_start_time;
        double time_from_start = jnt_tra_msg.points[cntr].time_from_start.toSec();
        //ROS_WARN("now %lf,tfs %lf",time_now, time_from_start);
        if(servo_control_trajectory)
        {

            if(flag)
            {
                flag = 0;
                uint8_t counter = 0;
                double pos_array[joint_num_write];
                //printf("pos_array:");
                for (auto const& pos:jnt_tra_msg.points[traj_size-1].positions)
                {
                    pos_array[counter] = pos;
                    //printf("=%lf ",pos);
                    counter++;
                }
                //printf("\n");
                arm_set_joint_positions(pos_array, 4.0);

                //arm_set_joint_positions(pos_array, (jnt_tra_msg.points[traj_size-1].time_from_start.toSec()-jnt_tra_msg.points[0].time_from_start.toSec()));
            }
            if (time_now > time_from_start)
            {
                cntr++;
                if (cntr < traj_size)
                {
                }
                else
                {
                    ROS_INFO("Trajectory done being executed.");
                    execute_joint_traj = false;
                    cntr = 0;
                }
            }
        }
        else
        {

            if(time_from_start>0)
                time_from_start = time_from_start;//-0.02;

            if (time_now > time_from_start)
            {
                cntr++;
                if (cntr < traj_size)
                {
                    //ROS_WARN("--now %lf,tfs %lf",time_now, time_from_start);
                    if (1)
                    {
                        uint8_t counter = 0;
                        double pos_array[joint_num_write];
                        //printf("pos_array:");
                        for (auto const& pos:jnt_tra_msg.points[cntr].positions)
                        {
                            pos_array[counter] = pos;
                            //printf("=%lf ",pos);
                            counter++;
                        }
                        //printf("\n");
                        arm_set_joint_positions(pos_array, (jnt_tra_msg.points[cntr].time_from_start.toSec()-jnt_tra_msg.points[cntr-1].time_from_start.toSec()));
                        //ROS_WARN("now %lf,tfs %lf",time_now, time_from_start);

                    }
                }
                else
                {
                    ROS_INFO("Trajectory done being executed.");
                    execute_joint_traj = false;
                    cntr = 0;
                }
            }
        }
    }

    /// @brief arm_calculate_gripper_degree_position 距离转换成角度
    /// @param dist - 夹爪的距离值
    short SagittariusArmReal::arm_calculate_gripper_degree_position(const float dist)
    {
        double half_dist = dist / 2.0;
        short result = -(3462*half_dist)*10;
        return result;
    }
    /// @brief arm_set_gripper_linear_position 设置夹爪的位置
    /// @param dist - 夹爪的距离值
    void SagittariusArmReal::arm_set_gripper_linear_position(const float dist)
    {
        short g_degree = arm_calculate_gripper_degree_position(dist);
        //printf("degree is %d\n",g_degree);
        arm_set_single_joint_degree_position(g_degree);
    }
    /// @brief arm_set_single_joint_degree_position 发送单个关节的角度。
    /// @param g_degree - 角度值
    void SagittariusArmReal::arm_set_single_joint_degree_position(short g_degree)
    {
        if(torque_status)
        {
            pSDKarm->SendArmEndAction(0,g_degree);
        }
    }
    /// @brief arm_execute_gripper_trajectory    夹爪的轨迹跟踪定时器
    /// @param - TimerEvent 事件定时器参数
    void SagittariusArmReal::arm_execute_gripper_trajectory(const ros::TimerEvent&)
    {
        static uint8_t cntr = 0;
        if (!execute_gripper_traj)
        {
            if (cntr != 0)
            {
                ROS_INFO("Gripper Trajectory stopped.");
                cntr = 0;
            }
            return;
        }
        int traj_size = gripper_tra_msg.points.size();
        double time_now = ros::Time::now().toSec() - gripper_start_time;
        double time_from_start = gripper_tra_msg.points.at(cntr).time_from_start.toSec();
        if (time_now > time_from_start)
        {
            while (time_now > time_from_start && cntr < (traj_size - 1))
            {
                cntr++;
                time_from_start = gripper_tra_msg.points.at(cntr).time_from_start.toSec();
            }
            if (cntr < (traj_size - 1))
            {
                arm_set_gripper_linear_position(gripper_tra_msg.points.at(cntr).positions.at(0)*2.0);
            }
            else
            {
                arm_set_gripper_linear_position(gripper_tra_msg.points.at(cntr).positions.at(0)*2.0);
                ROS_INFO("Trajectory done being executed.");
                execute_gripper_traj = false;
                cntr = 0;
            }
        }
    }

    /// @brief arm_set_joint_positions - 控制机械臂各个舵机的移动到目标位置
    /// @param joint_positions - 舵机的目标弧度; diff_time - 用时
    void SagittariusArmReal::arm_set_joint_positions(const double joint_positions[], double diff_time)
    {
        if(pSDKarm!=NULL)
        {
            angle[0] = joint_positions[0];
            angle[1] = joint_positions[1];
            angle[2] = joint_positions[2];
            angle[3] = joint_positions[3];
            angle[4] = joint_positions[4];
            angle[5] = joint_positions[5];
            short difftime = diff_time*1000;
            if(torque_status)
            {
                //pSDKarm->SendArmAllServerCB(angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]);
                pSDKarm->SendArmAllServerTime(difftime, angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]);
                //ROS_INFO("[%f,%f,%f,%f,%f,%f]", angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]);
            }

        }
    }
    /// @brief arm_write_joint_commands - 处理订阅到的机械臂控制命令。并控制机械臂各个舵机的移动到目标位置
    /// @param msg - 自定义消息。机械臂的位置控制。
    void SagittariusArmReal::arm_write_joint_commands(const sdk_sagittarius_arm::ArmRadControl &msg)
    {
        double joint_positions[msg.rad.size()];
        for (size_t i {0}; i < msg.rad.size(); i++)
            joint_positions[i] = msg.rad.at(i);
        arm_set_joint_positions(joint_positions, 0.0);
    }
    /// @brief arm_write_gripper_command - 控制机械臂末端舵机的移动长度
    /// @param msg -
    void SagittariusArmReal::arm_write_gripper_command(const std_msgs::Float64 &msg)
    {
        arm_set_gripper_linear_position(msg.data*2.0);
    }

}








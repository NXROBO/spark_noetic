/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2021, NXROBO.
 * All rights reserved.
 * Author: litian.zhuang  <litian.zhuang@nxrobo.com>   
 */
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_parser.h>
#include <ros/ros.h>
#include <stdio.h>

namespace sdk_sagittarius_arm
{
    CSDKarmParser::CSDKarmParser() :
        CParserBase()
    {
        // Do Nothing...
    }

    CSDKarmParser::~CSDKarmParser()
    {
        // Do Nothing...
    }

    int CSDKarmParser::Parse(char *data, size_t data_length, SDKSagittariusArmConfig &config, sensor_msgs::LaserScan &msg)
    {

        return ExitSuccess;
    }
} /*namespace sdk_sagittarius_arm*/

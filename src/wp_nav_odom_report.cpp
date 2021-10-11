/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/* @author Zhang Wanjie                                             */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
extern "C" {
#include "UDPClient.h"
}

static int Robot_ID = 1;    //机器人ID

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wp_nav_odom_report");
    InitUDPClient((char*)"192.168.1.110", 20180); //服务器IP地址
    
    ros::NodeHandle nh;
    ros::Rate r(10.0);

    tf::TransformListener listener;
    tf::StampedTransform transform;

    while(ros::ok())
    {
        try
        {
            listener.waitForTransform("map","base_footprint",  ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform("map","base_footprint", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) 
        {
            ROS_ERROR("[lookupTransform] %s",ex.what());
            break;
        }
        float tx = transform.getOrigin().x();
        float ty = transform.getOrigin().y();
        tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(transform.getRotation() , tf::Point(tx, ty, 0.0)), ros::Time::now(), "map");
        geometry_msgs::PoseStamped robot_pos;
        tf::poseStampedTFToMsg(p, robot_pos);

        tf::Quaternion quat(robot_pos.pose.orientation.x,robot_pos.pose.orientation.y,robot_pos.pose.orientation.z,robot_pos.pose.orientation.w);
        double roll,pitch,yaw;
        tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
        ROS_WARN("[Robot Pos]( %.2f , %.2f ) - %.2f" , robot_pos.pose.position.x, robot_pos.pose.position.y, yaw);

        SendRobotState(Robot_ID, 1, robot_pos.pose.position.x, robot_pos.pose.position.y, yaw);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
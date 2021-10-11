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
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <waterplus_map_tools/Waypoint.h>
#include <waterplus_map_tools/GetNumOfWaypoints.h>
#include <waterplus_map_tools/GetWaypointByIndex.h>
#include <waterplus_map_tools/GetWaypointByName.h>
#include <std_msgs/String.h>
#include <string>
extern "C" {
#include "UDPServer.h"
}

static tf::StampedTransform transform;
static ST_Ctrl st_ctrl;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

static ros::Publisher behaviors_pub;
static std_msgs::String behavior_msg;

void GrabResultCB(const std_msgs::String::ConstPtr &msg)
{
    ROS_WARN("[GrabResultCB] %s",msg->data.c_str());
}

void PassResultCB(const std_msgs::String::ConstPtr &msg)
{
    ROS_WARN("[PassResultCB] %s",msg->data.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wp_nav_remote");

    InitUDPServer(20181); //本地监听端口

    ros::NodeHandle nh;
    ros::ServiceClient cliGetNum = nh.serviceClient<waterplus_map_tools::GetNumOfWaypoints>("waterplus/get_num_waypoint");
    ros::ServiceClient cliGetWPIndex = nh.serviceClient<waterplus_map_tools::GetWaypointByIndex>("waterplus/get_waypoint_index");
    ros::ServiceClient cliGetWPName = nh.serviceClient<waterplus_map_tools::GetWaypointByName>("waterplus/get_waypoint_name");
    behaviors_pub = nh.advertise<std_msgs::String>("wpr1/behaviors", 30);
    ros::Subscriber res_grab = nh.subscribe("wpr1/grab_result", 30, GrabResultCB);
    ros::Subscriber res_pass = nh.subscribe("wpr1/pass_result", 30, PassResultCB);

    ///////////////////////////////////////////////////////////////////////////////////
    //把航点名称都列出来
    waterplus_map_tools::GetNumOfWaypoints srvNum;
    if (cliGetNum.call(srvNum))
    {
        ROS_INFO("Num_wp = %d", (int)srvNum.response.num);
    }
    else
    {
        ROS_ERROR("Failed to call service get_num_waypoints");
    }
    waterplus_map_tools::GetWaypointByIndex srvI;
    for(int i=0;i<srvNum.response.num;i++)
    {
        srvI.request.index = i;
        if (cliGetWPIndex.call(srvI))
        {
            std::string name = srvI.response.name;
            float x = srvI.response.pose.position.x;
            float y = srvI.response.pose.position.y;
            ROS_INFO("Get_wp_index: name = %s (%.2f,%.2f)", name.c_str(),x,y);
        }
        else
        {
            ROS_ERROR("Failed to call service get_wp_index");
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////
    // waterplus_map_tools::GetWaypointByName srvN;
    // for(int i=0;i<10;i++)
    // {
    //     std::ostringstream stringStream;
    //     stringStream << i;
    //     std::string wp_index = stringStream.str();
    //     srvN.request.name = wp_index;
    //     if (cliGetWPName.call(srvN))
    //     {
    //         std::string name = srvN.response.name;
    //         float x = srvN.response.pose.position.x;
    //         float y = srvN.response.pose.position.y;
    //         ROS_INFO("Get_wp_name: name = %s (%.2f,%.2f)", wp_index.c_str(),x,y);
    //     }
    //     else
    //     {
    //         ROS_ERROR("Failed to call service get_waypoint_name");
    //     }
    // }
    ////////////////////////////////////////////////////////////////////////////////////

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        if(!ros::ok())
            break;
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    int nNumOfWaypoints = 0;
    move_base_msgs::MoveBaseGoal goal;
    ros::Rate r(1.0);
    while(ros::ok())
    {
        // 等待接收指令
        bool bNewCmd = GetCtrlCmd(&st_ctrl);
        if(bNewCmd == true)
        {
            if(st_ctrl.ctrl == CTRL_MOVETO_NAME)
            {
                // 根据接收到的航点名称查询航点信息
                waterplus_map_tools::GetWaypointByName srvN;
                
                std::ostringstream stringStream;
                stringStream << st_ctrl.wp_name;
                std::string wp_name = stringStream.str();
                srvN.request.name = wp_name;
                if (cliGetWPName.call(srvN))
                {
                    std::string name = srvN.response.name;
                    float x = srvN.response.pose.position.x;
                    float y = srvN.response.pose.position.y;
                    ROS_INFO("Get_wp_name: name = %s (%.2f,%.2f)", wp_name.c_str(),x,y);

                    // 开始导航
                    ROS_WARN("Go to the WayPoint[%s]",wp_name.c_str());
                    goal.target_pose.header.frame_id = "map";
                    goal.target_pose.header.stamp = ros::Time::now();
                    goal.target_pose.pose = srvN.response.pose;
                    ac.sendGoal(goal);
                    ac.waitForResult();
                    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        ROS_INFO("Arrived at WayPoint[%s] !",wp_name.c_str());
                    }
                    else
                        ROS_INFO("Failed to get to WayPoint[%s] ...",wp_name.c_str() );
                }
                else
                {
                    ROS_ERROR("Failed to call service get_waypoint_name");
                }
            }
            if(st_ctrl.ctrl == CTRL_MOVETO_POS)
            {
                // 根据接收到的目标点进行导航
                ROS_INFO("Go to the WayPoint( %.2f , %.2f ) %.2f",st_ctrl.x,st_ctrl.y,st_ctrl.angle);

                // 开始导航
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose.position.x = st_ctrl.x;
                goal.target_pose.pose.position.y = st_ctrl.y;
                float angle = st_ctrl.angle*3.14159/180;
                tf::Quaternion quat;
                // 目标姿态,函数三个参数分别为滚转,俯仰和偏转角,单位为弧度
                quat.setRPY(0.0, 0.0, angle);
                // 将欧拉角旋转量转换成四元数表达
                transform.setRotation(quat);
                goal.target_pose.pose.orientation.x = transform.getRotation().getX();
                goal.target_pose.pose.orientation.y = transform.getRotation().getY();
                goal.target_pose.pose.orientation.z = transform.getRotation().getZ();
                goal.target_pose.pose.orientation.w = transform.getRotation().getW();
                ac.sendGoal(goal);
                ac.waitForResult();
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Arrived at WayPoint( %.2f , %.2f ) %.2f",st_ctrl.x,st_ctrl.y,st_ctrl.angle);
                }
                else
                    ROS_INFO("Failed to get to WayPoint( %.2f , %.2f ) %.2f",st_ctrl.x,st_ctrl.y,st_ctrl.angle );
            }
            if(st_ctrl.ctrl == CTRL_GRAB)
            {
                behavior_msg.data = "grab start";
                behaviors_pub.publish(behavior_msg);
            }
            if(st_ctrl.ctrl == CTRL_PASS)
            {
                behavior_msg.data = "pass start";
                behaviors_pub.publish(behavior_msg);
            }
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
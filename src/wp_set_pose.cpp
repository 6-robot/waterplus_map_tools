/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2025-2035, Waterplus http://www.6-robot.com
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
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include "waterplus_map_tools/GetWaypointByName.h"

// 全局变量，用于发布者和服务客户端
ros::Publisher initial_pose_pub;
ros::ServiceClient client;

// 回调函数，处理来自 /waterplus/set_pose 话题的消息
void setPoseCallback(const std_msgs::String::ConstPtr& msg)
{
    std::string waypoint_name = msg->data;
    ROS_INFO("接收到航点名称: %s", waypoint_name.c_str());

    // 准备并发送服务请求
    waterplus_map_tools::GetWaypointByName srv;
    srv.request.name = waypoint_name;

    if (client.call(srv))
    {
        // 构建 geometry_msgs/PoseWithCovarianceStamped 消息
        geometry_msgs::PoseWithCovarianceStamped initial_pose_msg;
        initial_pose_msg.header.stamp = ros::Time::now();
        initial_pose_msg.header.frame_id = "map"; // 初始位姿通常在 "map" 坐标系下

        // 填充从服务获取的位姿
        initial_pose_msg.pose.pose = srv.response.pose;

        // 填充协方差矩阵 (covariance)
        initial_pose_msg.pose.covariance[0] = 0.25;  // x的方差 (0.5m^2)
        initial_pose_msg.pose.covariance[7] = 0.25;  // y的方差 (0.5m^2)
        initial_pose_msg.pose.covariance[35] = 0.0685; // yaw角的方差 (约0.26弧度^2)

        // 稍作等待，确保发布者已经连接到订阅者
        ros::Duration(1.0).sleep();

        // 发布消息
        initial_pose_pub.publish(initial_pose_msg);

        ROS_WARN("将机器人定位到航点位置: %s", waypoint_name.c_str());
    }
    else
    {
        ROS_ERROR("调用 /waterplus/get_waypoint_name 服务失败！ 航点 '%s' 是否未设置？", waypoint_name.c_str());
    }
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    // 初始化ROS节点
    ros::init(argc, argv, "wp_set_pose");
    ros::NodeHandle nh;

    // 创建到 /initialpose 话题的发布者
    initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);

    // 创建服务客户端，并等待服务启动
    client = nh.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    ROS_INFO("等待服务 '/waterplus/get_waypoint_name' 启动...");
    client.waitForExistence(); // 等待服务启动

    // 创建 /waterplus/set_pose 话题的订阅者
    ros::Subscriber sub = nh.subscribe("/waterplus/set_pose", 1, setPoseCallback);

    ROS_INFO("节点已准备就绪，等待来自'/waterplus/set_pose'的航点名称...");

    // 保持节点运行，处理回调
    ros::spin();

    return 0;
}
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
#include "waterplus_map_tools/GetWaypointByName.h"

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    // 初始化ROS节点
    ros::init(argc, argv, "set_pose_from_waypoint_node");
    ros::NodeHandle nh;

    // 1. 从命令行参数获取航点名称
    std::string waypoint_name;
    if (argc < 2)
    {
        return 1;
    }
    waypoint_name = argv[1]; // argv[0] 是程序名, argv[1] 是第一个参数

    // 2. 创建服务客户端，并等待服务启动
    ros::ServiceClient client = nh.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    ROS_INFO("等待服务 '/waterplus/get_waypoint_name' 启动...");
    client.waitForExistence(); // 等待服务启动

    // 3. 准备并发送服务请求
    waterplus_map_tools::GetWaypointByName srv;
    srv.request.name = waypoint_name;

    if (client.call(srv))
    {
        // 4. 创建到 /initialpose 话题的发布者
        ros::Publisher initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);

        // 5. 构建 geometry_msgs/PoseWithCovarianceStamped 消息
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
        
        // 6. 发布消息
        initial_pose_pub.publish(initial_pose_msg);

        ROS_WARN("自动定位机器人到航点位置: %s", waypoint_name.c_str());
    }
    else
    {
        ROS_ERROR("调用 /waterplus/get_waypoint_name 服务失败！ 航点 '%s' 是否未设置？", waypoint_name.c_str());
        return 1;
    }

    // 给发布足够的时间，并处理任何待处理的ROS回调
    ros::spinOnce();
    ros::Duration(1.0).sleep();

    return 0;
}
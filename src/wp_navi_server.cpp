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
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <waterplus_map_tools/Waypoint.h>
#include <waterplus_map_tools/GetWaypointByName.h>

static bool bNewCmd = false;
ros::ServiceClient cliGetWPName;
static geometry_msgs::Pose wp_pose;
static ros::Publisher result_pub;
static std_msgs::String result_msg;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void NaviWaypointCB(const std_msgs::String::ConstPtr &msg)
{
    waterplus_map_tools::GetWaypointByName srvN;
    srvN.request.name = msg->data;
    if (cliGetWPName.call(srvN))
    {
        wp_pose = srvN.response.pose;
        std::string name = srvN.response.name;
        float x = srvN.response.pose.position.x;
        float y = srvN.response.pose.position.y;
        ROS_INFO("Get_wp_name: name = %s (%.2f,%.2f)", name.c_str(),x,y);
    }
    else
    {
        ROS_ERROR("Failed to call service get_waypoint_name");
    }
    bNewCmd = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wp_navi_server");

    ros::NodeHandle n;
    ros::Subscriber navi_name_sub = n.subscribe("waterplus/navi_waypoint", 10, NaviWaypointCB);
    result_pub = n.advertise<std_msgs::String>("waterplus/navi_result", 10);
    cliGetWPName = n.serviceClient<waterplus_map_tools::GetWaypointByName>("waterplus/get_waypoint_name");

    MoveBaseClient ac("move_base", true);
    move_base_msgs::MoveBaseGoal goal;

    ros::Rate r(30);

    while(ros::ok())
    {
        if(bNewCmd)
        {
            //wait for the action server to come up
            while(!ac.waitForServer(ros::Duration(5.0)))
            {
                if(!ros::ok())
                    break;
                ROS_INFO("Waiting for the move_base action server to come up");
            }
            
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose = wp_pose;
            ac.sendGoal(goal);
            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Arrived at WayPoint !");
                result_msg.data = "done";
            }
            else
            {
                ROS_WARN("Failed to get to WayPoint ..." );
                result_msg.data = "failure";
            }
            result_pub.publish(result_msg);
            bNewCmd = false;
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
    }
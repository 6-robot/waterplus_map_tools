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
static geometry_msgs::Pose goal_pose;
static ros::Publisher result_pub;
static std_msgs::String result_msg;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void NaviPoseCB(const geometry_msgs::Pose::ConstPtr &msg)
{
    goal_pose = *msg;
    bNewCmd = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_navi_server");

    ros::NodeHandle n;
    ros::Subscriber navi_name_sub = n.subscribe("waterplus/navi_pose", 10, NaviPoseCB);
    result_pub = n.advertise<std_msgs::String>("waterplus/navi_result", 10);

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
            goal.target_pose.pose = goal_pose;
            ac.sendGoal(goal);
            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Arrived at Pose ( %.2f , %.2f ) !",goal_pose.position.x, goal_pose.position.y);
            }
            else
                ROS_WARN("Failed to get to WayPoint ( %.2f , %.2f )..." ,goal_pose.position.x, goal_pose.position.y);
            result_msg.data = "done";
            result_pub.publish(result_msg);
            bNewCmd = false;
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
    }
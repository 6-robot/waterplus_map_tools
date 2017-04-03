
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"
#include "add_waypoint_tool.h"

namespace rviz
{
    AddWayPointTool::AddWayPointTool()
    {
        shortcut_key_ = 'a';
        topic_property_ = new StringProperty( "Topic", "/waterplus/add_waypoint","The topic on which to add new waypoints.",getPropertyContainer(), SLOT( updateTopic() ), this );
    }

    AddWayPointTool::~AddWayPointTool()
    {
    }

    void AddWayPointTool::onInitialize()
    {
        PoseTool::onInitialize();
        setName( "Add WayPoint" );
        updateTopic();
    }

    void AddWayPointTool::updateTopic()
    {
        pub_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_property_->getStdString(), 1);
    }

    void AddWayPointTool::onPoseSet(double x, double y, double theta)
    {
        std::string fixed_frame = context_->getFixedFrame().toStdString();
        tf::Quaternion quat;
        quat.setRPY(0.0, 0.0, theta);
        tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
        geometry_msgs::PoseStamped new_waypoint;
        tf::poseStampedTFToMsg(p, new_waypoint);
        ROS_INFO("Add new waypoint: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
            new_waypoint.pose.position.x, new_waypoint.pose.position.y, new_waypoint.pose.position.z,
            new_waypoint.pose.orientation.x, new_waypoint.pose.orientation.y, new_waypoint.pose.orientation.z, new_waypoint.pose.orientation.w, theta);
        pub_.publish(new_waypoint);
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::AddWayPointTool,rviz::Tool)


#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <waterplus_map_tools/Waypoint.h>
#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"
#include "add_waypoint_tool.h"

static int nWaypointCount = 0;

namespace rviz
{
    AddWaypointTool::AddWaypointTool()
    {
        shortcut_key_ = 'a';
        topic_property_ = new StringProperty( "Topic", "/waterplus/add_waypoint","The topic on which to add new waypoints.",getPropertyContainer(), SLOT( updateTopic() ), this );
    }

    AddWaypointTool::~AddWaypointTool()
    {
    }

    void AddWaypointTool::onInitialize()
    {
        PoseTool::onInitialize();
        setName( "Add Waypoint" );
        updateTopic();
    }

    void AddWaypointTool::updateTopic()
    {
        pub_ = nh_.advertise<waterplus_map_tools::Waypoint>( topic_property_->getStdString(), 1);
    }

    void AddWaypointTool::onPoseSet(double x, double y, double theta)
    {
        std::string fixed_frame = context_->getFixedFrame().toStdString();
        tf::Quaternion quat;
        quat.setRPY(0.0, 0.0, theta);
        tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
        geometry_msgs::PoseStamped new_pos;
        tf::poseStampedTFToMsg(p, new_pos);
        ROS_INFO("Add new waypoint: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
            new_pos.pose.position.x, new_pos.pose.position.y, new_pos.pose.position.z,
            new_pos.pose.orientation.x, new_pos.pose.orientation.y, new_pos.pose.orientation.z, new_pos.pose.orientation.w, theta);
        waterplus_map_tools::Waypoint new_waypoint;

        nWaypointCount ++;
        std::ostringstream stringStream;
        stringStream << nWaypointCount;
        new_waypoint.name = stringStream.str();

        new_waypoint.pose = new_pos.pose;
        pub_.publish(new_waypoint);
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::AddWaypointTool,rviz::Tool)

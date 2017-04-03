#include <tinyxml.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

static std::vector <move_base_msgs::MoveBaseGoal> arWayPoint;
static ros::Publisher marker_pub;
static visualization_msgs::Marker marker_waypoints;
static visualization_msgs::Marker text_marker;

void Init_WayPoints()
{
    move_base_msgs::MoveBaseGoal newWayPoint;
    newWayPoint.target_pose.header.frame_id = "map";
    newWayPoint.target_pose.pose.position.x = 0.0;
    newWayPoint.target_pose.pose.position.y = 0.0;
    newWayPoint.target_pose.pose.orientation.w = 1.0;
    arWayPoint.push_back(newWayPoint);

    newWayPoint.target_pose.pose.position.x = 1.0;
    newWayPoint.target_pose.pose.position.y = 0.0;
    newWayPoint.target_pose.pose.orientation.w = 1.0;
    arWayPoint.push_back(newWayPoint);


    newWayPoint.target_pose.pose.position.x = 1.0;
    newWayPoint.target_pose.pose.position.y = 1.0;
    newWayPoint.target_pose.pose.orientation.z = 1.0;
    arWayPoint.push_back(newWayPoint);

}

void Init_Marker()
{
    marker_waypoints.header.frame_id = "map";
    marker_waypoints.ns = "marker_waypoints";
    marker_waypoints.action = visualization_msgs::Marker::ADD;
    marker_waypoints.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_waypoints.mesh_resource = "package://waterplus_map_tools/meshes/waypoint.dae";
    marker_waypoints.scale.x = 1;
    marker_waypoints.scale.y = 1;
    marker_waypoints.scale.z = 1;
    marker_waypoints.color.r = 0;
    marker_waypoints.color.g = 0.5;
    marker_waypoints.color.b = 0.5;
    marker_waypoints.color.a = 1.0;
}

void DrawTextMarker(std::string inText, int inID, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB);
void PublishWaypointsMarker()
{
    int nNumWP = arWayPoint.size();
    for(int i=0; i<nNumWP ; i++ )
    {
        marker_waypoints.id = i;
        marker_waypoints.pose.position.x = arWayPoint[i].target_pose.pose.position.x;
        marker_waypoints.pose.position.y = arWayPoint[i].target_pose.pose.position.y;
        marker_waypoints.pose.position.z = -0.1;
        marker_waypoints.pose.orientation = arWayPoint[i].target_pose.pose.orientation;
        marker_pub.publish(marker_waypoints);
        ros::spinOnce();

        // text_marker.pose.position.x = arWayPoint[i].target_pose.pose.position.x;
        // text_marker.pose.position.y = arWayPoint[i].target_pose.pose.position.y;
        // text_marker.pose.position.z = 0;
        // text_marker.pose.orientation = arWayPoint[i].target_pose.pose.orientation;

        float wp_x = arWayPoint[i].target_pose.pose.position.x;
        float wp_y = arWayPoint[i].target_pose.pose.position.y;

        std::ostringstream stringStream;
        stringStream << "wp_" << i;
        std::string face_id = stringStream.str();
        DrawTextMarker(face_id,i,0.2,wp_x,wp_y,0.5,0,0.5,1.0);
        ros::spinOnce();
    }
}

void DrawTextMarker(std::string inText, int inID, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB)
{
    text_marker.header.frame_id = "map";
    text_marker.ns = "line_obj";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.id = inID;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = inScale;
    text_marker.color.r = inR;
    text_marker.color.g = inG;
    text_marker.color.b = inB;
    text_marker.color.a = 1.0;

    text_marker.pose.position.x = inX;
    text_marker.pose.position.y = inY;
    text_marker.pose.position.z = inZ;
    
    text_marker.pose.orientation=tf::createQuaternionMsgFromYaw(1.0);

    text_marker.text = inText;

    marker_pub.publish(text_marker);
}


void AddWayPointCallback(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
    ROS_WARN("[add_waypoint] new waypoint (%.2f %.2f) (%.2f %.2f %.2f %.2f) ", pos->pose.position.x, pos->pose.position.y, pos->pose.orientation.x, pos->pose.orientation.y, pos->pose.orientation.z, pos->pose.orientation.w);
    move_base_msgs::MoveBaseGoal newWayPoint;
    newWayPoint.target_pose.header.frame_id = "map";
    newWayPoint.target_pose.pose = pos->pose;
    arWayPoint.push_back(newWayPoint);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "wp_waypoint_manager");

    ros::NodeHandle nh;
    ros::Subscriber add_waypoint_sub = nh.subscribe("/waterplus/add_waypoint",10,&AddWayPointCallback);
    marker_pub = nh.advertise<visualization_msgs::Marker>("waypoints_marker", 100);
    Init_WayPoints();
    Init_Marker();

    ros::Rate r(10);

    while(ros::ok())
    {
        PublishWaypointsMarker();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
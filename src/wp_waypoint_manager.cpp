#include <tinyxml.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <waterplus_map_tools/Waypoint.h>
#include <waterplus_map_tools/GetNumOfWaypoints.h>
#include <waterplus_map_tools/GetWaypointByIndex.h>
#include <waterplus_map_tools/GetWaypointByName.h>
#include <string>

static std::vector <waterplus_map_tools::Waypoint> arWaypoint;
static ros::Publisher marker_pub;
static visualization_msgs::Marker marker_waypoints;
static visualization_msgs::Marker text_marker;

bool getNumOfWaypoints(waterplus_map_tools::GetNumOfWaypoints::Request &req, waterplus_map_tools::GetNumOfWaypoints::Response &res)
{
    res.num = arWaypoint.size();
    ROS_INFO("Get_num_wp: num_wp = %d", res.num);
    return true;
}

bool getWaypointByIndex(waterplus_map_tools::GetWaypointByIndex::Request &req, waterplus_map_tools::GetWaypointByIndex::Response &res)
{
    int nIndex = req.index;
    int nNumWP = arWaypoint.size();
    if(nIndex >= 0 && nIndex < nNumWP)
    {
        res.name = arWaypoint[nIndex].name;
        res.pose = arWaypoint[nIndex].pose;
        ROS_INFO("Get_wp_index: name = %s", arWaypoint[nIndex].name.c_str());
        return true;
    }
    else
    {
        ROS_INFO("Get_wp_index: failed! index = %d , num_wp = %d", nIndex , nNumWP);
        return false;
    }
}

bool getWaypointByName(waterplus_map_tools::GetWaypointByName::Request &req, waterplus_map_tools::GetWaypointByName::Response &res)
{
    std::string reqName = req.name;
    int nNumWP = arWaypoint.size();
    bool bResultGetWP = false;
    for(int i=0;i<nNumWP;i++)
    {
        std::size_t found = arWaypoint[i].name.find(reqName); 
        if(found != std::string::npos)
        {
            res.name = arWaypoint[i].name;
            res.pose = arWaypoint[i].pose;
            bResultGetWP = true;
            break;
        }
    }
    if(bResultGetWP == true)
    {
        ROS_INFO("Get_wp_name: name = %s", res.name.c_str());
        return true;
    }
    else
    {
        ROS_INFO("Get_wp_name: failed! There is no waypoint name %s", reqName.c_str());
        return false;
    }
}

void Init_Waypoints()
{
    waterplus_map_tools::Waypoint newWayPoint;
    newWayPoint.name = "0";
    newWayPoint.pose.position.x = 0.0;
    newWayPoint.pose.position.y = 0.0;
    newWayPoint.pose.orientation.w = 1.0;
    arWaypoint.push_back(newWayPoint);
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
    marker_waypoints.color.r = 1.0;
    marker_waypoints.color.g = 0.0;
    marker_waypoints.color.b = 1.0;
    marker_waypoints.color.a = 1.0;
}

void DrawTextMarker(std::string inText, int inID, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB);
void PublishWaypointsMarker()
{
    int nNumWP = arWaypoint.size();
    for(int i=0; i<nNumWP ; i++ )
    {
        marker_waypoints.id = i;
        marker_waypoints.pose.position.x = arWaypoint[i].pose.position.x;
        marker_waypoints.pose.position.y = arWaypoint[i].pose.position.y;
        marker_waypoints.pose.position.z = -0.01;
        marker_waypoints.pose.orientation = arWaypoint[i].pose.orientation;
        marker_pub.publish(marker_waypoints);
        ros::spinOnce();

        float wp_x = arWaypoint[i].pose.position.x;
        float wp_y = arWaypoint[i].pose.position.y;

        std::string face_id = arWaypoint[i].name;
        DrawTextMarker(face_id,i,0.2,wp_x,wp_y,0.55,0,0.5,1.0);
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


void AddWayPointCallback(const waterplus_map_tools::Waypoint::ConstPtr& wp)
{
    ROS_INFO("[add_waypoint] %s (%.2f %.2f) (%.2f %.2f %.2f %.2f) ",wp->name.c_str(), wp->pose.position.x, wp->pose.position.y, wp->pose.orientation.x, wp->pose.orientation.y, wp->pose.orientation.z, wp->pose.orientation.w);
    waterplus_map_tools::Waypoint newWayPoint;
    newWayPoint = *wp;
    arWaypoint.push_back(newWayPoint);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "wp_waypoint_manager");

    ros::NodeHandle nh;
    ros::Subscriber add_waypoint_sub = nh.subscribe("/waterplus/add_waypoint",10,&AddWayPointCallback);
    marker_pub = nh.advertise<visualization_msgs::Marker>("waypoints_marker", 100);
    Init_Waypoints();
    Init_Marker();

    ros::ServiceServer srvGetNum = nh.advertiseService("/waterplus/get_num_waypoint", getNumOfWaypoints);
    ros::ServiceServer srvGetWPIndex = nh.advertiseService("/waterplus/get_waypoint_index", getWaypointByIndex);
    ros::ServiceServer srvGetWPName = nh.advertiseService("/waterplus/get_waypoint_name", getWaypointByName);

    ros::Rate r(10);

    while(ros::ok())
    {
        PublishWaypointsMarker();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
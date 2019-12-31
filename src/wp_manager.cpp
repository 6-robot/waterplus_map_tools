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

#include <tinyxml.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <waterplus_map_tools/Waypoint.h>
#include <waterplus_map_tools/GetNumOfWaypoints.h>
#include <waterplus_map_tools/GetWaypointByIndex.h>
#include <waterplus_map_tools/GetWaypointByName.h>
#include <waterplus_map_tools/SaveWaypoints.h>
#include <string>

static std::vector <waterplus_map_tools::Waypoint> arWaypoint;
static std::vector <waterplus_map_tools::Waypoint> arCharger;
static ros::Publisher marker_pub;
static ros::Publisher charger_pub;
static visualization_msgs::Marker marker_waypoints;
static visualization_msgs::Marker marker_chargers;
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

bool getChargerByName(waterplus_map_tools::GetWaypointByName::Request &req, waterplus_map_tools::GetWaypointByName::Response &res)
{
    std::string reqName = req.name;
    int nNumCh = arCharger.size();
    bool bResultGetCh = false;
    for(int i=0;i<nNumCh;i++)
    {
        std::size_t found = arCharger[i].name.find(reqName); 
        if(found != std::string::npos)
        {
            res.name = arCharger[i].name;
            res.pose = arCharger[i].pose;
            bResultGetCh = true;
            break;
        }
    }
    if(bResultGetCh == true)
    {
        ROS_INFO("Get_charger_name: name = %s", res.name.c_str());
        return true;
    }
    else
    {
        ROS_INFO("Get_charger_name: failed! There is no charger name %s", reqName.c_str());
        return false;
    }
}

bool SaveWaypointsToFile(std::string inFilename);
bool saveWaypoints(waterplus_map_tools::SaveWaypoints::Request &req, waterplus_map_tools::SaveWaypoints::Response &res)
{
    return SaveWaypointsToFile(req.filename);
}

std::string Flt2Str(float inVal)
{
    std::ostringstream stringStream;
    stringStream << inVal;
    std::string retStr = stringStream.str();
    return retStr;
}

std::string Int2Str(int inVal)
{
    std::ostringstream stringStream;
    stringStream << inVal;
    std::string retStr = stringStream.str();
    return retStr;
}

bool SaveWaypointsToFile(std::string inFilename)
{
    TiXmlDocument *docSave = new TiXmlDocument();
    TiXmlElement *RootElement = new TiXmlElement("Waterplus");
    docSave->LinkEndChild(RootElement);

    int nNumWP = arWaypoint.size();
    for(int i=0;i<nNumWP;i++)
    {
        TiXmlElement *WaypointElement = new TiXmlElement("Waypoint");
        WaypointElement->InsertEndChild(TiXmlElement("Name"))->InsertEndChild(TiXmlText(arWaypoint[i].name));
        WaypointElement->InsertEndChild(TiXmlElement("Pos_x"))->InsertEndChild(TiXmlText(Flt2Str(arWaypoint[i].pose.position.x)));
        WaypointElement->InsertEndChild(TiXmlElement("Pos_y"))->InsertEndChild(TiXmlText(Flt2Str(arWaypoint[i].pose.position.y)));
        WaypointElement->InsertEndChild(TiXmlElement("Pos_z"))->InsertEndChild(TiXmlText(Flt2Str(arWaypoint[i].pose.position.z)));
        WaypointElement->InsertEndChild(TiXmlElement("Ori_x"))->InsertEndChild(TiXmlText(Flt2Str(arWaypoint[i].pose.orientation.x)));
        WaypointElement->InsertEndChild(TiXmlElement("Ori_y"))->InsertEndChild(TiXmlText(Flt2Str(arWaypoint[i].pose.orientation.y)));
        WaypointElement->InsertEndChild(TiXmlElement("Ori_z"))->InsertEndChild(TiXmlText(Flt2Str(arWaypoint[i].pose.orientation.z)));
        WaypointElement->InsertEndChild(TiXmlElement("Ori_w"))->InsertEndChild(TiXmlText(Flt2Str(arWaypoint[i].pose.orientation.w)));
        RootElement->InsertEndChild(*WaypointElement);  
    }

    int nNumCharger = arCharger.size();
    for(int i=0;i<nNumCharger;i++)
    {
        TiXmlElement *ChargerElement = new TiXmlElement("Charger");
        ChargerElement->InsertEndChild(TiXmlElement("Name"))->InsertEndChild(TiXmlText(arCharger[i].name));
        ChargerElement->InsertEndChild(TiXmlElement("Pos_x"))->InsertEndChild(TiXmlText(Flt2Str(arCharger[i].pose.position.x)));
        ChargerElement->InsertEndChild(TiXmlElement("Pos_y"))->InsertEndChild(TiXmlText(Flt2Str(arCharger[i].pose.position.y)));
        ChargerElement->InsertEndChild(TiXmlElement("Pos_z"))->InsertEndChild(TiXmlText(Flt2Str(arCharger[i].pose.position.z)));
        ChargerElement->InsertEndChild(TiXmlElement("Ori_x"))->InsertEndChild(TiXmlText(Flt2Str(arCharger[i].pose.orientation.x)));
        ChargerElement->InsertEndChild(TiXmlElement("Ori_y"))->InsertEndChild(TiXmlText(Flt2Str(arCharger[i].pose.orientation.y)));
        ChargerElement->InsertEndChild(TiXmlElement("Ori_z"))->InsertEndChild(TiXmlText(Flt2Str(arCharger[i].pose.orientation.z)));
        ChargerElement->InsertEndChild(TiXmlElement("Ori_w"))->InsertEndChild(TiXmlText(Flt2Str(arCharger[i].pose.orientation.w)));
        RootElement->InsertEndChild(*ChargerElement);  
    }

    bool res = docSave->SaveFile(inFilename);
    if(res == true)
        ROS_INFO("Saved waypoints to file! filename = %s", inFilename.c_str());
    else
        ROS_INFO("Failed to save waypoints... filename = %s", inFilename.c_str());

    return res;
}

bool LoadWaypointsFromFile(std::string inFilename)
{
    TiXmlDocument docLoad(inFilename);
    bool resLoad = docLoad.LoadFile();
    if(resLoad == false)
    {
        ROS_INFO("Failed to load waypoints... filename = %s", inFilename.c_str());
        return false;
    }

    waterplus_map_tools::Waypoint newWayPoint;
    TiXmlElement* RootElement = docLoad.RootElement();
    for(TiXmlNode* item = RootElement->FirstChild("Waypoint");item;item = item->NextSibling("Waypoint"))
    {
        TiXmlNode* child = item->FirstChild();
        const char* name = child->ToElement()->GetText();
        ROS_INFO("Load waypoint : %s", name);
        newWayPoint.name = std::string(name); 
        child = item->IterateChildren(child);
        const char* pos_x = child->ToElement()->GetText();
        newWayPoint.pose.position.x = std::atof(pos_x);
        child = item->IterateChildren(child);
        const char* pos_y = child->ToElement()->GetText();
        newWayPoint.pose.position.y = std::atof(pos_y);
        child = item->IterateChildren(child);
        const char* pos_z = child->ToElement()->GetText();
        newWayPoint.pose.position.z = std::atof(pos_z);
        child = item->IterateChildren(child);
        const char* ori_x = child->ToElement()->GetText();
        newWayPoint.pose.orientation.x = std::atof(ori_x);
        child = item->IterateChildren(child);
        const char* ori_y = child->ToElement()->GetText();
        newWayPoint.pose.orientation.y = std::atof(ori_y);
        child = item->IterateChildren(child);
        const char* ori_z = child->ToElement()->GetText();
        newWayPoint.pose.orientation.z = std::atof(ori_z);
        child = item->IterateChildren(child);
        const char* ori_w = child->ToElement()->GetText();
        newWayPoint.pose.orientation.w = std::atof(ori_w);
        arWaypoint.push_back(newWayPoint);
    }

    for(TiXmlNode* item = RootElement->FirstChild("Charger");item;item = item->NextSibling("Charger"))
    {
        TiXmlNode* child = item->FirstChild();
        const char* name = child->ToElement()->GetText();
        ROS_INFO("Load charger : %s", name);
        newWayPoint.name = std::string(name); 
        child = item->IterateChildren(child);
        const char* pos_x = child->ToElement()->GetText();
        newWayPoint.pose.position.x = std::atof(pos_x);
        child = item->IterateChildren(child);
        const char* pos_y = child->ToElement()->GetText();
        newWayPoint.pose.position.y = std::atof(pos_y);
        child = item->IterateChildren(child);
        const char* pos_z = child->ToElement()->GetText();
        newWayPoint.pose.position.z = std::atof(pos_z);
        child = item->IterateChildren(child);
        const char* ori_x = child->ToElement()->GetText();
        newWayPoint.pose.orientation.x = std::atof(ori_x);
        child = item->IterateChildren(child);
        const char* ori_y = child->ToElement()->GetText();
        newWayPoint.pose.orientation.y = std::atof(ori_y);
        child = item->IterateChildren(child);
        const char* ori_z = child->ToElement()->GetText();
        newWayPoint.pose.orientation.z = std::atof(ori_z);
        child = item->IterateChildren(child);
        const char* ori_w = child->ToElement()->GetText();
        newWayPoint.pose.orientation.w = std::atof(ori_w);
        arCharger.push_back(newWayPoint);
    }

    return true;
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

    marker_chargers.header.frame_id = "map";
    marker_chargers.ns = "marker_waypoints";
    marker_chargers.action = visualization_msgs::Marker::ADD;
    marker_chargers.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_chargers.mesh_resource = "package://waterplus_map_tools/meshes/charger.dae";
    marker_chargers.scale.x = 1;
    marker_chargers.scale.y = 1;
    marker_chargers.scale.z = 1;
    marker_chargers.color.r = 0.5;
    marker_chargers.color.g = 0.0;
    marker_chargers.color.b = 1.0;
    marker_chargers.color.a = 1.0;
}

void DrawTextMarker(ros::Publisher* inPub, std::string inText, int inID, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB);
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

        std::string wp_name = arWaypoint[i].name;
        DrawTextMarker(&marker_pub,wp_name,i,0.2,wp_x,wp_y,0.55,1.0,0.0,0.0);
        ros::spinOnce();
    }

    int nNumCh = arCharger.size();
    for(int i=0; i<nNumCh ; i++ )
    {
        marker_chargers.id = i;
        marker_chargers.pose.position.x = arCharger[i].pose.position.x;
        marker_chargers.pose.position.y = arCharger[i].pose.position.y;
        marker_chargers.pose.position.z = -0.01;
        marker_chargers.pose.orientation = arCharger[i].pose.orientation;
        
        charger_pub.publish(marker_chargers);
        ros::spinOnce();

        float ch_x = arCharger[i].pose.position.x;
        float ch_y = arCharger[i].pose.position.y;

        std::string wp_name = arCharger[i].name;
        DrawTextMarker(&charger_pub, wp_name,i,0.2,ch_x,ch_y,0.35,1.0,0.0,0.0);
        ros::spinOnce();
    }
}

void DrawTextMarker(ros::Publisher* inPub, std::string inText, int inID, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB)
{
    text_marker.header.frame_id = "map";
    text_marker.ns = "text";
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

    inPub->publish(text_marker);
}

void AddWayPointCallback(const waterplus_map_tools::Waypoint::ConstPtr& wp)
{
    ROS_INFO("Add_waypoint: %s (%.2f %.2f) (%.2f %.2f %.2f %.2f) ",wp->name.c_str(), wp->pose.position.x, wp->pose.position.y, wp->pose.orientation.x, wp->pose.orientation.y, wp->pose.orientation.z, wp->pose.orientation.w);
    waterplus_map_tools::Waypoint newWayPoint;
    newWayPoint = *wp;
    arWaypoint.push_back(newWayPoint);
}

void AddChargerCallback(const waterplus_map_tools::Waypoint::ConstPtr& wp)
{
    ROS_INFO("Add_charger: %s (%.2f %.2f) (%.2f %.2f %.2f %.2f) ",wp->name.c_str(), wp->pose.position.x, wp->pose.position.y, wp->pose.orientation.x, wp->pose.orientation.y, wp->pose.orientation.z, wp->pose.orientation.w);
    waterplus_map_tools::Waypoint newCharger;
    newCharger = *wp;
    arCharger.push_back(newCharger);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wp_waypoint_manager");

    ros::NodeHandle n_param("~");
    std::string strLoadFile;
    n_param.param<std::string>("load", strLoadFile, "");
    if(strLoadFile.length() > 0)
    {
        ROS_INFO("Load waypoints from file : %s",strLoadFile.c_str());
        LoadWaypointsFromFile(strLoadFile);
    }

    ros::NodeHandle nh;
    marker_pub = nh.advertise<visualization_msgs::Marker>("waypoints_marker", 100);
    charger_pub = nh.advertise<visualization_msgs::Marker>("chargers_marker", 100);
    Init_Marker();
    ros::Subscriber add_waypoint_sub = nh.subscribe("/waterplus/add_waypoint",10,&AddWayPointCallback);
    ros::Subscriber add_charger_sub = nh.subscribe("/waterplus/add_charger",10,&AddChargerCallback);

    ros::ServiceServer srvGetNum = nh.advertiseService("/waterplus/get_num_waypoint", getNumOfWaypoints);
    ros::ServiceServer srvGetWPIndex = nh.advertiseService("/waterplus/get_waypoint_index", getWaypointByIndex);
    ros::ServiceServer srvGetWPName = nh.advertiseService("/waterplus/get_waypoint_name", getWaypointByName);
    ros::ServiceServer srvSaveWP = nh.advertiseService("/waterplus/save_waypoints", saveWaypoints);

    ros::ServiceServer srvGetChargerName = nh.advertiseService("/waterplus/get_charger_name", getChargerByName);

    ros::Rate r(10);

    while(ros::ok())
    {
        PublishWaypointsMarker();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
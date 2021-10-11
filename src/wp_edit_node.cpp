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
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <string>

using namespace visualization_msgs;
using namespace interactive_markers;
using namespace std;

static std::vector <waterplus_map_tools::Waypoint> arWaypoint;
static std::vector <waterplus_map_tools::Waypoint> arCharger;
static ros::Publisher marker_pub;
static visualization_msgs::Marker text_marker;
static InteractiveMarkerServer* pWaypointServer = NULL;
static MenuHandler* pMenuWaypoint = NULL;
static MenuHandler* pMenuCharger = NULL;

bool bDeleteWaypoint = false;
std::string strDelWaypointName;
bool bDeleteCharger = false;
std::string strDelChargerName;

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

void RemoveTextMarker()
{
    text_marker.action = 3;
    marker_pub.publish(text_marker);
}

void PublishTextMarker()
{
    int nNumWP = arWaypoint.size();
    for(int i=0; i<nNumWP ; i++ )
    {
        float wp_x = arWaypoint[i].pose.position.x;
        float wp_y = arWaypoint[i].pose.position.y;

        std::string wp_name = arWaypoint[i].name;
        DrawTextMarker(&marker_pub,wp_name,i,0.2,wp_x,wp_y,0.55,1.0,1.0,0.0);
        ros::spinOnce();
    }

    int nNumCh = arCharger.size();
    for(int i=0; i<nNumCh ; i++ )
    {
      
        float ch_x = arCharger[i].pose.position.x;
        float ch_y = arCharger[i].pose.position.y;

        std::string wp_name = arCharger[i].name;
        DrawTextMarker(&marker_pub, wp_name,nNumWP+i,0.2,ch_x,ch_y,0.35,1.0,1.0,0.0);
        ros::spinOnce();
    }
}

// 移动航点的回调函数
void processWaypointFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    ROS_WARN("[%s] p(%.2f,%.2f,%.2f) r(%.2f,%.2f,%.2f,%.2f)",feedback->marker_name.c_str(),
    feedback->pose.position.x,feedback->pose.position.y,feedback->pose.position.z,
    feedback->pose.orientation.x,feedback->pose.orientation.y,feedback->pose.orientation.z,feedback->pose.orientation.w);

    int nNumWP = arWaypoint.size();
    for(int i=0; i<nNumWP ; i++ )
    {
        if(feedback->marker_name == arWaypoint[i].name)
        {
            arWaypoint[i].pose = feedback->pose;
        }
    }

    std::ostringstream s;
    std::ostringstream mouse_point_ss;
    if( feedback->mouse_point_valid )
    {
        switch(feedback->event_type)
        {
            case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
                //ROS_INFO("BUTTON_CLICK");
                break;
            case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
                //ROS_INFO("MOUSE_DOWN");
                break;
            case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
                //ROS_INFO("MOUSE_UP");
                break;
            case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
                //ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
                break;
        }
    }
}

// 移动充电桩的回调函数
void processChargerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    ROS_WARN("[%s] p(%.2f,%.2f,%.2f) r(%.2f,%.2f,%.2f,%.2f)",feedback->marker_name.c_str(),
    feedback->pose.position.x,feedback->pose.position.y,feedback->pose.position.z,
    feedback->pose.orientation.x,feedback->pose.orientation.y,feedback->pose.orientation.z,feedback->pose.orientation.w);

    int nNumChargers = arCharger.size();
    for(int i=0; i<nNumChargers ; i++ )
    {
        if(feedback->marker_name == arCharger[i].name)
        {
            arCharger[i].pose = feedback->pose;
        }
    }

    std::ostringstream s;
    std::ostringstream mouse_point_ss;
    if( feedback->mouse_point_valid )
    {
        switch(feedback->event_type)
        {
            case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
                //ROS_INFO("BUTTON_CLICK");
                break;
            case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
                //ROS_INFO("MOUSE_DOWN");
                break;
            case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
                //ROS_INFO("MOUSE_UP");
                break;
            case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
                //ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
                break;
        }
    }
}

// 向服务器添加新的航点操作标记
void NewWaypointInterMarker(InteractiveMarkerServer* inServer,string inName, geometry_msgs::Pose InPose)
{
    visualization_msgs::InteractiveMarker wp_itr_marker;
    visualization_msgs::InteractiveMarkerControl wp_dis_ctrl;
    visualization_msgs::Marker wp_dis_marker;
    visualization_msgs::InteractiveMarkerControl move_control;
    wp_itr_marker.header.stamp=ros::Time::now();
    wp_itr_marker.name = inName;
    wp_itr_marker.description = inName;
    wp_itr_marker.pose = InPose;

    // 显示外形
    wp_itr_marker.header.frame_id = "map";
    wp_itr_marker.header.stamp=ros::Time::now();

    wp_dis_marker.ns = "marker_waypoints";
    wp_dis_marker.action = visualization_msgs::Marker::ADD;
    wp_dis_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    wp_dis_marker.mesh_resource = "package://waterplus_map_tools/meshes/waypoint.dae";
    wp_dis_marker.scale.x = 1;
    wp_dis_marker.scale.y = 1;
    wp_dis_marker.scale.z = 1;
    wp_dis_marker.color.r = 1.0;
    wp_dis_marker.color.g = 0.0;
    wp_dis_marker.color.b = 1.0;
    wp_dis_marker.color.a = 1.0;

    wp_dis_ctrl.markers.push_back( wp_dis_marker );
    wp_dis_ctrl.always_visible = true;
    wp_itr_marker.controls.push_back( wp_dis_ctrl );

    // 操作设置
    move_control.name = "move_x";
    move_control.orientation.w = 1.0;
	move_control.orientation.x = 1.0;
	move_control.orientation.y = 0.0;
	move_control.orientation.z = 0.0;
    move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    wp_itr_marker.controls.push_back(move_control);
    move_control.name = "move_z";
	move_control.orientation.x = 0.0;
	move_control.orientation.z = 1.0;
    wp_itr_marker.controls.push_back(move_control);
    move_control.name = "rotate_z";
	move_control.orientation.x = 0.0;
	move_control.orientation.y = 1.0;
	move_control.orientation.z = 0.0;
    move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    wp_itr_marker.controls.push_back(move_control);

    // 加入菜单
    visualization_msgs::Marker menu_marker;
    menu_marker.type = visualization_msgs::Marker::CUBE;
    menu_marker.scale.x = 0.5;
    menu_marker.scale.y = 0.5;
    menu_marker.scale.z = 0.5;
    menu_marker.color.r = 0.9;
    menu_marker.color.g = 0.9;
    menu_marker.color.b = 0.9;
    menu_marker.color.a = 0.0;  //全透明
    InteractiveMarkerControl menu_control;
    menu_control.interaction_mode = InteractiveMarkerControl::BUTTON;
    menu_control.always_visible = true;
    menu_control.markers.push_back( menu_marker );
    wp_itr_marker.controls.push_back( menu_control );

    inServer->insert(wp_itr_marker, &processWaypointFeedback);
}

// 向服务器添加新的充电桩操作标记
void NewChargerInterMarker(InteractiveMarkerServer* inServer,string inName, geometry_msgs::Pose InPose)
{
    visualization_msgs::InteractiveMarker wp_itr_marker;
    visualization_msgs::InteractiveMarkerControl wp_dis_ctrl;
    visualization_msgs::Marker wp_dis_marker;
    visualization_msgs::InteractiveMarkerControl move_control;
    wp_itr_marker.header.stamp=ros::Time::now();
    wp_itr_marker.name = inName;
    wp_itr_marker.description = inName;
    wp_itr_marker.pose = InPose;

    // 显示外形
    wp_itr_marker.header.frame_id = "map";
    wp_itr_marker.header.stamp=ros::Time::now();

    wp_dis_marker.ns = "marker_chargers";
    wp_dis_marker.action = visualization_msgs::Marker::ADD;
    wp_dis_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    wp_dis_marker.mesh_resource = "package://waterplus_map_tools/meshes/charger.dae";
    wp_dis_marker.scale.x = 1;
    wp_dis_marker.scale.y = 1;
    wp_dis_marker.scale.z = 1;
    wp_dis_marker.color.r = 0.0;
    wp_dis_marker.color.g = 0.0;
    wp_dis_marker.color.b = 1.0;
    wp_dis_marker.color.a = 1.0;

    wp_dis_ctrl.markers.push_back( wp_dis_marker );
    wp_dis_ctrl.always_visible = true;
    wp_itr_marker.controls.push_back( wp_dis_ctrl );

    // 操作设置
    move_control.name = "move_x";
    move_control.orientation.w = 1.0;
	move_control.orientation.x = 1.0;
	move_control.orientation.y = 0.0;
	move_control.orientation.z = 0.0;
    move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    wp_itr_marker.controls.push_back(move_control);
    move_control.name = "move_z";
	move_control.orientation.x = 0.0;
	move_control.orientation.z = 1.0;
    wp_itr_marker.controls.push_back(move_control);
    move_control.name = "rotate_z";
	move_control.orientation.x = 0.0;
	move_control.orientation.y = 1.0;
	move_control.orientation.z = 0.0;
    move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    wp_itr_marker.controls.push_back(move_control);

    // 加入菜单
    visualization_msgs::Marker menu_marker;
    menu_marker.type = visualization_msgs::Marker::CUBE;
    menu_marker.scale.x = 0.5;
    menu_marker.scale.y = 0.5;
    menu_marker.scale.z = 0.5;
    menu_marker.color.r = 0.9;
    menu_marker.color.g = 0.9;
    menu_marker.color.b = 0.9;
    menu_marker.color.a = 0.0;  //全透明
    InteractiveMarkerControl menu_control;
    menu_control.interaction_mode = InteractiveMarkerControl::BUTTON;
    menu_control.always_visible = true;
    menu_control.markers.push_back( menu_marker );
    wp_itr_marker.controls.push_back( menu_control );

    inServer->insert(wp_itr_marker, &processChargerFeedback);
}

// 菜单删除航点的回调函数
void DeleteWaypointCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  strDelWaypointName = feedback->marker_name;
  bDeleteWaypoint = true;
  ROS_WARN("Menu - Delete waypoint %s",strDelWaypointName.c_str());
}

// 菜单删除充电桩的回调函数
void DeleteChargerCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  strDelChargerName = feedback->marker_name;
  bDeleteCharger = true;
  ROS_WARN("Menu - Delete charger %s",strDelChargerName.c_str());
}

// 添加新航点回调函数
void AddWayPointCallback(const waterplus_map_tools::Waypoint::ConstPtr& wp)
{
    ROS_INFO("Add_waypoint: %s (%.2f %.2f) (%.2f %.2f %.2f %.2f) ",wp->name.c_str(), wp->pose.position.x, wp->pose.position.y, wp->pose.orientation.x, wp->pose.orientation.y, wp->pose.orientation.z, wp->pose.orientation.w);
    waterplus_map_tools::Waypoint newWayPoint;
    newWayPoint = *wp;
    int nWPNum = arWaypoint.size();
    for(int i= 0;i<nWPNum;i++)
    {
        if(newWayPoint.name == arWaypoint[i].name)
        {
            newWayPoint.name = newWayPoint.name + "_1";
        }
    }
    arWaypoint.push_back(newWayPoint);
    if(pWaypointServer != NULL)
    {
        NewWaypointInterMarker( pWaypointServer, newWayPoint.name, newWayPoint.pose );
        pMenuWaypoint->apply( *pWaypointServer, newWayPoint.name );
        //通知client（RVIZ）更新显示
        pWaypointServer->applyChanges();
    }
}

// 添加新充电桩回调函数
void AddChargerCallback(const waterplus_map_tools::Waypoint::ConstPtr& wp)
{
    ROS_INFO("Add_charger: %s (%.2f %.2f) (%.2f %.2f %.2f %.2f) ",wp->name.c_str(), wp->pose.position.x, wp->pose.position.y, wp->pose.orientation.x, wp->pose.orientation.y, wp->pose.orientation.z, wp->pose.orientation.w);
    waterplus_map_tools::Waypoint newCharger;
    newCharger = *wp;
    int nChargerNum = arCharger.size();
    for(int i= 0;i<nChargerNum;i++)
    {
        if(newCharger.name == arCharger[i].name)
        {
            newCharger.name = newCharger.name + "_1";
        }
    }
    arCharger.push_back(newCharger);
    if(pWaypointServer != NULL)
    {
        NewChargerInterMarker( pWaypointServer, newCharger.name, newCharger.pose );
        pMenuCharger->apply( *pWaypointServer, newCharger.name );
        //通知client（RVIZ）更新显示
        pWaypointServer->applyChanges();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wp_edit_node");

    std::string strLoadFile;
    char const* home = getenv("HOME");
    strLoadFile = home;
    strLoadFile += "/waypoints.xml";
    
    ros::NodeHandle n_param("~");
    std::string strParamFile;
    n_param.param<std::string>("load", strParamFile, "");
    if(strParamFile.length() > 0)
    {
        strLoadFile = strParamFile;
    }

    if(strLoadFile.length() > 0)
    {
        ROS_INFO("Load waypoints from file : %s",strLoadFile.c_str());
        LoadWaypointsFromFile(strLoadFile);
    }
    else
    {
        ROS_WARN("strLoadFile is empty. Failed to load waypoints!");
    }

    ros::NodeHandle nh;

    //创建服务
    interactive_markers::InteractiveMarkerServer wp_server("waypoints_move");
    pWaypointServer = &wp_server;
    
    marker_pub = nh.advertise<visualization_msgs::Marker>("text_marker", 100);
    ros::Subscriber add_waypoint_sub = nh.subscribe("waterplus/add_waypoint",10,&AddWayPointCallback);
    ros::Subscriber add_charger_sub = nh.subscribe("waterplus/add_charger",10,&AddChargerCallback);
    ros::ServiceServer srvGetNum = nh.advertiseService("waterplus/get_num_waypoint", getNumOfWaypoints);
    ros::ServiceServer srvGetWPIndex = nh.advertiseService("waterplus/get_waypoint_index", getWaypointByIndex);
    ros::ServiceServer srvGetWPName = nh.advertiseService("waterplus/get_waypoint_name", getWaypointByName);
    ros::ServiceServer srvSaveWP = nh.advertiseService("waterplus/save_waypoints", saveWaypoints);
    ros::ServiceServer srvGetChargerName = nh.advertiseService("waterplus/get_charger_name", getChargerByName);

    //将互动标记放到标记集合里，同时指定Feedback回调函数
    int nWPNum = arWaypoint.size();
    ROS_INFO("Num of waypoints = %d",nWPNum);
    for(int i=0; i< nWPNum; i++)
    {
        NewWaypointInterMarker( &wp_server, arWaypoint[i].name, arWaypoint[i].pose );
    }
    MenuHandler menu_waypoint;
    menu_waypoint.insert( "Delete", &DeleteWaypointCallback);
    pMenuWaypoint = &menu_waypoint;
    for(int i=0; i< nWPNum; i++)
    {
        menu_waypoint.apply( wp_server, arWaypoint[i].name );
    }
    // 充电桩标记初始化
    int nChargerNum = arCharger.size();
    ROS_INFO("Num of chargers = %d",nChargerNum);
    for(int i=0; i< nChargerNum; i++)
    {
        NewChargerInterMarker( &wp_server, arCharger[i].name, arCharger[i].pose );
    }
    MenuHandler menu_charger;
    menu_charger.insert( "Delete", &DeleteChargerCallback);
    pMenuCharger = &menu_charger;
    for(int i=0; i< nChargerNum; i++)
    {
        menu_charger.apply( wp_server, arCharger[i].name );
    }

    //通知client（RVIZ）更新显示
    wp_server.applyChanges();

    ros::Rate r(10);

    while(ros::ok())
    {
        if(bDeleteWaypoint == true)
        {
            bDeleteWaypoint = false;
            wp_server.erase(strDelWaypointName);
            wp_server.applyChanges();
            for(vector<waterplus_map_tools::Waypoint>::iterator iter=arWaypoint.begin(); iter!=arWaypoint.end(); )
            {
                if( (*iter).name == strDelWaypointName)
                    iter = arWaypoint.erase(iter);
                else
                    iter ++ ;
            }
            ROS_WARN("%s waypoint deleted!",strDelWaypointName.c_str());
            RemoveTextMarker();
        }
        if(bDeleteCharger == true)
        {
            bDeleteCharger = false;
            wp_server.erase(strDelChargerName);
            wp_server.applyChanges();
            for(vector<waterplus_map_tools::Waypoint>::iterator iter=arCharger.begin(); iter!=arCharger.end(); )
            {
                if( (*iter).name == strDelChargerName)
                    iter = arCharger.erase(iter);
                else
                    iter ++ ;
            }
            ROS_WARN("%s charger deleted!",strDelChargerName.c_str());
            RemoveTextMarker();
        }
        PublishTextMarker();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
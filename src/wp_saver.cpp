#include <ros/ros.h>
#include <waterplus_map_tools/SaveWaypoints.h>
#include <unistd.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wp_saver");

    ros::NodeHandle nh;
    ros::ServiceClient cliSave = nh.serviceClient<waterplus_map_tools::SaveWaypoints>("/waterplus/save_waypoints");
    waterplus_map_tools::SaveWaypoints srvS;

    char basePath[1024];
    memset(basePath, '\0', sizeof(basePath));
    getcwd(basePath, 999);
    std::string base_path(basePath);
    srvS.request.filename = base_path + "/waypoints.xml";

    for(int i=1; i<argc; i++)
    {
        if(!strcmp(argv[i], "-f"))
        {
            if(++i < argc)
            {
                srvS.request.filename = argv[i];
            }
        }
    }

    if (cliSave.call(srvS))
    {
        ROS_INFO("Save waypoints to the file : %s", srvS.request.filename.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call service save_waypoints");
    }

    return 0;
}
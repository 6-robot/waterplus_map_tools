#ifndef ADD_WAYPOINT_TOOL_H
#define ADD_WAYPOINT_TOOL_H

#ifndef Q_MOC_RUN
#include <QObject>
#include <ros/ros.h>
#include "rviz/default_plugin/tools/pose_tool.h"
#endif

namespace rviz
{
    class Arrow;
    class DisplayContext;
    class StringProperty;

    class AddWaypointTool: public rviz::PoseTool
    {
        Q_OBJECT
        public:
        AddWaypointTool();
        ~AddWaypointTool();

        virtual void onInitialize();

        protected:
        virtual void onPoseSet(double x, double y, double theta);

        private Q_SLOTS:
        void updateTopic();

        private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        StringProperty* topic_property_;
    };

}

#endif // ADD_WAYPOINT_TOOL_H
#ifndef ADD_CHARGER_TOOL_H
#define ADD_CHARGER_TOOL_H

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

    class AddChargerTool: public rviz::PoseTool
    {
        Q_OBJECT
        public:
        AddChargerTool();
        ~AddChargerTool();

        virtual void onInitialize();

        protected:
        virtual void onPoseSet(double x, double y, double theta);

        private Q_SLOTS:
        void updateTopic();

        private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        ros::ServiceClient cliGetChName;
        StringProperty* topic_property_;
    };

}

#endif // ADD_CHARGER_TOOL_H
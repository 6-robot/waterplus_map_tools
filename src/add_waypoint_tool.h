#ifndef ADD_WAYPOINT_TOOL_H
#define ADD_WAYPOINT_TOOL_H

#include <rviz/tool.h>

namespace Ogre
{
    class SceneNode;
    class Vector3;
}

namespace rviz
{
    class VectorProperty;
    class VisualizationManager;
    class ViewportMouseEvent;
}

namespace waterplus_map_tools
{

    class AddWayPointTool: public rviz::Tool
    {
        Q_OBJECT
        public:
        AddWayPointTool();
        ~AddWayPointTool();

        virtual void onInitialize();

        virtual void activate();
        virtual void deactivate();

        virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

        virtual void load( const rviz::Config& config );
        virtual void save( rviz::Config config ) const;

        private:
        void MakeWayPoint( const Ogre::Vector3& position );

        std::vector<Ogre::SceneNode*> waypoint_nodes_;
        Ogre::SceneNode* moving_waypoint_node_;
        std::string waypoint_resource_;
        rviz::VectorProperty* current_waypoint_property_;
    };

} // end namespace waterplus_map_tools

#endif // ADD_WAYPOINT_TOOL_H
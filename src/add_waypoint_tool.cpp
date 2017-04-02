#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include "add_waypoint_tool.h"

namespace waterplus_map_tools
{
    AddWayPointTool::AddWayPointTool()
    : moving_waypoint_node_( NULL )
    , current_waypoint_property_( NULL )
    {
        shortcut_key_ = 'a';
    }

    AddWayPointTool::~AddWayPointTool()
    {
        for( unsigned i = 0; i < waypoint_nodes_.size(); i++ )
        {
            scene_manager_->destroySceneNode( waypoint_nodes_[ i ]);
        }
    }

    void AddWayPointTool::onInitialize()
    {
    waypoint_resource_ = "package://waterplus_map_tools/media/waypoint.dae";

    if( rviz::loadMeshFromResource( waypoint_resource_ ).isNull() )
    {
        ROS_ERROR( "AddWayPointTool: failed to load model resource '%s'.", waypoint_resource_.c_str() );
        return;
    }

    moving_waypoint_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity = scene_manager_->createEntity( waypoint_resource_ );
    moving_waypoint_node_->attachObject( entity );
    moving_waypoint_node_->setVisible( false );
    }

    void AddWayPointTool::activate()
    {
    if( moving_waypoint_node_ )
    {
        moving_waypoint_node_->setVisible( true );

        current_waypoint_property_ = new rviz::VectorProperty( "WayPoint " + QString::number( waypoint_nodes_.size() ));
        current_waypoint_property_->setReadOnly( true );
        getPropertyContainer()->addChild( current_waypoint_property_ );
    }
    }

    void AddWayPointTool::deactivate()
    {
    if( moving_waypoint_node_ )
    {
        moving_waypoint_node_->setVisible( false );
        delete current_waypoint_property_;
        current_waypoint_property_ = NULL;
    }
    }

    int AddWayPointTool::processMouseEvent( rviz::ViewportMouseEvent& event )
    {
    if( !moving_waypoint_node_ )
    {
        return Render;
    }
    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
    if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                            ground_plane,
                                            event.x, event.y, intersection ))
    {
        moving_waypoint_node_->setVisible( true );
        moving_waypoint_node_->setPosition( intersection );
        current_waypoint_property_->setVector( intersection );

        if( event.leftDown() )
        {
        MakeWayPoint( intersection );
        current_waypoint_property_ = NULL; // Drop the reference so that deactivate() won't remove it.
        return Render | Finished;
        }
    }
    else
    {
        moving_waypoint_node_->setVisible( false ); // If the mouse is not pointing at the ground plane, don't show the waypoint.
    }
    return Render;
    }

    // This is a helper function to create a new waypoint in the Ogre scene and save its scene node in a list.
    void AddWayPointTool::MakeWayPoint( const Ogre::Vector3& position )
    {
        Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
        Ogre::Entity* entity = scene_manager_->createEntity( waypoint_resource_ );
        node->attachObject( entity );
        node->setVisible( true );
        node->setPosition( position );
        waypoint_nodes_.push_back( node );
    }


    void AddWayPointTool::save( rviz::Config config ) const
    {
        config.mapSetValue( "Class", getClassId() );

        rviz::Config waypoints_config = config.mapMakeChild( "WayPoints" );

        rviz::Property* container = getPropertyContainer();
        int num_children = container->numChildren();
        for( int i = 0; i < num_children; i++ )
        {
            rviz::Property* position_prop = container->childAt( i );
            
            rviz::Config waypoint_config = waypoints_config.listAppendNew();
            waypoint_config.mapSetValue( "Name", position_prop->getName() );
            position_prop->save( waypoint_config );
        }
    }

    void AddWayPointTool::load( const rviz::Config& config )
    {
        rviz::Config waypoints_config = config.mapGetChild( "WayPoints" );
        int num_waypoints = waypoints_config.listLength();
        for( int i = 0; i < num_waypoints; i++ )
        {
            rviz::Config waypoint_config = waypoints_config.listChildAt( i );
        
            QString name = "WayPoint " + QString::number( i + 1 );
            
            waypoint_config.mapGetString( "Name", &name );
            rviz::VectorProperty* prop = new rviz::VectorProperty( name );
            prop->load( waypoint_config );
            prop->setReadOnly( true );
            getPropertyContainer()->addChild( prop );
            MakeWayPoint( prop->getVector() );
        }
    }

} // end namespace waterplus_map_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(waterplus_map_tools::AddWayPointTool,rviz::Tool)

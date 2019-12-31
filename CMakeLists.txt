cmake_minimum_required(VERSION 2.8.3)
project(waterplus_map_tools)

find_package(catkin REQUIRED COMPONENTS message_generation rviz roscpp geometry_msgs std_msgs tf visualization_msgs interactive_markers move_base_msgs actionlib cmake_modules )

add_service_files(FILES
  SaveWaypoints.srv
  AddNewWaypoint.srv
  GetNumOfWaypoints.srv
  GetWaypointByIndex.srv
  GetWaypointByName.srv
  GetChargerByName.srv
)

add_message_files(FILES
  Waypoint.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
  INCLUDE_DIRS include
  LIBRARIES waterplus_map_tools 
  DEPENDS TinyXML
  )


include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

set(CMAKE_AUTOMOC ON)
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()
add_definitions(-DQT_NO_KEYWORDS)

find_package(TinyXML REQUIRED)

add_library(${PROJECT_NAME} 
    include/add_waypoint_tool.h
    src/add_waypoint_tool.cpp
    include/add_charger_tool.h
    src/add_charger_tool.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

## Install rules

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  waterplus_map_tools_plugin.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(wp_manager
  src/wp_manager.cpp
)
add_dependencies(wp_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wp_manager
  ${catkin_LIBRARIES} ${TinyXML_LIBRARIES}
)

add_executable(wp_edit_node
  src/wp_edit_node.cpp
)
add_dependencies(wp_edit_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wp_edit_node
  ${catkin_LIBRARIES} ${TinyXML_LIBRARIES}
)

add_executable(wp_nav_test
  src/wp_nav_test.cpp
)
add_dependencies(wp_nav_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wp_nav_test
  ${catkin_LIBRARIES}
)

add_executable(wp_saver
  src/wp_saver.cpp
)
add_dependencies(wp_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wp_saver
  ${catkin_LIBRARIES}
)

add_executable(wp_nav_remote
  src/wp_nav_remote.cpp
  src/network/UDPServer.c
)
add_dependencies(wp_nav_remote ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wp_nav_remote
  ${catkin_LIBRARIES}
)

add_executable(wp_nav_odom_report
  src/wp_nav_odom_report.cpp
  src/network/UDPClient.c
)
add_dependencies(wp_nav_odom_report ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wp_nav_odom_report
  ${catkin_LIBRARIES}
)

add_executable(charger_get_position
  src/charger_get_position.cpp
)
add_dependencies(charger_get_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(charger_get_position
  ${catkin_LIBRARIES}
)

add_executable(wp_navi_server
  src/wp_navi_server.cpp
)
add_dependencies(wp_navi_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wp_navi_server
  ${catkin_LIBRARIES}
)

add_executable(pose_navi_server
  src/pose_navi_server.cpp
)
add_dependencies(pose_navi_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_navi_server
  ${catkin_LIBRARIES}
)
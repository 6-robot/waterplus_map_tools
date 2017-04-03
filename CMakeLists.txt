cmake_minimum_required(VERSION 2.8.3)
project(waterplus_map_tools)

find_package(catkin REQUIRED COMPONENTS message_generation rviz roscpp geometry_msgs std_msgs tf visualization_msgs move_base_msgs actionlib cmake_modules )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waterplus_map_tools 
  DEPENDS TinyXML
  )

include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

set(CMAKE_AUTOMOC ON)
find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

find_package(TinyXML REQUIRED)

add_library(${PROJECT_NAME} 
    src/add_waypoint_tool.h
    src/add_waypoint_tool.cpp
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


add_executable(wp_waypoint_manager
  src/wp_waypoint_manager.cpp
)
add_dependencies(wp_waypoint_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wp_waypoint_manager
  ${catkin_LIBRARIES} ${TinyXML_LIBRARIES}
)
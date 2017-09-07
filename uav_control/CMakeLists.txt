cmake_minimum_required(VERSION 2.8.3)
project(uav_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  mavros_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  marker_detection.msg
  choice.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES uav_control
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_library(uav_control
#   src/${PROJECT_NAME}/uav_control.cpp
# )

# add_dependencies(uav_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(uav_control_node src/UAVSpiralMotion.cpp)
add_executable(uav_take_off src/UAVTakeoff.cpp)
add_executable(uav_waypoint src/UAVWaypointControl.cpp)
add_executable(uav_motion_primitive src/UAVMotionPrimitive.cpp)
add_executable(uav_marker_tracking src/UAVMarkerTracking.cpp)
#add_executable(marker_pooling src/MarkerPooling.cpp)

# add_dependencies(uav_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(uav_take_off
  ${catkin_LIBRARIES}
)
target_link_libraries(uav_waypoint
  ${catkin_LIBRARIES}
)
target_link_libraries(uav_motion_primitive
  ${catkin_LIBRARIES}
)
target_link_libraries(uav_marker_tracking
  ${catkin_LIBRARIES}
)
#target_link_libraries(marker_pooling
#  ${catkin_LIBRARIES}
#)
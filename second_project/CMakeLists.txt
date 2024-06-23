cmake_minimum_required(VERSION 3.0.2)
project(second_project)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  tf
  dynamic_reconfigure
  gmapping
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES first_project
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_tf_publisher src/odom_tf_publisher.cpp)
target_link_libraries(odom_tf_publisher ${catkin_LIBRARIES})

add_executable(goal_publisher src/goal_publisher.cpp)
target_link_libraries(goal_publisher ${catkin_LIBRARIES})

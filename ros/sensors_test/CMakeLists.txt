cmake_minimum_required(VERSION 2.8.3)
project(sensors_test)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_msgs
  geometry_msgs
  joy
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES chapter4
#  CATKIN_DEPENDS dynamixel_msgs geometry_msgs joy nav_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(teleop_joy src/teleop_joy.cpp)
add_executable(laser_scan src/laser_scan.cpp)
add_executable(dynamixel_controller src/dynamixel_controller.cpp)
add_executable(range_ultra src/range_ultra.cpp)
add_executable(teleop_imu src/teleop_imu.cpp)
add_executable(gps src/gps.cpp)

target_link_libraries(teleop_joy ${catkin_LIBRARIES})
target_link_libraries(laser_scan ${catkin_LIBRARIES})
target_link_libraries(dynamixel_controller ${catkin_LIBRARIES})
target_link_libraries(range_ultra ${catkin_LIBRARIES} )
target_link_libraries(teleop_imu ${catkin_LIBRARIES})
target_link_libraries(gps ${catkin_LIBRARIES})

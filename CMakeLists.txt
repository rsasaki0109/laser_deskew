cmake_minimum_required(VERSION 2.8.3)
project(laser_deskew)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  nav_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

message(status  ${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/laser_deskew.cpp src/laser_deskew_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
 )

add_executable(odom2tf src/odom2tf.cpp)
target_link_libraries(odom2tf ${catkin_LIBRARIES})
add_dependencies(odom2tf ${catkin_EXPORTED_TARGETS})

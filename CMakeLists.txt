cmake_minimum_required(VERSION 2.8.3)
project(spco_mapping)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(colormap_saver src/colormap_saver.cpp)
target_link_libraries(colormap_saver ${catkin_LIBRARIES})

cmake_minimum_required(VERSION 2.8.3)
project(3d-pointcloud)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(velopoints src/velo_points.cpp)
add_executable(octree_node src/octree.cpp)

target_link_libraries(velopoints
   ${catkin_LIBRARIES}
 )

target_link_libraries(octree_node
  	${catkin_LIBRARIES}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
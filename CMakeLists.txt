cmake_minimum_required(VERSION 2.8.3)
project(viewer)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
#	INCLUDE_DIRS include
	LIBRARIES viewer
	CATKIN_DEPENDS roscpp rospy std_msgs pcl_ros  
	)


add_executable(velopoints src/velo_points.cpp)

add_executable(octree_node src/octree.cpp)

target_link_libraries(velopoints
   ${catkin_LIBRARIES}
 )

target_link_libraries(octree_node
  	${catkin_LIBRARIES}
)
add_dependencies(velopoints roscpp rospy std_msgs pcl_ros)
add_dependencies(octree_node roscpp rospy std_msgs pcl_ros)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

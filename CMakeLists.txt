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
add_executable(don src/don.cpp)
add_executable(lanes src/lanes.cpp)
add_executable(octree_node src/octree.cpp)
add_executable(c_c src/conditional_clus.cpp)
add_executable(vie src/pclstream.cpp)
add_executable(rg src/rg_seg.cpp)
add_executable(road src/road.cpp)
add_executable(try src/trial.cpp)

target_link_libraries(velopoints
  ${catkin_LIBRARIES}
 )

 target_link_libraries(try
   ${catkin_LIBRARIES}
  )
 target_link_libraries(road
   ${catkin_LIBRARIES}
  )

 target_link_libraries(rg
    ${catkin_LIBRARIES}
  )

 target_link_libraries(don
    ${catkin_LIBRARIES}
  )

 target_link_libraries(lanes
    ${catkin_LIBRARIES}
  )

 target_link_libraries(c_c
    ${catkin_LIBRARIES}
  )
target_link_libraries(octree_node
  	${catkin_LIBRARIES}
)

target_link_libraries(vie
  	${catkin_LIBRARIES}
)

add_dependencies(road roscpp rospy std_msgs pcl_ros)
add_dependencies(rg roscpp rospy std_msgs pcl_ros)
add_dependencies(don roscpp rospy std_msgs pcl_ros)
add_dependencies(velopoints roscpp rospy std_msgs pcl_ros)
add_dependencies(lanes roscpp rospy std_msgs pcl_ros)
add_dependencies(octree_node roscpp rospy std_msgs pcl_ros)
add_dependencies(c_c roscpp rospy std_msgs pcl_ros)
add_dependencies(vie roscpp rospy std_msgs pcl_ros)
add_dependencies(try roscpp rospy std_msgs pcl_ros)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

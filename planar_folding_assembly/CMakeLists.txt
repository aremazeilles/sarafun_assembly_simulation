cmake_minimum_required( VERSION 2.8.3 )
project( planar_folding_assembly )

find_package( catkin REQUIRED )

find_package( gazebo REQUIRED )

include_directories( ${GAZEBO_INCLUDE_DIRS} )

link_directories( ${GAZEBO_LIBRARY_DIRS} )

catkin_package(
  LIBRARIES gazebo_ros_planar_folding_assembly_controller
  DEPENDS gazebo SDF
)

add_library( gazebo_ros_planar_folding_assembly_controller src/planar_folding_assembly_controller_plugin.cpp )
target_link_libraries( gazebo_ros_planar_folding_assembly_controller ${GAZEBO_LIBRARIES} )



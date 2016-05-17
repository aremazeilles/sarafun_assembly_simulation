cmake_minimum_required( VERSION 2.8.3 )
project( planar_folding_assembly )

find_package( catkin REQUIRED COMPONENTS
  geometry_msgs
)

find_package( gazebo REQUIRED )

find_package( Boost REQUIRED COMPONENTS thread )

include_directories( ${GAZEBO_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )

link_directories( ${GAZEBO_LIBRARY_DIRS} )

catkin_package(
  LIBRARIES gazebo_ros_planar_folding_assembly_controller
  CATKIN_DEPENDS geometry_msgs
  DEPENDS gazebo SDF
)

add_library( sarafun_pfa_empty_controller src/empty_controller.cpp )
target_link_libraries( sarafun_pfa_empty_controller ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} )

add_library( sarafun_pfa_vertical_force_controller src/vertical_force_controller.cpp )
target_link_libraries( sarafun_pfa_vertical_force_controller ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} )

add_library( sarafun_pfa_vertical_force_admittance_controller src/vertical_force_admittance_controller.cpp )
target_link_libraries( sarafun_pfa_vertical_force_admittance_controller ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} )



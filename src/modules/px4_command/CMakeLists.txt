cmake_minimum_required(VERSION 2.8.3)
project(px4_command)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  geometry_msgs
  sensor_msgs
  mavros
  nav_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  mavros_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vision_to_mavros
#  CATKIN_DEPENDS roscpp rospy tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(position_to_mavros_node src/position_to_mavros.cpp)
add_executable(set_mode_node src/Utilities/set_mode.cpp)
add_executable(collision_avoidance_vfh_node src/Application/collision_avoidance_vfh.cpp)
add_executable(collision_avoidance_matlabvfh_node src/Application/collision_avoidance_matlabvfh.cpp)
add_executable(circular_node src/Application/circular.cpp)
add_executable(circular_car_node src/Application/circular_car.cpp)
add_executable(ros_nav_quadrotor_node src/Application/ros_nav_quadrotor.cpp)

## Specify libraries to link a library or executable target against

target_link_libraries(position_to_mavros_node
  ${catkin_LIBRARIES}
)
target_link_libraries(set_mode_node
  ${catkin_LIBRARIES}
)
target_link_libraries(collision_avoidance_vfh_node
  ${catkin_LIBRARIES}
)
target_link_libraries(collision_avoidance_matlabvfh_node
  ${catkin_LIBRARIES}
)
target_link_libraries(circular_node
  ${catkin_LIBRARIES}
)
target_link_libraries(circular_car_node
  ${catkin_LIBRARIES}
)
target_link_libraries(ros_nav_quadrotor_node
  ${catkin_LIBRARIES}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
## Mark executables and/or libraries for installation
install(TARGETS  position_to_mavros_node set_mode_node collision_avoidance_vfh_node collision_avoidance_matlabvfh_node circular_node circular_car_node ros_nav_quadrotor_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

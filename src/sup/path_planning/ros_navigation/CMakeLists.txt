cmake_minimum_required(VERSION 2.8.3)
project(ros_navigation)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation
	move_base
	map_server
	amcl
	cartographer
	dwa_local_planner
)


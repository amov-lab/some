/***************************************************************************************************************************
*
* Author: bingo
* Email: bingobin.lw@gmail.com
* Time: 2019.12.17
* Description: Sending Position Information to mavros,The location information may come from lidar or t265

***************************************************************************************************************************/


#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher position_pub;

int main(int argc, char** argv){
  ros::init(argc, argv, "position_to_mavros");

  ros::NodeHandle node;

  geometry_msgs::PoseStamped cur_position;

  position_pub = node.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("carto_odom", "base_link",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    cur_position.pose.position.x = transformStamped.transform.translation.x ;
    cur_position.pose.position.y = transformStamped.transform.translation.y ;
    cur_position.pose.position.z = transformStamped.transform.translation.z ;

    cur_position.pose.orientation.x = transformStamped.transform.rotation.x;
    cur_position.pose.orientation.y = transformStamped.transform.rotation.y;
    cur_position.pose.orientation.z = transformStamped.transform.rotation.z;
    cur_position.pose.orientation.w = transformStamped.transform.rotation.w;
    cur_position.header.stamp = ros::Time::now();
    position_pub.publish(cur_position);
    rate.sleep();
  }
  return 0;
};



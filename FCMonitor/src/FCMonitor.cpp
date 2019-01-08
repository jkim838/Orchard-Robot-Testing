/*#include <ros/ros.h>
#include <stdio.h>
#include <gtest/gtest.h>
#include <geometry_msgs/Vector3.h>
#include "vision_msgs/Detection2DArray"
int length;

void object_position(const vision_msgs::Detection2DArray::constPtr& data){
  length = sizeof(data);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "FC_control_node");
  ros::NodeHandle FC_nh;
  ros::Subscriber FC_sub = FC.nh.subscribe("/objects", 10, object_position);
  ros::Rate rate(10);
  while(ros::ok()){
    ros::spinOnce();
    ROS_INFO('length: %i', length);
    rate.sleep();
  }
}*/

#include <ros/ros.h>
#include <stdio.h>
#include <gtest/gtest.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>

std_msgs::Float32MultiArray array;

void object_position(const std_msgs::Float32MultiArray::ConstPtr& data){
  array.data = data->data;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "FC_control_node");
  ros::NodeHandle FC_nh;
  ros::Subscriber FC_sub = FC_nh.subscribe("/position", 10, object_position);
  ros::Publisher FC_pub = FC_nh.advertise<std_msgs::Float32MultiArray>("/ObjectPosition",10);
  ros::Rate rate(10);


  while(ros::ok()){
    ros::spinOnce();
    FC_pub.publish(array);
    rate.sleep();
  }
}

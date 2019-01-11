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
#include <std_msgs/UInt32.h>
#include <vector>
#include <iostream>
#include <sensor_msgs/Image.h>

std_msgs::Float32MultiArray array;
unsigned int height_center;
unsigned int width_center;

/*
int find_length(std_msgs::Float32MultiArray *passed_array){

  int i = 0;
  for(int it = passed_array.begin(); it != passed_array.end(); ++it){
    i++;
  }
  return i;

}
*/

void object_position(const std_msgs::Float32MultiArray::ConstPtr& data){
  array.data = data->data;
  int arr_length = array.data.size();
  if(arr_length >= 3){
    for (int i = 1; i < arr_length - 1; i+=3){
      if((const unsigned) array.data[i] >= height_center-10 && (const unsigned) array.data[i] <= height_center+10){
        std::cout << "Located!" << std::endl;
      }
    }
  }
}

void image_size(const sensor_msgs::Image::ConstPtr& image_data){

  height_center = image_data->height;
  width_center = image_data->width;
  height_center /=2;
  width_center /=2;

}

int main(int argc, char **argv){
  ros::init(argc, argv, "FC_control_node");
  ros::NodeHandle FC_nh;
  ros::Subscriber FC_coordinate_sub = FC_nh.subscribe("/position", 10, object_position);
  ros::Subscriber FC_array_size_sub = FC_nh.subscribe("/debug_image", 10, image_size);
  ros::Publisher FC_pub = FC_nh.advertise<std_msgs::Float32MultiArray>("/ObjectPosition",10);
  ros::Rate rate(10);


  while(ros::ok()){
    ros::spinOnce();
    FC_pub.publish(array);
    rate.sleep();
  }
}

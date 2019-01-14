#include <ros/ros.h>
#include <stdio.h>
#include <gtest/gtest.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <vector>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <math.h>

#define PI 3.14159265
#define CENTER_MARGIN 50

std_msgs::Float32MultiArray array;
sensor_msgs::Image image_depth_array;
unsigned int height_center;
unsigned int width_center;
unsigned int maximum_depth;

float calculate_width(){

  float distance = 925; //mm
  float theta_degree = 58; //degrees
  // radians = ( degrees * pi ) / 180 ;
  float theta_rad = theta_degree * PI / 180 / 2;
  float width_in_mm = 2*distance*tan(theta_rad);
  std::cout << "width in mm: " << width_in_mm << std::endl;
  return width_in_mm;

}

void object_position(const std_msgs::Float32MultiArray::ConstPtr& data){
  array.data = data->data;
  // Find the size of the array containing the x,y-coordinates and score.
  // Divide the legnth of the array by three (x,y,score) to determine the number of the detected object.
  int arr_length = array.data.size();
  if(arr_length >= 3){
    // for every y-coordinate elements of the detected object,
    for (int i = 1; i < arr_length - 1; i+=3){
      // For realistic purposes, give 10px margin between center point of the y-axis...
      if((const unsigned) array.data[i] >= height_center- CENTER_MARGIN && (const unsigned) array.data[i] <= height_center + CENTER_MARGIN){
        std::cout << "Located!" << std::endl;

        // we assume the viewing distance (horizontal) is 1700mm at 925mm between camera surface and the canopy.
        // because image is 640px wide, each pixel represents 2.66mm
        float px_mm = calculate_width() / 640;
        // convert x-coordinate to mm...
        float x_mm = array.data[i-1] * px_mm;
        std::cout << "X in mm: " << x_mm << std::endl;
        // to determine the z-position of the target, use the maximum depth (maximum distance away from the camera)
        // remeber that depth is stored as an array... therefore we need to go through all the array, find the maximum distance.
      }
    }
  }
}

void image_size(const sensor_msgs::Image::ConstPtr& image_data){
  // Subscribed to topic "image", obtain the dimension of the image (default: 640*480px)...
  height_center = image_data->height;
  width_center = image_data->width;
  // Determine the center point of x,y-axis...
  height_center /=2;
  width_center /=2;
}

void image_depth(const sensor_msgs::Image::ConstPtr& image_depth_data){
  image_depth_array.data = image_depth_data->data;
  std::cout << "Image Width: " << image_depth_data->width << std::endl;
  std::cout << "Image Height: " << image_depth_data->height << std::endl;
  std::cout << "Array Length is: " << image_depth_array.data.size() << std::endl;
  // Iterate throgh entire array of depth...
  //for(int i = 0; i < )
}

int main(int argc, char **argv){
  ros::init(argc, argv, "FC_control_node");
  ros::NodeHandle FC_nh;
  // Subscriber Declaration...
  ros::Subscriber FC_coordinate_sub = FC_nh.subscribe("/position", 10, object_position);
  ros::Subscriber FC_array_size_sub = FC_nh.subscribe("/debug_image", 10, image_size);
  // ros::Subscriber FC_depth_info_sub = FC_nh.subscribe("/camera/depth/image_rect_raw", 10, image_depth);
  // Publisher Declaration...
  ros::Publisher FC_pub = FC_nh.advertise<std_msgs::Float32MultiArray>("/ObjectPosition",10);
  ros::Rate rate(10);


  while(ros::ok()){
    ros::spinOnce();
    FC_pub.publish(array);
    rate.sleep();
  }
}

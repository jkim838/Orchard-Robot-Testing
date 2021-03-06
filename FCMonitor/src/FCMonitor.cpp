#include <ros/ros.h>
#include <stdio.h>
#include <gtest/gtest.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <vector>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <math.h>
#include <librealsense2/rs.hpp>


#define PI 3.14159265
#define CENTER_MARGIN 20
#define LASER_FACTOR 24.0625

std_msgs::Float32MultiArray array;
sensor_msgs::Image image_depth_array;
unsigned int height_center;
unsigned int width_center;
unsigned int maximum_depth;

std_msgs::UInt64 laser_number;

//rs2::pipeline p;

//prototype
float image_depth(float x, float y);

float calculate_width(){

  float distance = 940; //mm
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
        std::cout << "Located within the margin!" << std::endl;

        // we assume the viewing distance (horizontal) is 1042mm at 940mm between camera surface and the canopy.
        // because image is 640px wide, each pixel represents 1.63mm

        // calculate depth to target...
        //float depth_to_target = image_depth(array.data[i-1], array.data[i]);

        // calculate the half of the view width...
        // by default it should be 521mm (0.5 * 1042mm)
        float half_width_mm = calculate_width() / 2;
        // calculate millimeter per pixel...
        float px_mm =  2 * half_width_mm / (width_center *2); // mm per pxiel
        // convert x-coordinate (in pixels) to millemeters...
        float x_mm = array.data[i-1] * px_mm; // target location in mm (x-axis)

        float laser_mm = 0; // location of the target in respect to the laser panel...
        float x3 = 0;
        float laser_correction = 0;
        uint64_t laser_number_feed = 0;

        // determine if the target is on the left-half/right-half side of the laser panel...
        if(x_mm >= half_width_mm && x_mm < 2* half_width_mm + 1){
          // the target is on right-half...
          // 385mm offset for laser panel, (x_mm - half_width_mm) for offset from viewing widht center point
          x3 = x_mm - half_width_mm;
          laser_correction = (x3/2.8249);
          std::cout << "debug: right" << std::endl;
          std::cout << "debug: correction factor: " << laser_correction << std::endl;
          laser_mm = 385 + x3 - laser_correction;
          laser_number_feed = (uint64_t)(laser_mm / LASER_FACTOR)+1;
        }
        else if(x_mm >= 0 && x_mm < half_width_mm){
          // the target is on left-half...
          x3 = half_width_mm - x_mm;
          laser_correction = (x3/2.8249);
          std::cout << "debug: left" << std::endl;
          std::cout << "debug: correction factor: " << laser_correction << std::endl;
          laser_mm = 385 - x3 + laser_correction;
          laser_number_feed = (uint64_t)(laser_mm / LASER_FACTOR)+2;
        }
        std::cout << "Debug: Laser mm is" << laser_mm << std::endl;
	      //Debugging Mode

        /***
        To-Do: convert laser_mm to laser number.
        ***/

        if(laser_number_feed > 32){
          laser_number_feed = 0;
        }
        laser_number.data = laser_number_feed;

        std::cout << "Debug: Activating Laser: " << laser_number_feed << std::endl;

      }
    }
  }
}

/*
float image_depth(float x, float y){

  std::cout << "POINT 1" << std::endl;
  rs2::frameset frames = p.wait_for_frames();
  std::cout << "POINT 2" << std::endl;
  rs2::depth_frame depth = frames.get_depth_frame();
  float depth_pixel = depth.get_distance(x,y);
  std::cout << "Debug: Depth at target" << x << "," << y << " is: " << depth_pixel << "mm" << std::endl;
  return depth_pixel;

}
*/

void image_size(const sensor_msgs::Image::ConstPtr& image_data){

  // Subscribed to topic "image", obtain the dimension of the image (default: 640*480px)...
  height_center = image_data->height;
  width_center = image_data->width;
  // Determine the center point of x,y-axis...
  height_center /=2;
  width_center /=2;

}

int main(int argc, char **argv){
  ros::init(argc, argv, "FC_control_node");
  ros::NodeHandle FC_nh;
  // Subscriber Declaration...
  ros::Subscriber FC_coordinate_sub = FC_nh.subscribe("/position", 10, object_position);
  ros::Subscriber FC_array_size_sub = FC_nh.subscribe("/debug_image", 10, image_size); // this is running multiple times?
  //ros::Subscriber FC_depth_info_sub = FC_nh.subscribe("/camera/depth/image_rect_raw", 10, image_depth);

  // Publisher Declaration...
  ros::Publisher FC_pub = FC_nh.advertise<std_msgs::UInt64>("/LaserNumber",10);
  ros::Rate rate(10);

  //p.start();

  while(ros::ok()){
    ros::spinOnce();
    FC_pub.publish(laser_number);
    rate.sleep();
  }
}

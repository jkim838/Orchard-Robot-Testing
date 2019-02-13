#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>

geometry_msgs::Vector3 position;
geometry_msgs::Vector3 orientation;
geometry_msgs::Twist pioneer_vel; 
float yaw_angle;
tf::Pose pose;
 float desiredAngle = 0;
 float kp = 2.5;
 float ki = 500;
 float integral;

//IMPORTANT:Motors are set to m/s therefore setting anything above 1 is pretty fast

//This function will process the pioneer robot position and orientation obtained from the odometer 
void pioneer_position(const nav_msgs::Odometry::ConstPtr& odometerData){

 float error;


 tf::poseMsgToTF(odometerData->pose.pose, pose);
 yaw_angle = tf::getYaw(pose.getRotation());
 position.x = odometerData->pose.pose.position.x;
 position.y = odometerData->pose.pose.position.y;
 position.z = odometerData->pose.pose.position.z;

 if(yaw_angle != desiredAngle){
	error = desiredAngle - yaw_angle;
	integral = integral + (error*0.0001);
	pioneer_vel.angular.z = kp*error + ki*integral;
	
 }
	pioneer_vel.linear.x = 0.03;
	sleep(0.0001);
}

int main (int argc, char **argv){

 //create a node for sending command to pioneer
 ros::init(argc,argv, "pioneer_node");
 
 //create a handler for the node
 ros::NodeHandle pioneer_nh;

 //publish velocity command to RosAria
 ros::Publisher pioneer_pub = pioneer_nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",10);

 //subscribe to the odometry to read the position of the robot 
 ros::Subscriber pioneer_sub = pioneer_nh.subscribe("/RosAria/pose",10, pioneer_position);

 //Loop the velocity command at 10Hz
 ros::Rate rate(10);

 while (ros::ok()){

  //setting the velocity of the pioneer robot. geometry_msgs::Twist type contains linear and angular velocity varibale
  //linear and angular velcoty are both of type Vector 3, which is float x, float y, float z
  //Parallel to the rear wheels is X-axis, running through centre of the rear wheels is Y-axis and perpendicular to ground is Z-axis. 

  pioneer_pub.publish(pioneer_vel);
  ROS_INFO("running...");

  //retrieve information from odometer by calling the pioneer_position
  ros::spinOnce();
  ROS_INFO("x-pose: %f    y-pose: %f   z-pose: %f", position.x,position.y,position.z);
  ROS_INFO("z-orien: %f", yaw_angle);

  rate.sleep();
 
 }//while end
} //main end

#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Int16.h>
#include <stdlib.h> // For rand() and RAND_MAX

std_msgs::UInt64 nozzle_states;
std_msgs::UInt64 test;

void laserCommandCallback(const std_msgs::UInt64::ConstPtr& laserCommandData){

		std_msgs::UInt64 temp;
		nozzle_states.data = 210;//ms(0-255)
		nozzle_states.data = nozzle_states.data <<56;
		temp.data = (uint64_t) 1 << (laserCommandData->data)-1; //(0-40)
		nozzle_states.data |= temp.data;
}


int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "publish_velocity");
  ros::NodeHandle nh;

  // Create a publisher object.
  ros::Publisher pub = nh.advertise<std_msgs::UInt64>(
    "/nozzleBank", 1000);

  ros::Publisher pub2 = nh.advertise<std_msgs::UInt64>("/testing",1000);

  ros::Subscriber sub_laser_num = nh.subscribe("/LaserNumber", 100, laserCommandCallback);

  ros::Rate rate(50);
  // Loop at 5Hz until the node is shut down.
  while(ros::ok()) {
    ros::spinOnce();
    pub.publish(nozzle_states);
    nozzle_states.data = 0;
    rate.sleep();
  }//end while
}//end main 

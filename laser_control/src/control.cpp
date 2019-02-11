#include <ros/ros.h>
#include <std_msgs/UInt64.h>

int main (int argc, char **argv)
{
	ros::init(argc, argv, "controller_node");
	ros::NodeHandle new_handle;

	ros::Publisher pub = new_handle.advertise<std_msgs::UInt64>("/commandNode",1);

	ros::Rate rate(10);

	uint64_t i = 1;

	while (ros::ok())
	{
		std_msgs::UInt64 laser_command;	

		
		laser_command.data = i;

		if (i == 32){
		 i = 1;
	     	}

		pub.publish(laser_command);
		i++;
		rate.sleep();
	}
}

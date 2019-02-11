#include <iostream>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Depth_check");
    ros::NodeHandle DP_nh;
    std_msgs::Float32 toPub;
    // Subscriber Declaration...
    //ros::Subscriber FC_coordinate_sub = FC_nh.subscribe("/position", 10, object_position);
    // Publisher Declaration...
    ros::Publisher DP_pub = DP_nh.advertise<std_msgs::Float32MultiArray>("/ObjectDepth",10);
    ros::Rate rate(10);

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start();

    while (true)
    {
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        ros::spinOnce();

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame();

        // Get the depth frame's dimensions
        float width = depth.get_width();
        float height = depth.get_height();

        // Query the distance from the camera to the object in the center of the image
        float dist_to_center = depth.get_distance(width / 2, height / 2);
        toPub.data = depth.get_distance(width / 2, height / 2);

        // Print the distance
        std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";
        
        DP_pub.publish(dist_to_center);
        rate.sleep();
    }
}

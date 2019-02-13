#include <iostream>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>

std_msgs::Float32MultiArray out;

//void object_array_builder(const vision_msgs::Detection2DArray::ConstPtr& data)
//{
    
//}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Depth_check");
    ros::NodeHandle DP_nh;
    std_msgs::Float32 toPub;
    // Subscriber Declaration...
    //ros::Subscriber DP_coordinate_sub = DP_nh.subscribe("/objects", 10, object_array_builder);
    // Publisher Declaration...
    ros::Publisher DP_pub = DP_nh.advertise<std_msgs::Float32>("/ObjectDepth",10);
    ros::Rate rate(10);

    // Find RS device
    rs2::context ctx;
    auto list = ctx.query_devices(); //Get snapshot of connected devices
    if(list.size() == 0)
        throw std::runtime_error("No RS device detected. Is it plugged in?");
    rs2::device dev = list.front();

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    // Configure and start the pipeline
    p.start();

    while (ros::ok())
    {
        // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        ros::spinOnce();

        // Try to get frames of a depth and color image
        rs2::frame color_frame = frames.get_color_frame();
        rs2::depth_frame depth = frames.get_depth_frame();

        // Get the depth frame's dimensions
        float width = depth.get_width();
        float height = depth.get_height();

        // Generate depthcloud object
        rs2::pointcloud pc;
        rs2::points points = pc.calculate(depth);
        pc.map_to(color_frame);

        // Query the distance from the camera to the object in the center of the image
        float dist_to_center = depth.get_distance(width / 2, height / 2);
        toPub.data = depth.get_distance(width / 2, height / 2);

        // Print the distance
        std::cout << "The camera is facing an object " << dist_to_center << " meters away \n\r";

        

        // Print the distance from PC
        std::cout << "The camera is facing an object " << dist_to_center << " meters away \n\r";
        
        DP_pub.publish(toPub);
        rate.sleep();
    }
}

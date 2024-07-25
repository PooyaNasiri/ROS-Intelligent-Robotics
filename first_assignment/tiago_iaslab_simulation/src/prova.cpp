#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    // Print information from the LaserScan message
    ROS_INFO("Received LaserScan message:");
    ROS_INFO("Header: %s", scan_msg->header.frame_id.c_str());
    ROS_INFO("Angle Min: %f", scan_msg->angle_min);
    ROS_INFO("Angle Max: %f", scan_msg->angle_max);
    ROS_INFO("Angle Increment: %f", scan_msg->angle_increment);
    ROS_INFO("Time Increment: %f", scan_msg->time_increment);
    ROS_INFO("Scan Time: %f", scan_msg->scan_time);
    ROS_INFO("Range Min: %f", scan_msg->range_min);
    ROS_INFO("Range Max: %f", scan_msg->range_max);

    // Print the range data
    ROS_INFO("Ranges:");
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        ROS_INFO("[%zu] %f", i, scan_msg->ranges[i]);
    }
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "scan_listener");

    // Create a ROS node handle
    ros::NodeHandle nh;

    // Subscribe to the /scan topic
    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);

    // Spin to allow callback function to run
    ros::spin();

    return 0;
}


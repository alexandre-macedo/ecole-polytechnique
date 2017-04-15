#include <ros/ros.h>

int main (int argc, char** argv)
{
    ros::init(argc, argv, "node_name");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("This node does nothing.");
}

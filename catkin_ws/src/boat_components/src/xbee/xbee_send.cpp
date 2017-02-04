#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <init_param.h>

serial::Serial ser;

void writeCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO_STREAM("Writing to serial port:" << msg->data);
    ser.write(msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xbee_send");
    ros::NodeHandle nh;
    const std::string node_name = ros::this_node::getName();

    // Get parameters from server/
    const std::vector<std::string> parameters_names = {"port", "baud_rate", "timeout"};
    const std::vector<std::string> parameters_default = {"NODEFAULT", "9600", "1000"};
    
    int parameters_number = parameters_names.size();
    std::vector<std::string> parameters(parameters_number);
    if (!initParam(parameters_names, parameters_default, node_name, parameters, parameters_number))
        return -1;

    // Start serial communication with Xbee
    try
    {
        ser.setPort(parameters[0]);
        ser.setBaudrate(std::stoul(parameters[1]));
        serial::Timeout to = serial::Timeout::simpleTimeout(std::stoul(parameters[2]));
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Could not connect to Xbee.");
        return -1;
    }

    if (ser.isOpen())
        ROS_INFO_STREAM("Serial Port initialized.");
    else
    {
        ROS_ERROR_STREAM("Unable to initialize port.");
        return -1;
    }

    // Subscribe to topics
    ros::Subscriber send_sub = nh.subscribe(node_name + "/send", 2, writeCallback);

    // Run ros
    ros::spin();
}
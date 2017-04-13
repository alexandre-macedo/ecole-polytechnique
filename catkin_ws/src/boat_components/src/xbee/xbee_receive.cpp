#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <btmx/init_param.h>

serial::Serial ser;

void writeCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO_STREAM("Writing to serial port:" << msg->data);
    ser.write(msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xbee_receive");
    ros::NodeHandle nh;
    const std::string node_name = ros::this_node::getName();

    // Get parameters from server/
    const std::vector<std::string> parameters_names = {"port", "baud_rate", "timeout", "loop_rate"};
    const std::vector<std::string> parameters_default = {"NODEFAULT", "9600", "1000", "60"};

    std::vector<std::string> parameters(parameters_names.size());
    if (!btmx::initParam(parameters, parameters_names, parameters_default, node_name))
        return 1;

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
        return 1;
    }

    if (ser.isOpen())
        ROS_INFO_STREAM("Serial port initialized.");
    else
    {
        ROS_ERROR_STREAM("Unable to initialize serial port.");
        return 1;
    }

    // Read from serial and publish information

    ros::Publisher read_pub = nh.advertise<std_msgs::String>(node_name + "/commands" , 1000);
    ros::Rate loop_rate(std::stoul(parameters[3]));
    std_msgs::String result;
    while (ros::ok())
    {
        ros::spinOnce();
        if (ser.available())
        {
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Reading from serial port");
            ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(result);
        }
        loop_rate.sleep();
    }
}
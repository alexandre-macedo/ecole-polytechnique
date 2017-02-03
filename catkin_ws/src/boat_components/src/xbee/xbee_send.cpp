#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO_STREAM("Writing to serial port:" << msg->data);
    ser.write(msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xbee_send");
    ros::NodeHandle nh;

    // Get parameters from server/

    const std::string parameters_name[] = {"port", "baud_rate", "timeout"};
    const std::string parameters_default[] = {"NODEFAULT", "9600", "1000"};
    std::string parameters[3];

    bool erro_flag = false;
    const std::string node_name = ros::this_node::getName();
    ROS_INFO_STREAM("Initializing parameter for " << node_name);
    for (unsigned int i = 0; i < sizeof(parameters_name) / sizeof(parameters_name[0]); i++)
    {
        if (nh.getParam(node_name + "/" + parameters_name[i], parameters[i]))
            ROS_INFO_STREAM("Parameter " << node_name + "/" + parameters_name[i] << " initialized to " << parameters[i] << ".");
        else
        {
            if (parameters_default[i] == "NODEFAULT")
            {
                ROS_ERROR_STREAM("Parameter " << node_name + "/" + parameters_name[i] << " not found.");
                erro_flag = true;
            }
            else
            {
                parameters[i] = parameters_default[i];
                ROS_WARN_STREAM("Parameter " << node_name + "/" + parameters_name[i] << " not found. Initializing to default value " << parameters_default[i] << ".");
            }
        }
    }
    if (erro_flag){
        ROS_ERROR_STREAM("Invalid parameter initialization. Consider solving it before continuing. Node " << node_name << " has died.");
        return -1;
    }
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
    ros::Subscriber send_sub = nh.subscribe(node_name + "/send", 2, write_callback);

    // Run ros
    ros::spin();
}
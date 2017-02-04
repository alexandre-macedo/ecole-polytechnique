#include <ros/ros.h>
#include <init_param.h>

bool initParam(const std::vector<std::string> &parameters_names, const std::vector<std::string> &parameters_default, const std::string &node_name, std::vector<std::string> &parameters, const int &parameters_number)
{
    if (parameters_names.size() != parameters_number || parameters_default.size() != parameters_number || parameters.size() != parameters_number)
    {
        ROS_ERROR_STREAM("Invalid argument sizes. Parameters not initialized. Check your code.");
        return false;
    }
    bool error_flag = false;
    ROS_INFO_STREAM("Initializing parameter for " << node_name);
    for (unsigned int i = 0; i < parameters_number; i++)
    {
        if (ros::param::get(node_name + "/" + parameters_names[i], parameters[i]))
            ROS_INFO_STREAM("Parameter " << node_name + "/" + parameters_names[i] << " initialized to " << parameters[i] << ".");
        else
        {
            if (parameters_default[i] == "NODEFAULT")
            {
                ROS_ERROR_STREAM("Parameter " << node_name + "/" + parameters_names[i] << " not found.");
                error_flag = true;
            }
            else
            {
                parameters[i] = parameters_default[i];
                ROS_WARN_STREAM("Parameter " << node_name + "/" + parameters_names[i] << " not found. Initializing to default value " << parameters_default[i] << ".");
            }
        }
    }
    if (error_flag)
    {
        ROS_ERROR_STREAM("Invalid parameter initialization. Consider solving it before continuing. Node " << node_name << " should die.");
        return false;
    }
    return true;
}
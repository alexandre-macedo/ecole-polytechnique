#ifndef INIT_PARAM_H
#define INIT_PARAM_H

#include <ros/ros.h>

namespace btmx
{
bool initParam(std::vector<std::string> &parameters, const std::vector<std::string> &parameters_names, const std::vector<std::string> &parameters_default, const std::string &node_name);

bool initParam(std::vector<std::string> &parameters, const std::vector<std::string> &parameters_names, const std::vector<std::string> &parameters_default);

template <typename T>
void initParam(T &parameter, const std::string &parameter_name, const T &default_value, const std::string &node_name)
{
    if (ros::param::get(parameter_name, parameter))
    {
        ROS_INFO_STREAM("Parameter " << node_name + "/" + parameter_name << " initialized to " << parameter << ".");
    }
    else
    {
        parameter = default_value;
        ROS_WARN_STREAM("Parameter " << node_name + "/" + parameter_name << " not found. Initializing to default value " << default_value << ".");
    }
}

template <typename T>
void initParam(T &parameter, const std::string &parameter_name, const T &default_value)
{
    if (ros::param::get(parameter_name, parameter))
    {

        ROS_INFO_STREAM("Parameter " << parameter_name << " initialized to " << parameter << ".");
    }
    else
    {
        parameter = default_value;
        ROS_WARN_STREAM("Parameter " << parameter_name << " not found. Initializing to default value " << default_value << ".");
    }
}

template <typename T>
bool initParam(T &parameter, const std::string &parameter_name, const std::string &node_name)
{
    if (ros::param::get(node_name + "/" + parameter_name, parameter))
    {
        ROS_INFO_STREAM("Parameter " << node_name + "/" + parameter_name << " initialized to " << parameter << ".");
        return true;
    }
    else
        ROS_ERROR_STREAM("Invalid parameter initialization for " << parameter_name << ". Consider solving it before continuing.");
    return false;
}

template <typename T>
bool initParam(T &parameter, const std::string &parameter_name)
{
    if (ros::param::get(parameter_name, parameter))
    {
        ROS_INFO_STREAM("Parameter " << parameter_name << " initialized to " << parameter << ".");
        return true;
    }
    else
    {
        ROS_ERROR_STREAM("Invalid parameter initialization for " << parameter_name << ". Consider solving it before continuing.");
        return false;
    }
}
}
#endif
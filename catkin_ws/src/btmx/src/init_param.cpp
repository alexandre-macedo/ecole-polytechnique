#include <ros/ros.h>
#include <btmx/init_param.h>

namespace btmx
{
bool initParam(const std::vector<std::string> &parameters_names, const std::vector<std::string> &parameters_default, std::vector<std::string> &parameters, const int &parameters_number, const std::string &node_name)
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
        ROS_ERROR_STREAM("Invalid parameter initialization. Consider solving it before continuing.");
        return false;
    }
    return true;
}

bool initParam(const std::vector<std::string> &parameters_names, const std::vector<std::string> &parameters_default, std::vector<std::string> &parameters, const int &parameters_number)
{
    if (parameters_names.size() != parameters_number || parameters_default.size() != parameters_number || parameters.size() != parameters_number)
    {
        ROS_ERROR_STREAM("Invalid argument sizes. Parameters not initialized. Check your code.");
        return false;
    }
    bool error_flag = false;
    ROS_INFO_STREAM("Initializing parameter");
    for (unsigned int i = 0; i < parameters_number; i++)
    {
        if (ros::param::get(parameters_names[i], parameters[i]))
            ROS_INFO_STREAM("Parameter " << parameters_names[i] << " initialized to " << parameters[i] << ".");
        else
        {
            if (parameters_default[i] == "NODEFAULT")
            {
                ROS_ERROR_STREAM("Parameter " << parameters_names[i] << " not found.");
                error_flag = true;
            }
            else
            {
                parameters[i] = parameters_default[i];
                ROS_WARN_STREAM("Parameter " << parameters_names[i] << " not found. Initializing to default value " << parameters_default[i] << ".");
            }
        }
    }
    if (error_flag)
    {
        ROS_ERROR_STREAM("Invalid parameter initialization. Consider solving it before continuing.");
        return false;
    }
    return true;
}

template <typename T>
T initParam(const std::string &parameter_name, const T &default_value, const std::string &node_name)
{
    T v;
    if (ros::param::get(parameter_name, v))
    {
        ROS_INFO_STREAM("Parameter " << node_name + "/" + parameter_name << " initialized to " << v << ".");
        return v;
    }
    else
        ROS_WARN_STREAM("Parameter " << node_name + "/" + parameter_name << " not found. Initializing to default value " << default_value << ".");
    return default_value;
}

template <typename T>
T initParam(const std::string &parameter_name, const std::string &node_name)
{
    T v;
    if (ros::param::get(node_name + "/" + parameter_name, v))
    {
        ROS_INFO_STREAM("Parameter " << node_name + "/" + parameter_name << " initialized to " << v << ".");
        return v;
    }
    else
        ROS_ERROR_STREAM("Invalid parameter initialization for " << parameter_name << ". Consider solving it before continuing.");
    return T();
}

template <typename T>
T initParam(const std::string &parameter_name, const T &default_value)
{
    T v;
    if (ros::param::get(parameter_name, v))
    {
        ROS_INFO_STREAM("Parameter " << parameter_name << " initialized to " << v << ".");
        return v;
    }
    else
        ROS_WARN_STREAM("Parameter " << parameter_name << " not found. Initializing to default value " << default_value << ".");
    return default_value;
}

template <typename T>
T initParam(const std::string &parameter_name)
{
    T v;
    if (ros::param::get(parameter_name, v))
    {
        ROS_INFO_STREAM("Parameter " << parameter_name << " initialized to " << v << ".");
        return v;
    }
    else
        ROS_ERROR_STREAM("Invalid parameter initialization for " << parameter_name << ". Consider solving it before continuing.");
    return T();
}
}
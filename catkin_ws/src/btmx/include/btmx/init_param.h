#ifndef INIT_PARAM_H
#define INIT_PARAM_H

namespace ipl
{
bool initParam(const std::vector<std::string> &parameters_names, const std::vector<std::string> &parameters_default, std::vector<std::string> &parameters, const int &parameters_number, const std::string &node_name);

bool initParam(const std::vector<std::string> &parameters_names, const std::vector<std::string> &parameters_default, std::vector<std::string> &parameters, const int &parameters_number);

template<typename T>
T getParam(const std::string &parameter_name, const T &default_value);

template<typename T>
T getParam(const std::string &parameter_name, const T &default_value, const std::string &node_name);

template<typename T>
T getParam(const std::string &parameter_name);

template<typename T>
T getParam(const std::string &parameter_name, const std::string &node_name);
}
#endif
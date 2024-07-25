#ifndef ROS2_SPDLOG_LOGGER_ROS2_SPDLOG_LOGGER_EXCEPTIONS_HPP_
#define ROS2_SPDLOG_LOGGER_ROS2_SPDLOG_LOGGER_EXCEPTIONS_HPP_
#include <stdexcept>

namespace ros2_spdlog_logger
{

class LoggingAlreadyInitialized : public std::runtime_error{
public:
LoggingAlreadyInitialized(): std::runtime_error("Logging system has already been initialized"){}
};

class LoggingNotInitialized : public std::runtime_error{
public:
LoggingNotInitialized(): std::runtime_error("Logging system was not initialized"){}
};

}

#endif // ROS2_SPDLOG_LOGGER_ROS2_SPDLOG_LOGGER_EXCEPTIONS_HPP_

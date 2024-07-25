#ifndef ROS2_SPDLOG_LOGGER__ROS2_SPDLOG_LOGGER_HPP_
#define ROS2_SPDLOG_LOGGER__ROS2_SPDLOG_LOGGER_HPP_

#include <spdlog/common.h>
#include <spdlog/spdlog.h>
#include <memory>
#include <rclcpp/utilities.hpp>

#include "ros2_spdlog_logger/visibility_control.h"
#include "ros2_spdlog_logger/logging_macros.hpp"

namespace ros2_spdlog_logger
{

using Logger = spdlog::logger;
using LoggerPtr = std::shared_ptr<Logger>;

/**
  * Initializes the rclcpp global context and sets up our custom logging functions.
  */
ROS2_SPDLOG_LOGGER_PUBLIC void init(
  int argc,
  char const * const * argv,
  const rclcpp::InitOptions & init_options = rclcpp::InitOptions(),
  rclcpp::SignalHandlerOptions signal_handler_options = rclcpp::SignalHandlerOptions::All);

/**
  * Initializes the rclcpp global context and sets up our custom logging functions.
  * Returns all non-ROS arguments, as if you called rclcpp::init_and_remove_ros_arguments().
  */
ROS2_SPDLOG_LOGGER_PUBLIC std::vector<std::string> init_and_remove_ros_arguments(
  int argc,
  char const * const * argv,
  const rclcpp::InitOptions & init_options = rclcpp::InitOptions());

/**
 * Get a logger instance with the given logger name, using default settings.
*/
ROS2_SPDLOG_LOGGER_PUBLIC LoggerPtr get_logger(const std::string & logger_name = DEFAULT_LOGGER);

/**
 * Add sink to the list of global sinks.
 * This function can only be used before initializing the logging system, otherwise it will throw.
*/
ROS2_SPDLOG_LOGGER_PUBLIC void add_sink(spdlog::sink_ptr sinks);

/**
 * Initialize the sinks to be used in the logging system.
 * This allows you to provide custom sinks for added functionality.
*/
template<typename ...TSinks>
ROS2_SPDLOG_LOGGER_PUBLIC void add_sinks(TSinks... sinks){
  (add_sink(sinks), ...);
}
}  // namespace ros2_spdlog_logger

#endif  // ROS2_SPDLOG_LOGGER__ROS2_SPDLOG_LOGGER_HPP_

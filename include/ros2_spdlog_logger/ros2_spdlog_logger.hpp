#ifndef ROS2_SPDLOG_LOGGER__ROS2_SPDLOG_LOGGER_HPP_
#define ROS2_SPDLOG_LOGGER__ROS2_SPDLOG_LOGGER_HPP_

#include <spdlog/spdlog.h>
#include <memory>
#include <string_view>
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
ROS2_SPDLOG_LOGGER_PUBLIC void init_with_global_context(
  int argc,
  char const * const * argv,
  const rclcpp::InitOptions & init_options = rclcpp::InitOptions(),
  rclcpp::SignalHandlerOptions signal_handler_options = rclcpp::SignalHandlerOptions::All);

/**
 * Get a logger instance with the given logger name, using default settings.
*/
ROS2_SPDLOG_LOGGER_PUBLIC LoggerPtr get_logger(const std::string & logger_name = DEFAULT_LOGGER);

/**
 * Calls the underlying shutdown routines for the logging libraries and destroys the sinks.
 * Should not be required to be called so long as the default rclcpp::Context is destroyed properly.
*/
ROS2_SPDLOG_LOGGER_PUBLIC void shutdown();

}  // namespace ros2_spdlog_logger

#endif  // ROS2_SPDLOG_LOGGER__ROS2_SPDLOG_LOGGER_HPP_

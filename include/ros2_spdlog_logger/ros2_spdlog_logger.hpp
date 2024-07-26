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
  * Initializes the rclcpp::DefaultContext and sets up our custom logging functions.
  * It is recommended to not use hook_on_shutdown and instead use the ros2_spdlog_logger::shutdown().
  */
ROS2_SPDLOG_LOGGER_PUBLIC void init(
  int argc,
  char const * const * argv,
  bool hook_on_shutdown = true,
  const rclcpp::InitOptions & init_options = rclcpp::InitOptions(),
  rclcpp::SignalHandlerOptions signal_handler_options = rclcpp::SignalHandlerOptions::All);

/**
  * Initializes the rclcpp::DefaultContext and sets up our custom logging functions.
  * Returns all non-ROS arguments, as if you called rclcpp::init_and_remove_ros_arguments().
  * It is recommended to not use hook_on_shutdown and instead use the ros2_spdlog_logger::shutdown().
  */
ROS2_SPDLOG_LOGGER_PUBLIC std::vector<std::string> init_and_remove_ros_arguments(
  int argc,
  char const * const * argv,
  bool hook_on_shutdown = false,
  const rclcpp::InitOptions & init_options = rclcpp::InitOptions());

/**
 * Shutdown the logging system and optionally rclcpp::DefaultContext.
 * It is recommended to wrap the shutdown with a scope guard to ensure its execution.
*/
ROS2_SPDLOG_LOGGER_PUBLIC void shutdown(bool shutdown_rclcpp_context = false, const std::string& reason_for_shutdown = "");

/**
 * Get a logger instance with the given logger name, using default settings.
*/
ROS2_SPDLOG_LOGGER_PUBLIC LoggerPtr get_logger(const std::string & logger_name = DEFAULT_LOGGER);

/**
 * Add sink to the list of global sinks.
 * This allows you to provide custom sinks for added functionality.
 * This function can only be used before initializing the logging system, otherwise it will throw.
*/
ROS2_SPDLOG_LOGGER_PUBLIC void add_sink(spdlog::sink_ptr sinks);

/**
 * Add sinks to the list of global sinks.
 * This allows you to provide custom sinks for added functionality.
 * This function can only be used before initializing the logging system, otherwise it will throw.
*/
template<typename ...TSinks>
ROS2_SPDLOG_LOGGER_PUBLIC void add_sinks(TSinks... sinks){
  (add_sink(sinks), ...);
}
}  // namespace ros2_spdlog_logger

#endif  // ROS2_SPDLOG_LOGGER__ROS2_SPDLOG_LOGGER_HPP_

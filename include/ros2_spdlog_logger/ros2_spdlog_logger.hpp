#ifndef ROS2_SPDLOG_LOGGER__ROS2_SPDLOG_LOGGER_HPP_
#define ROS2_SPDLOG_LOGGER__ROS2_SPDLOG_LOGGER_HPP_

#include "spdlog/spdlog.h"

// add aliases for spdlog levels
#define ROS2_SPDLOG_TRACE_LEVEL SPDLOG_LEVEL_TRACE
#define ROS2_SPDLOG_DEBUG_LEVEL SPDLOG_LEVEL_DEBUG
#define ROS2_SPDLOG_INFO_LEVEL SPDLOG_LEVEL_INFO
#define ROS2_SPDLOG_WARN_LEVEL SPDLOG_LEVEL_WARN
#define ROS2_SPDLOG_ERROR_LEVEL SPDLOG_LEVEL_ERROR
#define ROS2_SPDLOG_FATAL_LEVEL SPDLOG_LEVEL_CRITICAL
#define ROS2_SPDLOG_UNSET_LEVEL SPDLOG_LEVEL_OFF

// if not defined use default spdlog level
#ifndef ROS2_SPDLOG_DEFAULT_PATTERN
// use default spdlog pattern
#define ROS2_SPDLOG_DEFAULT_PATTERN "%+"
#endif

// if not defined, set it to info
#ifndef ROS2_SPDLOG_ACTIVE_LEVEL
#define ROS2_SPDLOG_ACTIVE_LEVEL ROS2_SPDLOG_INFO_LEVEL
#endif

#define DEFAULT_LOGGER "default"

#include <memory>
#include <string_view>
#include "rclcpp/utilities.hpp"

#define RUN_ONCE(runner) \
  do{ \
    static std::atomic<bool> has_executed_before = false; \
    if (RCUTILS_UNLIKELY(!has_executed_before)) { \
      has_executed_before = true; \
      runner; \
    } \
  } while(0)

#define RUN_THROTTLE(duration, runner) \
  do{ \
    static std::atomic<bool> has_executed_before = false; \
    static std::atomic<std::chrono::steady_clock::rep> last_execution_time = \
      std::chrono::steady_clock::now(); \
    if (RCUTILS_UNLIKELY(!has_executed_before || \
      ((std::chrono::steady_clock::now() - last_execution_time) >= duration))) { \
      has_executed_before = true; \
      last_execution_time = std::chrono::steady_clock::now(); \
      runner; \
    } \
  } while(0)

#if ROS2_SPDLOG_ACTIVE_LEVEL <= ROS2_SPDLOG_TRACE_LEVEL
#define LOG_TRACE(logger, ...) SPDLOG_LOGGER_CALL( \
    logger, spdlog::level::level_enum::trace, \
    __VA_ARGS__)
#define LOG_TRACE_ONCE(...) RUN_ONCE(LOG_TRACE(__VA_ARGS__))
#define LOG_TRACE_THROTTLE(duration, ...) RUN_THROTTLE(duration, LOG_TRACE(__VA_ARGS__))
#else
#define LOG_TRACE(logger, ...) ((void)0)
#define LOG_TRACE_ONCE(...) ((void)0)
#define LOG_TRACE_THROTTLE(duration, ...) ((void)0)
#endif

#if ROS2_SPDLOG_ACTIVE_LEVEL <= ROS2_SPDLOG_DEBUG_LEVEL
#define LOG_DEBUG(logger, ...) SPDLOG_LOGGER_CALL( \
    logger, spdlog::level::level_enum::debug, \
    __VA_ARGS__)
#define LOG_DEBUG_ONCE(...) RUN_ONCE(LOG_DEBUG(__VA_ARGS__))
#define LOG_DEBUG_THROTTLE(duration, ...) RUN_THROTTLE(duration, LOG_DEBUG(__VA_ARGS__))
#else
#define LOG_DEBUG(logger, ...) ((void)0)
#define LOG_DEBUG_ONCE(...) ((void)0)
#define LOG_DEBUG_THROTTLE(duration, ...) ((void)0)
#endif


#if ROS2_SPDLOG_ACTIVE_LEVEL <= ROS2_SPDLOG_INFO_LEVEL
#define LOG_INFO(logger, ...) SPDLOG_LOGGER_CALL( \
    logger, spdlog::level::level_enum::info, \
    __VA_ARGS__)
#define LOG_INFO_ONCE(...) RUN_ONCE(LOG_INFO(__VA_ARGS__))
#define LOG_INFO_THROTTLE(duration, ...) RUN_THROTTLE(duration, LOG_INFO(__VA_ARGS__))
#else
#define LOG_INFO(logger, ...) ((void)0)
#define LOG_INFO_ONCE(...) ((void)0)
#define LOG_INFO_THROTTLE(duration, ...) ((void)0)
#endif

#if ROS2_SPDLOG_ACTIVE_LEVEL <= ROS2_SPDLOG_WARN_LEVEL
#define LOG_WARN(logger, ...) SPDLOG_LOGGER_CALL( \
    logger, spdlog::level::level_enum::warn, \
    __VA_ARGS__)
#define LOG_WARN_ONCE(...) RUN_ONCE(LOG_WARN(__VA_ARGS__))
#define LOG_WARN_THROTTLE(duration, ...) RUN_THROTTLE(duration, LOG_WARN(__VA_ARGS__))
#else
#define LOG_WARN(logger, ...) ((void)0)
#define LOG_WARN_ONCE(...) ((void)0)
#define LOG_WARN_THROTTLE(duration, ...) ((void)0)
#endif


#if ROS2_SPDLOG_ACTIVE_LEVEL <= ROS2_SPDLOG_ERROR_LEVEL
#define LOG_ERROR(logger, ...) SPDLOG_LOGGER_CALL( \
    logger, spdlog::level::level_enum::err, \
    __VA_ARGS__)
#define LOG_ERROR_ONCE(...) RUN_ONCE(LOG_ERROR(__VA_ARGS__))
#define LOG_ERROR_THROTTLE(duration, ...) RUN_THROTTLE(duration, LOG_ERROR(__VA_ARGS__))
#else
#define LOG_ERROR(logger, ...)((void)0)
#define LOG_ERROR_ONCE(...) ((void)0)
#define LOG_ERROR_THROTTLE(duration, ...) ((void)0)
#endif

#if ROS2_SPDLOG_ACTIVE_LEVEL <= ROS2_SPDLOG_FATAL_LEVEL
#define LOG_FATAL(logger, ...) SPDLOG_LOGGER_CALL( \
    logger, spdlog::level::level_enum::critical, \
    __VA_ARGS__)
#define LOG_FATAL_ONCE(...) RUN_ONCE(LOG_FATAL(__VA_ARGS__))
#define LOG_FATAL_THROTTLE(duration, ...) RUN_THROTTLE( \
    duration, \
    LOG_FATAL(__VA_ARGS__))
#else
#define LOG_FATAL(logger, ...)((void)0)
#define LOG_FATAL_ONCE(...) ((void)0)
#define LOG_FATAL_THROTTLE(duration, ...)((void)0)
#endif

#include "ros2_spdlog_logger/visibility_control.h"

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
#include <chrono>
#include <filesystem>
#include <cinttypes>
#include <cstdint>

#include <rcutils/allocator.h>
#include <rcutils/logging.h>
#include <rcutils/process.h>
#include <rcutils/snprintf.h>
#include <rcutils/strdup.h>
#include <rcutils/time.h>

#include <rcl/init.h>
#include <rcl/logging.h>
#include <rcl_logging_interface/rcl_logging_interface.h>

#include <rclcpp/exceptions.hpp>
#include <rclcpp/contexts/default_context.hpp>
#include <rcpputils/scope_exit.hpp>

#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

#include "ros2_spdlog_logger/ros2_spdlog_logger.hpp"

namespace
{
// list of sinks to be used
static std::vector<spdlog::sink_ptr> sink_list = {};

ROS2_SPDLOG_LOGGER_LOCAL void create_sinks(const std::string & file_name)
{
  // move sinks to sink list
  sink_list = {
    std::make_shared<spdlog::sinks::stdout_color_sink_mt>(),
    std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_name, false),
  };
}

ROS2_SPDLOG_LOGGER_LOCAL std::vector<spdlog::sink_ptr> get_sinks()
{
  if (sink_list.empty()) {
    throw std::runtime_error("Sinks were not initialized!");
  }

  return sink_list;
}

ROS2_SPDLOG_LOGGER_LOCAL ros2_spdlog_logger::LoggerPtr create_logger(
  const std::string & logger_name)
{
  // initialize sinks
  const auto sinks = get_sinks();

#ifndef DISABLE_ASYNC_LOGGER
  auto logger = std::make_shared<spdlog::async_logger>(
    logger_name, sinks.cbegin(),
    sinks.cend(), spdlog::thread_pool(), spdlog::async_overflow_policy::block);
#else
  auto logger = std::make_shared<spdlog::logger>(
    logger_name, sinks.cbegin(),
    sinks.cend());
#endif // !DISABLE_ASYNC_LOGGER

  // set default pattern and register logger
  logger->set_level(static_cast<spdlog::level::level_enum>(SPDLOG_ACTIVE_LEVEL));
  logger->set_pattern(ROS2_SPDLOG_DEFAULT_PATTERN);
  spdlog::register_logger(logger);

  return logger;
}
ROS2_SPDLOG_LOGGER_LOCAL [[maybe_unused]] spdlog::level::level_enum to_spdlog_severity(
  int external_level)
{
  switch (external_level) {
    case RCUTILS_LOG_SEVERITY_DEBUG:
      return spdlog::level::level_enum::debug;
    case RCUTILS_LOG_SEVERITY_INFO:
      return spdlog::level::level_enum::info;
    case RCUTILS_LOG_SEVERITY_WARN:
      return spdlog::level::level_enum::warn;
    case RCUTILS_LOG_SEVERITY_ERROR:
      return spdlog::level::level_enum::err;
    case RCUTILS_LOG_SEVERITY_FATAL:
      return spdlog::level::level_enum::critical;
    default:
      return spdlog::level::level_enum::off;
  }
}

// function adapted from the official rcl_logging spdlog sink
ROS2_SPDLOG_LOGGER_LOCAL std::string get_log_filename()
{
  const auto allocator = rcutils_get_default_allocator();
  // To be compatible with ROS 1, we construct a default filename of
  // the form ~/.ros/log/<exe>_<pid>_<milliseconds-since-epoch>.log

  char * logdir = nullptr;
  rcl_logging_ret_t dir_ret = rcl_logging_get_logging_directory(allocator, &logdir);
  if (RCL_LOGGING_RET_OK != dir_ret) {
    // We couldn't get the log directory, so get out of here without setting up
    // logging.
    rclcpp::exceptions::throw_from_rcl_error(dir_ret, "Failed to get logging directory");
  }
  RCPPUTILS_SCOPE_EXIT(
    {
      allocator.deallocate(logdir, allocator.state);
    });

  std::error_code ec;
  std::filesystem::path logdir_path(logdir);

  std::filesystem::create_directories(logdir_path, ec);
  // create_directories returns true if it created the directory, and false if it did not.
  // This behavior is maintained regardless of whether an error occurred.  Since we don't
  // actually care whether the directory was created, we only check for errors.
  if (ec.value() != 0) {
    rclcpp::exceptions::throw_from_rcl_error(
      RCL_LOGGING_RET_ERROR,
      "Failed to create log directory");
  }

  // Now get the milliseconds since the epoch in the local timezone.
  rcutils_time_point_value_t now;
  rcutils_ret_t ret = rcutils_system_time_now(&now);
  if (ret != RCUTILS_RET_OK) {
    // We couldn't get the system time, so get out of here without setting up
    // logging.  We don't need to call RCUTILS_SET_ERROR_MSG either since
    // rcutils_system_time_now() already did it.
    rclcpp::exceptions::throw_from_rcl_error(RCL_LOGGING_RET_ERROR, "Failed to get system time");
  }
  int64_t ms_since_epoch = RCUTILS_NS_TO_MS(now);

  char * basec = rcutils_get_executable_name(allocator);

  if (basec == nullptr) {
    // We couldn't get the program name, so get out of here without setting up
    // logging.
    rclcpp::exceptions::throw_from_rcl_error(
      RCL_LOGGING_RET_ERROR,
      "Failed to get the executable name");
  }
  RCPPUTILS_SCOPE_EXIT(
    {
      allocator.deallocate(basec, allocator.state);
    });
  char name_buffer[4096] = {0};
  int print_ret = rcutils_snprintf(
    name_buffer, sizeof(name_buffer),
    "%s/%s_%i_%" PRId64 ".log", logdir,
    basec, rcutils_get_pid(), ms_since_epoch);
  if (print_ret < 0) {
    rclcpp::exceptions::throw_from_rcl_error(
      RCL_LOGGING_RET_ERROR,
      "Failed to create log file name string");
  }

  return std::string(name_buffer, print_ret);
}

extern "C" {
ROS2_SPDLOG_LOGGER_PUBLIC void logging_sink_output_handler(
  const rcutils_log_location_t * location,
  [[maybe_unused]] int severity,
  const char * name,
  rcutils_time_point_value_t timestamp,
  const char * format,
  va_list * args)
{

  // build message using the funcs that would be used by rcl
  const auto default_allocator = rcutils_get_default_allocator();
  thread_local std::array<char, 1024> msg_buf = {0};
  rcutils_char_array_t msg_array = {
    msg_buf.data(),
    false,
    0u,
    msg_buf.size(),
    default_allocator
  };

  //format message
  rcl_ret_t status = rcutils_char_array_vsprintf(&msg_array, format, *args);

  RCPPUTILS_SCOPE_EXIT(
      {
        rcl_ret_t status_fini = rcutils_char_array_fini(&msg_array);
        if (RCL_RET_OK != status_fini) {
          RCUTILS_SAFE_FWRITE_TO_STDERR("failed to finalize char array: ");
          RCUTILS_SAFE_FWRITE_TO_STDERR(rcl_get_error_string().str);
          rcl_reset_error();
          RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
        }
      }
  );

  if (status == RCL_RET_OK) {
    // get logger and parse attributes
    auto logger = ros2_spdlog_logger::get_logger(std::string(name));

    // build spdlog compatible loc data
    auto spdlog_loc = spdlog::source_loc(
      location->file_name, location->line_number,
      location->function_name);

    // convert timestamp to chrono timepoint
    const auto timestamp_ns = std::chrono::nanoseconds(timestamp);
    const auto spdlog_tp = std::chrono::time_point<std::chrono::system_clock>(
      timestamp_ns);

    // finally, log the message
    logger->log(
      spdlog_tp, spdlog_loc,
      to_spdlog_severity(severity), std::string(msg_array.buffer, msg_array.buffer_length));
  } else {
    RCUTILS_SAFE_FWRITE_TO_STDERR("failed to format user log message: ");
    RCUTILS_SAFE_FWRITE_TO_STDERR(rcl_get_error_string().str);
    rcl_reset_error();
    RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
  }

}
}
}

namespace ros2_spdlog_logger
{


void init_with_global_context(
  int argc,
  char const * const * argv,
  const rclcpp::InitOptions & init_options,
  rclcpp::SignalHandlerOptions signal_handler_options)
{
  // prevent concurrent calls to this function
  static std::mutex init_mux;
  std::lock_guard lock(init_mux);

  // prevent multiple initializations
  static bool has_been_initialized = false;
  if (!has_been_initialized) {
    has_been_initialized = true;

    // copy the init options but disable logging
    auto changed_init_options = init_options;
    changed_init_options.auto_initialize_logging(false);

    // init all rclcpp things without logging
    rclcpp::init(
      argc, argv, changed_init_options, signal_handler_options
    );


    // initialize spdlog default thread pool if async (1 threads with 8k size queue to ensure ordering)
    #ifndef DISABLE_ASYNC_LOGGER
    spdlog::init_thread_pool(8192, 1);
    #endif// !DISABLE_ASYNC_LOGGER

    // get the filename for logging and initialize our spdlog sinks
    create_sinks(get_log_filename());

    auto logger = create_logger(DEFAULT_LOGGER);
    spdlog::set_default_logger(logger);

    // setup a post-shutdown callback to destroy all logging things beforehand
    rclcpp::on_shutdown(
      []() {
        ros2_spdlog_logger::shutdown();
    }
    );

    // set the output handler for logging using our custom sink
    rcutils_logging_set_output_handler(logging_sink_output_handler);

  } else {
    get_logger("rclcpp")->warn("logging was initialized more than once");
  }
}

void shutdown()
{
  spdlog::shutdown();
  sink_list.clear();
}

ros2_spdlog_logger::LoggerPtr get_logger(const std::string & logger_name)
{
  static std::mutex get_logger_mux;
  std::scoped_lock lock(get_logger_mux);

  // fetch existing logger entry
  auto logger = spdlog::get(logger_name);

  // if non found, create a logger
  if (logger == nullptr) {
    logger = create_logger(logger_name);
  }

  return logger;
}

}  // namespace ros2_spdlog_logger

#include <cassert>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <cinttypes>
#include <cstdint>

#include <iterator>
#include <mutex>
#include <optional>
#include <rclcpp/utilities.hpp>
#include <rcutils/allocator.h>
#include <rcutils/logging.h>
#include <rcutils/macros.h>
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
#include <spdlog/spdlog.h>

#include "ros2_spdlog_logger/ros2_spdlog_logger.hpp"
#include "ros2_spdlog_logger/exceptions.hpp"
#include "ros2_spdlog_logger/visibility_control.h"


// use slightly modified ros pattern ([{severity}] [{time}] [{name}]: {message}): "[%l] [%Y-%m-%d %H:%M:%S.%e] [%n] [%s:%#]: %v"
// spdlog's is [%Y-%m-%d %H:%M:%S.%e] [%n] [%l] [%s:%#] %v

#define SPDLOG_DEFAULT_PATTERN "%+"
#define ROS_DEFAULT_PATTERN "[%^%8!l%$] [%E.%F] %@ [%n]: %v"

namespace {
// helper function to get environment variable as a optional
ROS2_SPDLOG_LOGGER_LOCAL std::optional<std::string> getenv_opt(const std::string_view env)
{
  if (const auto* env_data = std::getenv(env.data()); env_data != nullptr){
    return std::string(env_data);
  }
  return std::nullopt;
}

ROS2_SPDLOG_LOGGER_LOCAL std::string getenv_opt(const std::string_view env, const std::string& default_value)
{
  return getenv_opt(env).value_or(default_value);
}

ROS2_SPDLOG_LOGGER_LOCAL bool getenv_value(const std::string_view env, const bool default_value)
{
  const auto env_value = getenv_opt(env);
  if (!env_value.has_value()){
    return default_value;
  }

  std::string lowered_env = env_value.value();

  std::transform(lowered_env.begin(), lowered_env.end(), lowered_env.begin(),
    [](unsigned char c){ return std::tolower(c); });

  if (env_value->compare("0") == 0 || env_value->compare("false") == 0){
    return false;
  }
  else if (env_value->compare("1") == 0 || env_value->compare("true") == 0){
    return true;
  }

  // unable to parse it, just return default value
  return default_value;
}

ROS2_SPDLOG_LOGGER_LOCAL std::string get_pattern(){
  static const auto default_pattern = getenv_opt("ROS2_SPDLOG_PATTERN", SPDLOG_DEFAULT_PATTERN);
  static const auto use_ros_pattern = getenv_value("USE_ROS_PATTERN", false);

  if (use_ros_pattern){
    return ROS_DEFAULT_PATTERN;
  }

  return default_pattern;
}

// convert ros severity levels to spdlog
ROS2_SPDLOG_LOGGER_LOCAL spdlog::level::level_enum to_spdlog_severity(
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
}


extern "C" {
ROS2_SPDLOG_LOGGER_PUBLIC void logging_sink_output_handler(
  const rcutils_log_location_t * location,
  int severity,
  const char * name,
  rcutils_time_point_value_t timestamp,
  const char * format,
  va_list * args)
{

  // build message using the funcs that would be used by rcl
  const auto default_allocator = rcutils_get_default_allocator();

  // local buffer for processing messages (same size as defined in rcutils)
  thread_local std::array<char, 2048> msg_buf;

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
      location->file_name, static_cast<int>(location->line_number),
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
} // extern "C"

namespace
{
// list of all sinks we want to use for our loggers
// the reason for storing it like this is so we can keep track of sinks
// to create new loggers on the spot if we want/need to
inline std::vector<spdlog::sink_ptr> global_sink_list = {}; // NOSONAR

// prevent multiple initializations
inline std::atomic<bool> is_logging_system_initialized = false; // NOSONAR

ROS2_SPDLOG_LOGGER_LOCAL ros2_spdlog_logger::LoggerPtr create_logger(
  const std::string & logger_name)
{
  if (RCUTILS_UNLIKELY(!is_logging_system_initialized)){
    throw ros2_spdlog_logger::LoggingNotInitialized();
  }

#ifndef DISABLE_ASYNC_LOGGER
  auto logger = std::make_shared<spdlog::async_logger>(
    logger_name, global_sink_list.cbegin(),
    global_sink_list.cend(), spdlog::thread_pool(), spdlog::async_overflow_policy::block);
#else
  auto logger = std::make_shared<spdlog::logger>(
    logger_name, global_sink_list.cbegin(),
    global_sink_list.cend());
#endif // !DISABLE_ASYNC_LOGGER

  // get the default pattern that we want for logging everything
  static const auto default_pattern = get_pattern();
  logger->set_pattern(default_pattern);

  // maintain same flushing behavior as in rcl_logging_spdlog
  logger->flush_on(spdlog::level::err);

  spdlog::register_logger(logger);

  return logger;
}

// function adapted from the official rcl_logging spdlog sink
// (https://github.com/ros2/rcl_logging/)
ROS2_SPDLOG_LOGGER_LOCAL std::string get_rcl_logging_filename()
{
  const auto allocator = rcutils_get_default_allocator();
  // To be compatible with ROS 1, we construct a default filename of
  // the form ~/.ros/log/<exe>_<pid>_<milliseconds-since-epoch>.log

  char * logdir = nullptr;
  if (rcl_logging_ret_t dir_ret = rcl_logging_get_logging_directory(allocator, &logdir);
    RCL_LOGGING_RET_OK != dir_ret)
  {
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
  if (rcutils_ret_t ret = rcutils_system_time_now(&now); ret != RCUTILS_RET_OK) {
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
  RCPPUTILS_SCOPE_EXIT({
      allocator.deallocate(basec, allocator.state);
    });

  std::array<char, 4096> name_buffer = {0};
  int print_ret = rcutils_snprintf(
    name_buffer.data(), sizeof(name_buffer),
    "%s/%s_%i_%" PRId64 ".log", logdir,
    basec, rcutils_get_pid(), ms_since_epoch);

  if (print_ret < 0) {
    rclcpp::exceptions::throw_from_rcl_error(
      RCL_LOGGING_RET_ERROR,
      "Failed to create log file name string");
  }

  return std::string(name_buffer.data(), print_ret);
}


/**
 * Calls the underlying shutdown routines for the logging libraries and destroys the sinks.
*/
ROS2_SPDLOG_LOGGER_LOCAL void shutdown_logging_system()
{
  static std::mutex shutdown_mux;
  std::scoped_lock lock{shutdown_mux};

  if (!is_logging_system_initialized) {
    throw ros2_spdlog_logger::LoggingNotInitialized();
  }

  // stop rerouting messages from rcutils
  rcutils_logging_set_output_handler(nullptr);

  spdlog::shutdown();
  global_sink_list.clear();
  is_logging_system_initialized = false;
}

/**
* Initializes our logging system.
*/
ROS2_SPDLOG_LOGGER_LOCAL void init_logging_system(bool hook_on_shutdown){
  // prevent concurrent calls to this function
  static std::mutex init_mux;
  std::scoped_lock lock{init_mux};

  if (is_logging_system_initialized) {
    throw ros2_spdlog_logger::LoggingAlreadyInitialized();
  }

  // initialize spdlog default thread pool if async (1 threads with 8k size queue to ensure ordering)
  #ifndef DISABLE_ASYNC_LOGGER
  spdlog::init_thread_pool(8192, 1);
  #endif// !DISABLE_ASYNC_LOGGER

  // get the filename for logging and append our spdlog sinks to whatever sinks were added before
  // TODO: Make this customizable somehow (ideally using the already parsed rcl arguments)
  ros2_spdlog_logger::add_sinks(std::make_shared<spdlog::sinks::stdout_color_sink_mt>(),
    std::make_shared<spdlog::sinks::basic_file_sink_mt>(get_rcl_logging_filename(), false));

  // maintain same flushing behavior
  spdlog::flush_every(std::chrono::seconds(5));

  if (hook_on_shutdown){
    // setup a post-shutdown callback to destroy all logging things beforehand
    rclcpp::on_shutdown(
      []() {
        shutdown_logging_system();
    }
    );
  }

  // set the output handler for logging using our custom sink
  rcutils_logging_set_output_handler(logging_sink_output_handler);

  // mark the system as initialized
  is_logging_system_initialized = true;

  // setup default logger (need to do it after is_logging_system_initialized=true)
  auto logger = create_logger(DEFAULT_LOGGER);
  spdlog::set_default_logger(logger);
}
}


namespace ros2_spdlog_logger
{
ROS2_SPDLOG_LOGGER_PUBLIC void init(
  int argc,
  char const * const * argv,
  bool hook_on_shutdown,
  const rclcpp::InitOptions & init_options,
  rclcpp::SignalHandlerOptions signal_handler_options)
{
  // copy the init options but disable logging
  auto changed_init_options = init_options;
  changed_init_options.auto_initialize_logging(false);

  // init all rclcpp things without logging
  rclcpp::init(
    argc, argv, changed_init_options, signal_handler_options
  );

  init_logging_system(hook_on_shutdown);
}

ROS2_SPDLOG_LOGGER_PUBLIC std::vector<std::string> init_and_remove_ros_arguments(
  int argc,
  char const * const * argv,
  bool hook_on_shutdown,
  const rclcpp::InitOptions & init_options)
{
  // copy the init options but disable logging
  auto changed_init_options = init_options;
  changed_init_options.auto_initialize_logging(false);

  // init all rclcpp things without logging
  const auto non_ros_args = rclcpp::init_and_remove_ros_arguments(
    argc, argv, changed_init_options
  );

  init_logging_system(hook_on_shutdown);

  return non_ros_args;
}

ROS2_SPDLOG_LOGGER_PUBLIC void shutdown(){
  shutdown_logging_system();
}

ROS2_SPDLOG_LOGGER_PUBLIC ros2_spdlog_logger::LoggerPtr get_logger(const std::string & logger_name)
{
  static std::mutex get_logger_mux;
  std::scoped_lock lock{get_logger_mux};

  // fetch existing logger entry
  auto logger = spdlog::get(logger_name);

  // if non found, create a logger
  if (RCUTILS_UNLIKELY(logger == nullptr)) {
    logger = create_logger(logger_name);
  }

  return logger;
}

ROS2_SPDLOG_LOGGER_PUBLIC void add_sink(spdlog::sink_ptr sink_to_add){
  if (RCUTILS_UNLIKELY(is_logging_system_initialized)){
    throw LoggingAlreadyInitialized();
  }

  global_sink_list.push_back(sink_to_add);
}
}  // namespace ros2_spdlog_logger

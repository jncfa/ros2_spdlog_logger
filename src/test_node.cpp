#include <cstdio>
#include "ros2_spdlog_logger/ros2_spdlog_logger.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // the initialization now uses our custom init function
  ros2_spdlog_logger::init_with_global_context(argc, argv);

  // the scope exit ensures we always shutdown the global rclcpp::Context, exiting cleanly
  RCPPUTILS_SCOPE_EXIT({rclcpp::shutdown();});

  // the loggers are simply typedefs to spdlog::Logger, so if you know how to use those it should be easy!
  auto test = ros2_spdlog_logger::get_logger("my_custom_logger");
  test->info("no way! i can just use this spdlog now!");
  test->info("can use fmt {} now! yay", "formatting");

  // though you are encouraged to use the macros instead, since this will provide additional debug info
  LOG_INFO(test, "can use fmt {} now! yay", "formatting");

  // in case of legacy code / compat with standard ros2 packages, this will reroute the calls to spdlog safely while formatting
  RCLCPP_INFO(rclcpp::get_logger("rclcpp logger"), "But I can still use the old rclcpp %s %s, nice!", "formatting", "too");

  return EXIT_SUCCESS;
}

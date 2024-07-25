#include <cstdio>
#include <fmt/core.h>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rcpputils/scope_exit.hpp>
#include <spdlog/spdlog.h>
#include <string>
#include "ros2_spdlog_logger/ros2_spdlog_logger.hpp"

#include "spdlog/sinks/callback_sink.h"


// helper function to determine if an environment variable is defined
bool is_env_defined(const std::string_view env)
{
  return std::getenv(env.data()) != nullptr;
}

void logger_usage()
{
  auto logger = ros2_spdlog_logger::get_logger("my_custom_logger");

  // the loggers are simply typedefs to spdlog::Logger, so if you know how to use those it should be easy!
  logger->info("no way! i can just use this spdlog now!");
  logger->info("can use fmt {} now! yay", "formatting");

  // though you are encouraged to use the macros instead, since this will provide additional debug info
  LOG_INFO(logger, "can use fmt {} now! whoaaa", "formatting");

  // in case of legacy code / compat with standard ros2 packages, this will reroute the calls to spdlog safely while formatting
  RCLCPP_INFO(rclcpp::get_logger("rclcpp logger"),
    "But I can still use the old rclcpp %s %s, nice!", "formatting", "too");
}

int main(int argc, char ** argv)
{
  // add a dummy sink that just keeps track of messages that have been logged, and prints them at the end of the program
  std::vector<std::string> logged_messages;
  if (!is_env_defined("TEST_CONCURRENCY")) {
    ros2_spdlog_logger::add_sink(
      std::make_shared<spdlog::sinks::callback_sink_mt>([&logged_messages](const spdlog::details::log_msg &msg){
        logged_messages.emplace_back(msg.payload.data(), msg.payload.size());
      })
    );
  }

  RCPPUTILS_SCOPE_EXIT({
    if (!is_env_defined("TEST_CONCURRENCY")) {
      fmt::print("found {} messages in the queue\n", logged_messages.size());
      for(const auto& c: logged_messages){
        fmt::print("{}\n", c);
      }
     }
  });

  // the initialization now uses our custom init function
  ros2_spdlog_logger::init(argc, argv);

  // the scope exit ensures we always shutdown the global rclcpp::Context, exiting cleanly
  RCPPUTILS_SCOPE_EXIT({rclcpp::shutdown();});

  // check if we call this once or if we want to test for concurrency
  if (!is_env_defined("TEST_CONCURRENCY")) {
    logger_usage();
  } else {
    std::vector<std::thread> vectest;
    for (auto x = 0; x < 10; x++) {
      vectest.emplace_back([](){
        // use the logger a bunch of times
          for(auto y = 0; y < 100000; y++) {
            logger_usage();
          }
      });
    }

    for (auto & thread : vectest) {
      if(thread.joinable()) {
        thread.join();
      }
    }
  }
  spdlog::error("print this now!");
  return EXIT_SUCCESS;
}

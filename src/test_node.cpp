#include <cstdio>
#include "ros2_spdlog_logger/ros2_spdlog_logger.hpp"
#include "rclcpp/rclcpp.hpp"

// helper function to determine if an environment variable is defined
bool is_env_defined(const std::string_view env){
  return std::getenv(env.data()) != nullptr;
}

void logger_usage(){
  auto logger = ros2_spdlog_logger::get_logger("my_custom_logger");

  // the loggers are simply typedefs to spdlog::Logger, so if you know how to use those it should be easy!
  logger->info("no way! i can just use this spdlog now!");
  logger->info("can use fmt {} now! yay", "formatting");

  // though you are encouraged to use the macros instead, since this will provide additional debug info
  LOG_INFO(logger, "can use fmt {} now! yay", "formatting");

  // in case of legacy code / compat with standard ros2 packages, this will reroute the calls to spdlog safely while formatting
  RCLCPP_INFO(rclcpp::get_logger("rclcpp logger"), "But I can still use the old rclcpp %s %s, nice!", "formatting", "too");
}

int main(int argc, char ** argv)
{

  // the initialization now uses our custom init function
  ros2_spdlog_logger::init_with_global_context(argc, argv);

  // the scope exit ensures we always shutdown the global rclcpp::Context, exiting cleanly
  RCPPUTILS_SCOPE_EXIT({rclcpp::shutdown();});

  // check if we call this once or if we want to test for concurrency
  if (!is_env_defined("TEST_CONCURRENCY")){
    logger_usage();
  }
  else{
    std::vector<std::thread> vectest;
    for (auto i = 0; i<10; i++){
      vectest.emplace_back([](){
        // use the logger a bunch of times
        for(auto i = 0; i < 100000; i++){
          logger_usage();
        }
      });
    }

    for (auto &thread : vectest){
      if(thread.joinable()){
        thread.join();
      }
    }
  }

  return EXIT_SUCCESS;
}

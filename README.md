# ros2_spdlog_logger

> What if we could just use spdlog instead?

This is a package that allows you to override the logging system in rcl/rclcpp to use spdlog!
I've done some concurrency tests with tsan&valgrind to ensure that this package is stable for now, but do beware that this isn't intended to be customizable by the rcl/rclcpp API

You can also run this quick test if you build
```bash
TEST_CONCURRENCY=1 ros2 run --prefix "valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --verbose --log-file=valgrind-out.txt" ros2_spdlog_logger test_node
```

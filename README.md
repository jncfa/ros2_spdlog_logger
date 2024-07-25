# ros2_spdlog_logger

> What if we could just use spdlog instead?

This is a package that allows you to override the logging system in rcl/rclcpp to use spdlog!

The tl;dr for how this works is, we replace [rclcpp's logging system initialization](https://github.com/ros2/rclcpp/blob/a4d7210b9cef081707ecec8aea7a7988ffd87b7f/rclcpp/src/rclcpp/context.cpp#L215-L234) with [our own](https://github.com/jncfa/ros2_spdlog_logger/blob/e8c62d89d88b3d5e1a182f860b129c70c5bb7633/src/ros2_spdlog_logger.cpp#L331-L339)!

We use our own [logging output handler](https://github.com/jncfa/ros2_spdlog_logger/blob/e8c62d89d88b3d5e1a182f860b129c70c5bb7633/src/ros2_spdlog_logger.cpp#L73-L136), that will capture all logging calls that rely on rclcpp macros, which before was rerouted to [rclcpp's logging output handler](https://github.com/ros2/rclcpp/blob/a4d7210b9cef081707ecec8aea7a7988ffd87b7f/rclcpp/src/rclcpp/context.cpp#L125-L143), and instead we run a very similar code to what the underlying [rcutils logging call](https://github.com/ros2/rcutils/blob/cf3e9986a557e4c893eb718b90c77ce189494a11/src/logging.c#L1373-L1465) would do.

I've done quite a bit of testing with this package and found no issues, but please understand that this is relying on some rcl/rclcpp internal API that may change in the future.
That being said, this should work for any ROS2 distribution from at least Foxy up to Jazzy.

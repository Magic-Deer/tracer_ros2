/*
 * tracer_base_node.cpp
 *
 * Created on: 3 2, 2021 16:39
 * Description:
 *
 * Copyright (c) 2022 agilex Robot Pte. Ltd.
 */

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tracer_base/tracer_base_ros.hpp"

using namespace westonrobot;

int main(int argc, char **argv) {
  // setup ROS node
  rclcpp::init(argc, argv);

  auto robot = std::make_shared<TracerBaseRos>("tracer");

  std::cout << "Robot initialized, start running ..." << std::endl;
  robot->Run();
  robot.reset();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}

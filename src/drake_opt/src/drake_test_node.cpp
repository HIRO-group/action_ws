
// ROS
#include <ros/ros.h>

// Drake
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>

// c++
#include <iostream>

using namespace drake;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_plan");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  std::cout << "hello world" << std::endl;

  return 0;
}
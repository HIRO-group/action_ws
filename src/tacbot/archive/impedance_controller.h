#ifndef CONTACT_IMPEDANCE_CONTROLLER_H
#define CONTACT_IMPEDANCE_CONTROLLER_H

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <Eigen/Core>
#include <array>
#include <atomic>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

struct ControllerPrintData {
  std::mutex mutex;
  bool has_data;
  std::array<double, 7> tau_d_last;
  franka::RobotState robot_state;
  std::array<double, 7> gravity;
};

template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1,
            std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}

class ImpedanceController {
 public:
  void init();
  void executeTrajectory(
      const robot_trajectory::RobotTrajectoryPtr& trajectory);

 private:
  ControllerPrintData print_data_;
  const double print_rate_ms_ = 10.0;
  std::atomic_bool execution_status_{true};

  std::shared_ptr<franka::Robot> robot_;
  std::shared_ptr<franka::Model> robot_model_;
  std::string franka_address_;

  robot_trajectory::RobotTrajectoryPtr trajectory_;
  std::size_t trajectory_idx_ = 0;

  // Stiffness
  const std::array<double, 7> k_gains_ = {
      {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
  // Damping
  const std::array<double, 7> d_gains_ = {
      {50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};

  void setDefaultBehavior();

  void printControlData();

  franka::Torques impedanceControlCallback(const franka::RobotState& state,
                                           franka::Duration duration);
  franka::JointPositions motionGeneratorCallback(
      const franka::RobotState& state, franka::Duration duration);
};

#endif
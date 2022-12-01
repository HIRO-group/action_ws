#ifndef CONTACT_IMPEDANCE_CONTROLLER_H
#define CONTACT_IMPEDANCE_CONTROLLER_H

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <Eigen/Core>
#include <array>
#include <iostream>
#include <memory>

class ImpedanceController {
 public:
  void init();
  void setDefaultBehavior();

 private:
  std::shared_ptr<franka::Robot> robot_;
  franka::Model robot_model_;
  std::string franka_address_;

  // Stiffness
  const std::array<double, 7> k_gains_ = {
      {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
  // Damping
  const std::array<double, 7> d_gains_ = {
      {50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
};

#endif
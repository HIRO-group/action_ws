#include "impedance_controller.h"
void ImpedanceController::init() {
  std::cout << "Initializing Controller!" << std::endl;
  robot_ = std::make_shared<franka::Robot>(franka_address_);
  setDefaultBehavior();

  robot_model_ = robot->loadModel();
}

void ImpedanceController::setDefaultBehavior() {
  // robot_->setCollisionBehavior({{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
  //                              {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
  //                              {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
  //                              {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
  //                              {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
  //                              {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
  //                              {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
  //                              {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

  robot_->setCollisionBehavior({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                               {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

  robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

franka::Torques ImpedanceControl::impedanceControlCallback(
    const RobotState& state, franka::Duration duration) {
  // Read current coriolis terms from model.
  std::array<double, 7> coriolis = robot_model_.coriolis(state);

  // Compute torque command from joint impedance control law.
  // Note: The answer to our Cartesian pose inverse kinematics is always in
  // state.q_d with one time step delay.
  std::array<double, 7> tau_d_calculated;
  for (size_t i = 0; i < 7; i++) {
    tau_d_calculated[i] = k_gains[i] * (state.q_d[i] - state.q[i]) -
                          d_gains[i] * state.dq[i] + coriolis[i];
  }

  // The following line is only necessary for printing the rate limited torque.
  // As we activated rate limiting for the control loop (activated by default),
  // the torque would anyway be adjusted!
  std::array<double, 7> tau_d_rate_limited = franka::limitRate(
      franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);
}
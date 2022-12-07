#include "impedance_controller.h"

constexpr char LOGNAME[] = "impedance_controller";

void ImpedanceController::init() {
  std::cout << "Initializing Controller!" << std::endl;
  robot_ = std::make_shared<franka::Robot>(franka_address_);
  setDefaultBehavior();
  robot_model_ = std::make_shared<franka::Model>(robot_->loadModel());
}

void ImpedanceController::setDefaultBehavior() {
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

franka::Torques ImpedanceController::impedanceControlCallback(
    const franka::RobotState& state, franka::Duration duration) {
  std::array<double, 7> coriolis = robot_model_->coriolis(state);
  std::array<double, 7> tau_d_calculated;
  for (size_t i = 0; i < 7; i++) {
    tau_d_calculated[i] = k_gains_[i] * (state.q_d[i] - state.q[i]) -
                          d_gains_[i] * state.dq[i] + coriolis[i];
  }
  std::array<double, 7> tau_d_rate_limited = franka::limitRate(
      franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);

  if (print_data_.mutex.try_lock()) {
    print_data_.has_data = true;
    print_data_.robot_state = state;
    print_data_.tau_d_last = tau_d_rate_limited;
    print_data_.gravity = robot_model_->gravity(state);
    print_data_.mutex.unlock();
  }

  return tau_d_rate_limited;
}

franka::JointPositions ImpedanceController::motionGeneratorCallback(
    const franka::RobotState& state, franka::Duration duration) {
  if (trajectory_ && !trajectory_->empty()) {
    ROS_ERROR_NAMED(LOGNAME, "Trajectory is empty. Unable to execute.");
  }

  ROS_INFO_NAMED(LOGNAME, "trajectory_idx_: %ld", trajectory_idx_);
  ROS_INFO_NAMED(LOGNAME, "duration: %f", duration.toSec());

  moveit::core::RobotState robot_state =
      trajectory_->getWayPoint(trajectory_idx_);
  double t_from_start =
      trajectory_->getWayPointDurationFromStart(trajectory_idx_);
  double t_from_previous =
      trajectory_->getWayPointDurationFromPrevious(trajectory_idx_);
  ROS_INFO_NAMED(LOGNAME, "t_from_start: %f", t_from_start);
  ROS_INFO_NAMED(LOGNAME, "t_from_previous: %f", t_from_previous);

  const std::vector<std::string> names = robot_state.getVariableNames();
  const double* pos = robot_state.getVariablePositions();
  robot_state.printStatePositions();

  std::array<double, 7> position_arr;
  for (std::size_t i = 0; i < names.size(); ++i) {
    position_arr[i] = pos[i];
  }

  franka::JointPositions joint_positions(position_arr);
  trajectory_idx_++;

  if (trajectory_idx_ >= trajectory_->getWayPointCount()) {
    ROS_INFO_NAMED(LOGNAME, "Motion finished");
    return franka::MotionFinished(joint_positions);
  }

  return joint_positions;
}

void ImpedanceController::printControlData() {
  while (execution_status_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>((1.0 / print_rate_ms_ * 1000.0))));
    if (print_data_.mutex.try_lock()) {
      if (print_data_.has_data) {
        double error_rms(0.0);
        std::array<double, 7> tau_error{};
        std::array<double, 7> tau_d_actual{};
        for (size_t i = 0; i < 7; ++i) {
          tau_d_actual[i] = print_data_.tau_d_last[i] + print_data_.gravity[i];
          tau_error[i] = tau_d_actual[i] - print_data_.robot_state.tau_J[i];
          error_rms += std::pow(tau_error[i], 2.0) / tau_error.size();
        }

        error_rms = std::sqrt(error_rms);
        std::cout << "tau_error [Nm]: " << tau_error << std::endl
                  << "tau_commanded [Nm]: " << tau_d_actual << std::endl
                  << "tau_measured [Nm]: " << print_data_.robot_state.tau_J
                  << std::endl
                  << "root mean square of tau_error [Nm]: " << error_rms
                  << std::endl
                  << "-----------------------" << std::endl;
        print_data_.has_data = false;
      }

      print_data_.mutex.unlock();
    }
  }
}

void ImpedanceController::executeTrajectory(
    const robot_trajectory::RobotTrajectoryPtr& trajectory) {
  ROS_ERROR_NAMED(LOGNAME, "Executing trajectory.");

  if (trajectory_ && !trajectory_->empty()) {
    ROS_ERROR_NAMED(LOGNAME, "Trajectory is empty. Unable to set.");
  }
  ROS_INFO_NAMED(LOGNAME, "Num waypoints: %ld", trajectory->getWayPointCount());
  ROS_INFO_NAMED(LOGNAME, "Duration: %f", trajectory->getDuration());
  trajectory_ = trajectory;

  std::thread print_thread(&ImpedanceController::printControlData, this);

  std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
      control_callback =
          std::bind(&ImpedanceController::impedanceControlCallback, this,
                    std::placeholders::_1, std::placeholders::_2);

  std::function<franka::JointPositions(const franka::RobotState&,
                                       franka::Duration)>
      motion_generator_callback =
          std::bind(&ImpedanceController::motionGeneratorCallback, this,
                    std::placeholders::_1, std::placeholders::_2);

  robot_->control(control_callback, motion_generator_callback);
}
#include "joint_position_controller.h"

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <cmath>

namespace pick_and_place {

bool JointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                   ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointPositionController: Could not read parameter arm_id");
    return false;
  }

  auto* model_interface =
      robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointPositionController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointPositionController: Exception getting model handle from "
        "interface: "
        << ex.what());
    return false;
  }

  auto* state_interface =
      robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointPositionController: Error getting state interface "
        "from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointPositionController: Exception getting state handle "
        "from interface: "
        << ex.what());
    return false;
  }

  position_joint_interface_ =
      robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionController: Error getting position joint "
        "interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM(
        "JointPositionController: Wrong number of joint names, got "
        << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] =
          position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionController: Exception getting joint handles: "
          << e.what());
      return false;
    }
  }

  // std::array<double, 7> q_start{
  //     {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  // for (size_t i = 0; i < q_start.size(); i++) {
  //   if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) >
  //   0.1) {
  //     ROS_ERROR_STREAM(
  //         "JointPositionController: Robot is not in the expected "
  //         "starting position for "
  //         "running this example. Run `roslaunch franka_example_controllers "
  //         "move_to_start.launch "
  //         "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>`
  //         first.");
  //     return false;
  //   }
  // }
  double publish_rate(5.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("ContactController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  trajectory_monitor_pub_.init(node_handle, "trajectory_monitor", 1);

  trajectory_sub_ =
      node_handle.subscribe("contact_trajectory", 1,
                            &JointPositionController::trajectoryCallback, this);

  // ros::spinOnce();

  return true;
}

void JointPositionController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }
  elapsed_time_ = ros::Duration(0.0);
}

bool JointPositionController::isApproxEqual(const std::array<double, 7>& a,
                                            const std::array<double, 7>& b,
                                            double tolerance) {
  for (size_t i = 0; i < 7; i++) {
    double dif = a[i] - b[i];
    if (std::abs(dif) > tolerance) {
      return false;
    }
  }
  return true;
}

int JointPositionController::isRobotStateValid(
    const franka::RobotState& robot_state) {
  const double tau_thresh = 50.0;
  int status = 1;

  for (std::size_t i = 0; i < robot_state.q.size(); i++) {
    double tau_J = robot_state.tau_J[i];
    if (std::abs(tau_J) > tau_thresh) {
      status = -1;
      break;
    }
  }
  return status;
}

void JointPositionController::update(const ros::Time& /*time*/,
                                     const ros::Duration& period) {
  elapsed_time_ += period;

  std::size_t num_pts = joint_trajectory_.points.size();
  if (num_pts == 0) {
    elapsed_time_ = ros::Duration(0.0);
    return;
  } else {
    ROS_INFO_STREAM("Executing trajectory with pts: " << num_pts);
    is_executing = true;
  }

  franka::RobotState robot_state = state_handle_->getRobotState();
  int is_valid = isRobotStateValid(robot_state);

  if (trajectory_monitor_pub_.trylock()) {
    franka_msgs::FrankaState franka_state_msg;
    robotStateToMsg(robot_state, franka_state_msg);
    trajectory_monitor_pub_.msg_.franka_state = franka_state_msg;

    trajectory_monitor_pub_.msg_.header.stamp = ros::Time::now();
    ROS_INFO_STREAM("pt_idx_: " << pt_idx_);
    trajectory_monitor_pub_.msg_.pt_idx = pt_idx_;
    trajectory_monitor_pub_.msg_.is_valid = is_valid;
    trajectory_monitor_pub_.unlockAndPublish();
  }

  if (is_valid == -1) {
    ROS_ERROR_STREAM("Robot state is invalid.");
    // std::cout << "Robot State" << std::endl;
    // std::cout << robot_state << std::endl;
    reset();
    return;
  }

  std::array<double, 7> desired_pose{};
  std::array<double, 7> current_pose{};

  std::size_t i = pt_idx_;
  for (; i < num_pts; i++) {
    trajectory_msgs::JointTrajectoryPoint point = joint_trajectory_.points[i];
    ros::Duration tm_from_start = point.time_from_start;

    ROS_INFO_STREAM("i: " << i);
    // ROS_INFO_STREAM("tm_from_start: " << tm_from_start.toSec());
    // ROS_INFO_STREAM("elapsed_time_: " << elapsed_time_.toSec());
    // ROS_INFO_STREAM("period: " << period.toSec());

    for (std::size_t j = 0; j < 7; j++) {
      current_pose[j] = position_joint_handles_[j].getPosition();
      desired_pose[j] = point.positions[j];
    }

    if (isApproxEqual(desired_pose, current_pose)) {
      pt_idx_++;
      continue;
    } else {
      break;
    }

    // if (elapsed_time_ <= tm_from_start) {
    //   pt_idx = i;
    //   for (std::size_t j = 0; j < 7; j++) {
    //     double position = point.positions[pt_idx];
    //     desired_pose[j] = position;
    //   }
    //   break;
    // }
  }

  if (i >= num_pts) {
    ROS_INFO_STREAM("Trajectory completed.");
    reset();
    return;
  }

  std::array<double, 7> delta_angle{};
  for (size_t i = 0; i < 7; ++i) {
    delta_angle[i] = desired_pose[i] - current_pose[i];
  }

  double max_delta = 0.001;

  for (size_t i = 0; i < 7; ++i) {
    double delta = delta_angle[i];
    ROS_INFO_STREAM("desired_pose[i]: " << desired_pose[i]);
    ROS_INFO_STREAM("current_pose[i]: " << current_pose[i]);
    ROS_INFO_STREAM("delta_angle[i]: " << delta_angle[i]);
    if (std::abs(delta) > max_delta) {
      double sign = (delta < 0) ? -1.0 : 1.0;
      delta = max_delta * sign;
      ROS_INFO_STREAM("delta: " << delta);
    }
    position_joint_handles_[i].setCommand(current_pose[i] + delta);
  }
}

void JointPositionController::reset() {
  joint_trajectory_.points.clear();
  elapsed_time_ = ros::Duration(0.0);
  pt_idx_ = 0;
  is_executing = false;
}

void JointPositionController::trajectoryCallback(
    const trajectory_msgs::JointTrajectoryPtr& msg) {
  if (is_executing == true) {
    return;
  }
  ROS_INFO_STREAM("JointPositionController::trajectoryCallback.");
  joint_trajectory_ = *msg;
}

void JointPositionController::robotStateToMsg(
    const franka::RobotState& robot_state,
    franka_msgs::FrankaState& franka_state_msg) {
  for (size_t i = 0; i < robot_state.cartesian_collision.size(); i++) {
    franka_state_msg.cartesian_collision[i] =
        robot_state.cartesian_collision[i];
    franka_state_msg.cartesian_contact[i] = robot_state.cartesian_contact[i];
    franka_state_msg.K_F_ext_hat_K[i] = robot_state.K_F_ext_hat_K[i];
    franka_state_msg.O_F_ext_hat_K[i] = robot_state.O_F_ext_hat_K[i];
    franka_state_msg.O_dP_EE_d[i] = robot_state.O_dP_EE_d[i];
    franka_state_msg.O_dP_EE_c[i] = robot_state.O_dP_EE_c[i];
    franka_state_msg.O_ddP_EE_c[i] = robot_state.O_ddP_EE_c[i];
  }

  for (size_t i = 0; i < robot_state.q.size(); i++) {
    franka_state_msg.q[i] = robot_state.q[i];
    franka_state_msg.q_d[i] = robot_state.q_d[i];
    franka_state_msg.dq[i] = robot_state.dq[i];
    franka_state_msg.dq_d[i] = robot_state.dq_d[i];
    franka_state_msg.ddq_d[i] = robot_state.ddq_d[i];
    franka_state_msg.tau_J[i] = robot_state.tau_J[i];
    franka_state_msg.dtau_J[i] = robot_state.dtau_J[i];
    franka_state_msg.tau_J_d[i] = robot_state.tau_J_d[i];
    franka_state_msg.theta[i] = robot_state.theta[i];
    franka_state_msg.dtheta[i] = robot_state.dtheta[i];
    franka_state_msg.joint_collision[i] = robot_state.joint_collision[i];
    franka_state_msg.joint_contact[i] = robot_state.joint_contact[i];
    franka_state_msg.tau_ext_hat_filtered[i] =
        robot_state.tau_ext_hat_filtered[i];
  }

  for (size_t i = 0; i < robot_state.elbow.size(); i++) {
    franka_state_msg.elbow[i] = robot_state.elbow[i];
    franka_state_msg.elbow_d[i] = robot_state.elbow_d[i];
    franka_state_msg.elbow_c[i] = robot_state.elbow_c[i];
    franka_state_msg.delbow_c[i] = robot_state.delbow_c[i];
    franka_state_msg.ddelbow_c[i] = robot_state.ddelbow_c[i];
  }

  for (size_t i = 0; i < robot_state.O_T_EE.size(); i++) {
    franka_state_msg.O_T_EE[i] = robot_state.O_T_EE[i];
    franka_state_msg.F_T_EE[i] = robot_state.F_T_EE[i];
    franka_state_msg.F_T_NE[i] = robot_state.F_T_NE[i];
    franka_state_msg.NE_T_EE[i] = robot_state.NE_T_EE[i];
    franka_state_msg.EE_T_K[i] = robot_state.EE_T_K[i];
    franka_state_msg.O_T_EE_d[i] = robot_state.O_T_EE_d[i];
    franka_state_msg.O_T_EE_c[i] = robot_state.O_T_EE_c[i];
  }
  franka_state_msg.m_ee = robot_state.m_ee;
  franka_state_msg.m_load = robot_state.m_load;
  franka_state_msg.m_total = robot_state.m_total;

  for (size_t i = 0; i < robot_state.I_load.size(); i++) {
    franka_state_msg.I_ee[i] = robot_state.I_ee[i];
    franka_state_msg.I_load[i] = robot_state.I_load[i];
    franka_state_msg.I_total[i] = robot_state.I_total[i];
  }

  for (size_t i = 0; i < robot_state.F_x_Cload.size(); i++) {
    franka_state_msg.F_x_Cee[i] = robot_state.F_x_Cee[i];
    franka_state_msg.F_x_Cload[i] = robot_state.F_x_Cload[i];
    franka_state_msg.F_x_Ctotal[i] = robot_state.F_x_Ctotal[i];
  }

  franka_state_msg.O_ddP_O[2] = -9.81;

  franka_state_msg.time = robot_state.time.toSec();
  franka_state_msg.control_command_success_rate =
      robot_state.control_command_success_rate;
  // franka_state_msg.current_errors =
  // errorsToMessage(robot_state.current_errors);
  // franka_state_msg.last_motion_errors =
  //     errorsToMessage(robot_state.last_motion_errors);

  switch (robot_state.robot_mode) {
    case franka::RobotMode::kOther:
      franka_state_msg.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_OTHER;
      break;

    case franka::RobotMode::kIdle:
      franka_state_msg.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_IDLE;
      break;

    case franka::RobotMode::kMove:
      franka_state_msg.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_MOVE;
      break;

    case franka::RobotMode::kGuiding:
      franka_state_msg.robot_mode =
          franka_msgs::FrankaState::ROBOT_MODE_GUIDING;
      break;

    case franka::RobotMode::kReflex:
      franka_state_msg.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_REFLEX;
      break;

    case franka::RobotMode::kUserStopped:
      franka_state_msg.robot_mode =
          franka_msgs::FrankaState::ROBOT_MODE_USER_STOPPED;
      break;

    case franka::RobotMode::kAutomaticErrorRecovery:
      franka_state_msg.robot_mode =
          franka_msgs::FrankaState::ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY;
      break;
  }
}

}  // namespace pick_and_place

PLUGINLIB_EXPORT_CLASS(pick_and_place::JointPositionController,
                       controller_interface::ControllerBase)

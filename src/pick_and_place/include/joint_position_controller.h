#ifndef JOINT_POSITION_CONTROLLER_H
#define JOINT_POSITION_CONTROLLER_H

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
#include <franka_msgs/FrankaState.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <array>
#include <string>
#include <vector>

#include "pick_and_place/TrajExecutionMonitor.h"

namespace pick_and_place {

class JointPositionController
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface, franka_hw::FrankaStateInterface,
          hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware,
            ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;

  ros::Duration elapsed_time_;
  std::array<double, 7> initial_pose_{};

  ros::Subscriber trajectory_sub_;
  trajectory_msgs::JointTrajectory joint_trajectory_;
  std::size_t pt_idx_ = 0;
  bool is_executing = false;

  franka_hw::TriggerRate rate_trigger_{1.0};
  realtime_tools::RealtimePublisher<TrajExecutionMonitor>
      trajectory_monitor_pub_;

  int isRobotStateValid(const franka::RobotState& robot_state);

  void robotStateToMsg(const franka::RobotState& robot_state,
                       franka_msgs::FrankaState& franka_state_msg);

  void trajectoryCallback(const trajectory_msgs::JointTrajectoryPtr& msg);

  bool isApproxEqual(const std::array<double, 7>& a,
                     const std::array<double, 7>& b, double tolerance = 1e-2);

  void reset();
};

}  // namespace pick_and_place

#endif
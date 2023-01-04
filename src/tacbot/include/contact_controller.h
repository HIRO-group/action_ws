#ifndef CONTACT_CONTROLLER_H
#define CONTACT_CONTROLLER_H

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <tacbot/JointTorqueComparison.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <memory>
#include <string>
#include <vector>

namespace tacbot {
class ContactController
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface, franka_hw::FrankaStateInterface,
          hardware_interface::EffortJointInterface,
          hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated,
      const std::array<double, 7>&
          tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  static constexpr double kDeltaTauMax{1.0};
  double radius_{0.1};
  double acceleration_time_{2.0};
  double vel_max_{0.05};
  double angle_{0.0};
  double vel_current_{0.0};

  std::vector<double> k_gains_;
  std::vector<double> d_gains_;
  double coriolis_factor_{1.0};
  std::array<double, 7> dq_filtered_;
  std::array<double, 16> initial_pose_;

  franka_hw::TriggerRate rate_trigger_{1.0};
  std::array<double, 7> last_tau_d_{};
  realtime_tools::RealtimePublisher<JointTorqueComparison> torques_publisher_;
};

}  // namespace tacbot

#endif
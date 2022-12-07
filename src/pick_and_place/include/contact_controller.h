#ifndef CONTACT_CONTROLLER_H
#define CONTACT_CONTROLLER_H

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <array>
#include <string>
#include <vector>

namespace pick_and_place {
class ContactController : public controller_interface::MultiInterfaceController<
                              hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware,
            ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  ros::Duration elapsed_time_;
  std::array<double, 7> initial_pose_{};
};

}  // namespace pick_and_place

#endif
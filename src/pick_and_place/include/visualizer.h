#ifndef VISUALIZER_H
#define VISUALIZER_H

// ROS
#include <moveit/planning_interface/planning_interface.h>
#include <ros/ros.h>

#include "contact_planner.h"
#include "manipulability_measures.h"
#include "utilities.h"
#include "visualizer_data.h"

namespace pick_and_place {

class ContactPlanner;

class Visualizer {
 public:
  Visualizer();

  void setContactPlanner(
      const std::shared_ptr<ContactPlanner>& contact_planner);

  void visualizeManipVec(std::size_t state_num);
  void visualizeRepulseVec(std::size_t state_num);
  void visualizeRepulseOrigin(std::size_t state_num);
  void saveOriginVec(const Eigen::Vector3d& origin, const Eigen::Vector3d& vec,
                     std::size_t num_pts, std::size_t pt_num);
  void visualizeObstacleMarker(
      const std::vector<Eigen::Vector3d>& obstacle_pos);
  void visualizeRepulsedState();
  void visualizeTreeStates();
  void visualizeGoalState();
  void visualizeTrajectory(const planning_interface::MotionPlanResponse& res,
                           std::string name);

 private:
  ros::NodeHandle nh_;
  std::size_t dof_ = 7;                         // get this from robot model
  const std::string group_name_ = "panda_arm";  // likewise

  ros::Publisher robot_marker_pub_;
  ros::Publisher obstacle_marker_pub_;
  ros::Publisher arrow_pub_;
  ros::Publisher manipulability_pub_;
  ros::Publisher rep_state_publisher_;
  ros::Publisher tree_states_publisher_;
  ros::Publisher goal_state_publisher_;
  std::vector<ros::Publisher> trajectory_publishers_;

  std::size_t viz_state_idx_ = 0;
  std::shared_ptr<ContactPlanner> contact_planner_;
};
}  // namespace pick_and_place

#endif
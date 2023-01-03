#ifndef CONTACT_PLANNER_H
#define CONTACT_PLANNER_H

// ROS
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

// C++
#include <algorithm>
#include <mutex>
#include <vector>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/ompl_interface/detail/constrained_goal_sampler.h>
#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// OMPL
#include <ompl/base/objectives/VFUpstreamCriterionOptimizationObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/CVFRRT.h>
#include <ompl/geometric/planners/rrt/ClassicTRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/PathSimplifier.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/SVD>

// Local libraries, helper functions, and utilities
#include "contact_perception.h"
#include "manipulability_measures.h"
#include "pick_and_place/TrajExecutionMonitor.h"
#include "utilities.h"
#include "visualizer.h"
#include "visualizer_data.h"

namespace pick_and_place {

class ContactPlanner {
 public:
  ContactPlanner();

  void init();
  void setCurToStartState(planning_interface::MotionPlanRequest& req);
  void createPlanningContext(const moveit_msgs::MotionPlanRequest& req);
  void changePlanner();
  bool generatePlan(planning_interface::MotionPlanResponse& res);

  moveit_msgs::Constraints createPoseGoal();
  moveit_msgs::Constraints createJointGoal();

  std::string getGroupName();
  std::vector<Eigen::Vector3d> getSimObstaclePos();
  std::string getDefaultPlannerId();
  ompl_interface::ModelBasedPlanningContextPtr getPlanningContext();

  void executeTrajectory();
  void monitorExecution();

 private:
  ros::NodeHandle nh_;
  const std::string group_name_ = "panda_arm";
  const std::string default_planner_id_ = "panda_arm[RRT]";

  std::shared_ptr<ContactPerception> contact_perception_;
  VisualizerData vis_data_;
  friend class Visualizer;

  const std::size_t dof_ = 7;  // get this from robot model
  std::size_t sample_state_count_ = 0;

  ros::Publisher trajectory_pub_;
  ros::Subscriber execution_monitor_sub_;
  std::mutex monitor_mtx_;
  pick_and_place::TrajExecutionMonitor monitor_msg_;

  const bool use_sim_obstacles_ = true;
  std::vector<Eigen::Vector3d> sim_obstacle_pos_;
  std::vector<double> joint_goal_pos_;

  planning_interface::MotionPlanResponse plan_response_;

  const moveit::core::JointModelGroup* joint_model_group_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  moveit::core::RobotModelPtr robot_model_;
  kinematics_metrics::KinematicsMetricsPtr kinematics_metrics_;
  moveit::core::RobotStatePtr robot_state_;
  ompl_interface::ModelBasedPlanningContextPtr context_;
  ompl::base::OptimizationObjectivePtr optimization_objective_;

  std::unique_ptr<ompl_interface::OMPLInterface> getOMPLInterface(
      const moveit::core::RobotModelConstPtr& model, const ros::NodeHandle& nh);
  void setPlanningContextParams(
      ompl_interface::ModelBasedPlanningContextPtr& context);
  planning_interface::PlannerConfigurationSettings getPlannerConfigSettings(
      const planning_interface::PlannerConfigurationMap& pconfig_map,
      const std::string& planner_id);

  Eigen::VectorXd obstacleField(const ompl::base::State* base_state);
  Eigen::VectorXd goalField(const ompl::base::State* state);
  Eigen::VectorXd negGoalField(const ompl::base::State* state);
  Eigen::VectorXd totalField(const ompl::base::State* state);

  Eigen::Vector3d scaleToDist(Eigen::Vector3d vec);
  void extractPtsFromModel(const moveit::core::RobotStatePtr& robot_state,
                           const moveit::core::LinkModel* link_model,
                           std::vector<Eigen::Vector3d>& link_pts,
                           std::size_t& num_pts);
  std::size_t getPtsOnRobotSurface(
      const moveit::core::RobotStatePtr& robot_state,
      std::vector<std::vector<Eigen::Vector3d>>& rob_pts);
  std::vector<Eigen::Vector3d> getLinkToObsVec(
      const std::vector<std::vector<Eigen::Vector3d>>& rob_pts);
  std::vector<Eigen::Vector3d> getObstacles(const Eigen::Vector3d& pt_on_rob);

  void executionMonitorCallback(
      const pick_and_place::TrajExecutionMonitor& msg);
  void updateObstacles();
};
}  // namespace pick_and_place
#endif
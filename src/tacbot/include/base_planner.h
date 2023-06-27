#ifndef TACBOT_BASE_PLANNER_H
#define TACBOT_BASE_PLANNER_H

// ROS
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

// C++
#include <math.h>

#include <algorithm>
#include <chrono>
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
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

// OMPL

#include <ompl/geometric/SimpleSetup.h>

#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// Local libraries, helper functions, and utilities
#include "utilities.h"
#include "visualizer_data.h"

namespace tacbot {

class BasePlanner {
 public:
  BasePlanner();

  /** \brief Initialize the planning scene, monitoring, publishers and
   * subscribers.*/
  virtual void init();

  /** \brief Set the start state in the request message to the current state of
    the robot.
    @param req The motion planning request.
  */
  void setCurToStartState(planning_interface::MotionPlanRequest& req);
  void setStartState(planning_interface::MotionPlanRequest& req,
                     const std::vector<double>& pos);

  /** \brief Changes the planner from the default one that is native to the
     moveit environment, such as RRT, to one that has been specifically created
     for the contact motion planning library, such as ContactTRRT.
  */
  virtual void changePlanner();

  /** \brief Calls on the context to use the input parameters and the planner
     setup to generate a trajectory for the robot to follow. Any changes to the
     planning request or context need to be done before this call.
      @param res The motion planning response where the trajectory and results
     will be stored.
      @return bool Whether or not the plan and time parameterization have been
     successfully accomplished.
  */
  virtual bool generatePlan(planning_interface::MotionPlanResponse& res);

  /** \brief Create a sample joint goal state, in joint space, for the robot to
    reach.
      @return moveit_msgs::Constraints The goal state and restrictions as
     specified by the moveit message.
  */
  moveit_msgs::Constraints createJointGoal();

  /** \brief Getter function for the group name of the robot. Usually based on
     the robot_description loaded into the parameter server as the urdf.
      @return string for the group name of the robot.
  */
  std::string getGroupName();

  /** \brief Getter for the moveit plannning context.
    @return Planning context.
  */
  ompl_interface::ModelBasedPlanningContextPtr getPlanningContext() {
    return context_;
  };

  void setPlanningContext(
      const ompl_interface::ModelBasedPlanningContextPtr& context) {
    context_ = context;
  }

  planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor() {
    return psm_;
  }

  std::vector<std::string> getJointNames() {
    return joint_model_group_->getActiveJointModelNames();
  }

  std::vector<double> getJointGoalPos() { return joint_goal_pos_; }

  std::shared_ptr<VisualizerData> getVisualizerData() { return vis_data_; }

  moveit::core::RobotModelPtr getRobotModel() { return robot_model_; }

  void convertTraj(std::vector<std::array<double, 7>>& joint_waypoints,
                   std::vector<std::array<double, 7>>& joint_velocities);

  void setPlannerName(std::string planner_name) {
    planner_name_ = planner_name;
  }

  const moveit::core::JointModelGroup* getJointModelGroup() {
    return joint_model_group_;
  }

 protected:
  ros::NodeHandle nh_;

  /** \brief Default robot being used.*/
  const std::string group_name_ = "panda_arm";

  /** \brief Planner used to solve motino planning problems.*/
  std::string planner_name_ = "panda_arm[RRTConnect]";

  /** \brief The class that's used to save information for visualization.*/
  std::shared_ptr<VisualizerData> vis_data_;

  /** \brief Number of degrees of freedom of the panda robot.*/
  const std::size_t dof_ = 7;

  /** \brief The goal pose for the robot. The robot will try to move to this
   * state when planning a trajectory.*/
  std::vector<double> joint_goal_pos_;

  /** \brief The resonse after a plan has been generated.*/
  planning_interface::MotionPlanResponse plan_response_;

  /** \brief These are all elements of the moveit interface that we use to set
   * up the planner. Please look into each of their individual documentations
   * for more information.*/
  const moveit::core::JointModelGroup* joint_model_group_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  moveit::core::RobotModelPtr robot_model_;
  kinematics_metrics::KinematicsMetricsPtr kinematics_metrics_;
  moveit::core::RobotStatePtr robot_state_;
  ompl_interface::ModelBasedPlanningContextPtr context_;
  ompl::base::OptimizationObjectivePtr optimization_objective_;
};
}  // namespace tacbot
#endif
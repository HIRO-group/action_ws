#ifndef CONTACT_PLANNER_H
#define CONTACT_PLANNER_H

// ROS
#include <ros/ros.h>

// C++
#include <algorithm>
#include <valarray>
#include <vector>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit/ompl_interface/detail/constrained_goal_sampler.h>
#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// OMPL
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/CVFRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/VFRRT.h>

#include "ompl/geometric/planners/est/EST.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/SVD>

struct Manipulability {
  Eigen::MatrixXd eigen_values;
  Eigen::MatrixXd eigen_vectors;

  Eigen::Vector3d getVector(std::size_t i) {
    Eigen::Vector3d eig_vec(eigen_vectors(0, i), eigen_vectors(1, i),
                            eigen_vectors(2, i));
    return eig_vec;
  }
  bool pass = false;
};

class ContactPlanner {
 public:
  void init();
  void setCurToStartState(planning_interface::MotionPlanRequest& req);
  ompl_interface::ModelBasedPlanningContextPtr createPlanningContext(
      const moveit_msgs::MotionPlanRequest& req, const ros::NodeHandle& nh);
  void changePlanner(ompl_interface::ModelBasedPlanningContextPtr& context);
  bool generatePlan(const ompl_interface::ModelBasedPlanningContextPtr& context,
                    planning_interface::MotionPlanResponse& res);

  moveit_msgs::Constraints createPoseGoal();
  moveit_msgs::Constraints createJointGoal();

  std::string getGroupName();

  void visualizeTrajectory(const planning_interface::MotionPlanResponse& res);
  void visualizeRepulsedState();
  void visualizeTreeStates();
  void visualizeGoalState();

  friend std::ostream& operator<<(std::ostream& os,
                                  const geometry_msgs::Pose& pose);
  void promptAnyInput();

 private:
  ros::NodeHandle nh_;
  const std::string group_name_ = "panda_arm";

  const Eigen::Vector3d obstacle_pos_{0.4, 0.0, 0.6};
  static const std::vector<double> joint_goal_pos_;

  const moveit::core::JointModelGroup* joint_model_group_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  moveit::core::RobotModelPtr robot_model_;
  kinematics_metrics::KinematicsMetricsPtr kinematics_metrics_;
  moveit::core::RobotStatePtr robot_state_;

  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

  ros::Publisher marker_pub_;
  ros::Publisher arrow_pub_;
  ros::Publisher manipulability_pub_;
  ros::Publisher trajectory_publisher_;
  ros::Publisher rep_state_publisher_;
  ros::Publisher tree_states_publisher_;
  ros::Publisher goal_state_publisher_;
  std::size_t dof_ = 7;  // get this from robot model

  std::vector<std::vector<double>> sample_joint_angles_;
  std::vector<std::vector<double>> sample_desired_angles_;
  std::vector<std::vector<double>> sample_final_angles_;

  std::size_t viz_state_idx_ = 0;
  std::size_t sample_state_count_ = 0;
  std::vector<Eigen::VectorXd> repulsed_vec_at_link_;
  std::vector<Eigen::VectorXd> repulsed_origin_at_link_;
  std::vector<std::vector<Manipulability>> manipulability_;

  void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_,
                     bool damped = true);

  void printPlannerConfigMap(
      const planning_interface::PlannerConfigurationMap& planner_config_map);
  void printJointTrajectory(
      const trajectory_msgs::JointTrajectory& joint_trajectory);
  void printStateSpace(
      const ompl_interface::ModelBasedStateSpacePtr& state_space);

  void visualizeManipVec(std::size_t state_num);
  void visualizeRepulseVec(std::size_t state_num);
  visualization_msgs::Marker getObstacleMarker();
  void visualizeRepulseOrigin(std::size_t state_num);
  void saveOriginVec(const Eigen::Vector3d& origin, const Eigen::Vector3d& vec,
                     std::size_t link_num);
  void saveJointAngles(const std::vector<double>& joint_angles);
  void saveRepulseAngles(const std::vector<double>& joint_angles,
                         const Eigen::VectorXd& d_q_out);

  std::unique_ptr<ompl_interface::OMPLInterface> getOMPLInterface(
      const moveit::core::RobotModelConstPtr& model, const ros::NodeHandle& nh);
  void setPlanningContextParams(
      ompl_interface::ModelBasedPlanningContextPtr& context);
  planning_interface::PlannerConfigurationSettings getPlannerConfigSettings(
      const planning_interface::PlannerConfigurationMap& pconfig_map,
      const std::string& planner_id);
  Eigen::Vector3d scaleToDist(Eigen::Vector3d vec);
  Eigen::MatrixXd getLinkPositions(moveit::core::RobotStatePtr robot_state);
  ompl::base::PlannerPtr createPlanner(
      const ompl::base::SpaceInformationPtr& si);

  Eigen::VectorXd obstacleField(const ompl::base::State* base_state,
                                const ompl::base::State* prev_state);
  Eigen::VectorXd goalField(const ompl::base::State* state);
  Eigen::VectorXd negGoalField(const ompl::base::State* state);
  Eigen::VectorXd totalField(const ompl::base::State* state);

  Eigen::VectorXd toEigen(std::vector<double> stl_vec);
  std::vector<double> toStlVec(Eigen::VectorXd eig_vec);
  std::vector<double> toStlVec(
      const ompl::base::RealVectorStateSpace::StateType& vec_state,
      std::size_t size);
};

#endif
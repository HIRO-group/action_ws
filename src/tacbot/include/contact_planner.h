#ifndef TACBOT_CONTACT_PLANNER_H
#define TACBOT_CONTACT_PLANNER_H

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
#include <ompl/geometric/planners/rrt/ClassicTRRT.h>
#include <ompl/geometric/planners/rrt/ContactTRRT.h>

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
#include "tacbot/TrajExecutionMonitor.h"
#include "utilities.h"
#include "visualizer.h"
#include "visualizer_data.h"

namespace tacbot {

/** \class This class is used to generate robot trajectories. Unlike traditional
 * motion planners, this planner can create a plan in which a robot will make
 * contact with obstacles. If there are obstacles in space but there is a
 * contact-free path then the robot will generate this path. If there are
 * obstacles in space and the planner needs to generate a plan such that some
 * obstacle interaction occurs, then the robot will still generate this plan.
 *
 * This class uses the ContactPerception class, which feeds the robot points
 * from a point cloud around the robot.
 * This class uses the ContactController, which then can execute a desired
 * trajectory. Finally, this class uses the VisualizerData class, where it
 * stores certain planning elements for visualization.
 *
 */
class ContactPlanner {
 public:
  ContactPlanner();

  /** \brief Initialize the planning scene, monitoring, publishers and
   * subscribers.*/
  void init();

  /** \brief Set the start state in the request message to the current state of
    the robot.
    @param req The motion planning request.
  */
  void setCurToStartState(planning_interface::MotionPlanRequest& req);

  /** \brief Create the moveit-based planning context given the specifications
     of the input request. This request specifies things like the planner name,
     the maximum planning time, certain limits, etc.
      @param req The motion planning request.
  */
  void createPlanningContext(const moveit_msgs::MotionPlanRequest& req);

  /** \brief Changes the planner from the default one that is native to the
     moveit environment, such as RRT, to one that has been specifically created
     for the contact motion planning library, such as ContactTRRT.
  */
  void changePlanner();

  /** \brief Calls on the context to use the input parameters and the planner
     setup to generate a trajectory for the robot to follow. Any changes to the
     planning request or context need to be done before this call.
      @param res The motion planning response where the trajectory and results
     will be stored.
      @return bool Whether or not the plan and time parameterization have been
     successfully accomplished.
  */
  bool generatePlan(planning_interface::MotionPlanResponse& res);

  /** \brief Creates a sample end-effector goal state, in cartesian space, for
     the robot to reach. Generally used to test the planner without having to
     look at the member variables of the Constraints message.
      @return moveit_msgs::Constraints The goal state and restrictions as
     specified by the moveit message.
  */
  moveit_msgs::Constraints createPoseGoal();

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

  /** \brief Getter for the positions of the simulated obstacles in the robot's
    planning scene. This will generally be used for visualization and debug
    messages.
    @return std::vector<Eigen::Vector3d> of the positions (x,y,z) of the
    obstacles.
  */
  std::vector<Eigen::Vector3d> getSimObstaclePos();

  /** \brief Get the planner that is set into the context at class
    initialization. This will be RRT or some other default class that is native
    to the moveit envrionment. This class will be used for planner generation
    unless changePlanner is called.
    @return string of the planner name. refer to ompl_planning.yaml for a full
    list of available planner.
  */
  std::string getDefaultPlannerId();

  /** \brief Getter for the moveit plannning context.
    @return Planning context.
  */
  ompl_interface::ModelBasedPlanningContextPtr getPlanningContext();

  /** \brief Publishes the generated trajectory over a ros topic. This
   * trajectory will get read by the controller to try and execute it.
   */
  void executeTrajectory();

  /** \brief As the trajectory is executed, the controlled publishes to a topic
   * which keeps track of which point on the trajectory gets executed, whether
   * or not there are errors, and what's the current robot state.
   * TrajExecutionMonitor.msg has been created for this purpose.
   */
  void monitorExecution();

  void runCollisionDetection();

 private:
  ros::NodeHandle nh_;

  /** \brief Default robot being used.*/
  const std::string group_name_ = "panda_arm";

  /** \brief Default planner set to the planning context.*/
  const std::string default_planner_id_ = "panda_arm[RRT]";

  /** \brief The class that handles point cloud processing of the surrounding
   * environment and transfers this information to the planner.*/
  std::shared_ptr<ContactPerception> contact_perception_;

  /** \brief The class that's used to save information for visualization.*/
  VisualizerData vis_data_;

  /** \brief Class that uses the vis_data_ class to visualize data that's saved
   * there.*/
  friend class Visualizer;

  /** \brief Number of degrees of freedom of the panda robot.*/
  const std::size_t dof_ = 7;

  /** \brief The number of samples that have been processed by the contact
   * planner class. As TRRT or other class generates random samples, this class
   * processes these samples and with each sample this counter gets
   * incremented.*/
  std::size_t sample_state_count_ = 0;

  /** \brief Publishes the generated trajectory. The controller subscribes to
   * this topic.*/
  ros::Publisher trajectory_pub_;

  /** \brief The planner subsribes to this topic to monitor whether there are
   * any error during execution. If there are, then the planner needs to update
   * its scene information.*/
  ros::Subscriber execution_monitor_sub_;

  /** \brief Mutex for the monitor_msg_.*/
  std::mutex monitor_mtx_;

  /** \brief Each time the executionMonitorCallback gets called, it writes to
   * this member. The planner monitors this variable after execution, which is
   * why we use a mutex for this member.*/
  tacbot::TrajExecutionMonitor monitor_msg_;

  /** \brief Whether or not to use simulated obstacles or the ContactPerception
   * class to fill obstacles into the robot's planning scene.*/
  const bool use_sim_obstacles_ = true;

  /** \brief The posisitoins of the simulated obstacles. It's a vector of the
   * x,y,z positions of the obstacles in cartesian space.*/
  std::vector<Eigen::Vector3d> sim_obstacle_pos_;

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

  std::vector<std::pair<Eigen::Vector3d, double>> spherical_obstacles_;

  /** \brief Get the ompl interface for the robot. The interface is part of the
    moveit infrastructure.
    @param model The robot model
    @param nh The ros nodehandle
    @return OMPLInterface moveit class
  */
  std::unique_ptr<ompl_interface::OMPLInterface> getOMPLInterface(
      const moveit::core::RobotModelConstPtr& model, const ros::NodeHandle& nh);

  /** \brief Set the paramters of the planning context. The parameters include
    things like whether or not to smooth the trajectory, the goal threshold, max
    goal samples, etc.
    @param context The planning context.
  */
  void setPlanningContextParams(
      ompl_interface::ModelBasedPlanningContextPtr& context);

  /** \brief Getter for the planner config parameters. We
    extract these parameters from the pconfig_map argument if they exist.
    @param pconfig_map The pconfig map.
    @param planner_id The planner name for which to read the parameters. For
    example, RRT.
    @return PlannerConfigurationSettings The planner settings.
 */
  planning_interface::PlannerConfigurationSettings getPlannerConfigSettings(
      const planning_interface::PlannerConfigurationMap& pconfig_map,
      const std::string& planner_id);

  /** \brief Generate a vector within a vector field of obstacles based on the
    robot state. Obstacles will generate a repulsive field and if the robot
    state is close to the obstacles then the repulsive field should be stronger.
    The implementation of this vector field is very important. This is one of
    the most important functions of this class.
    @param The input state. Generally generated by the sampling planner such as
    VFRRT.
    @return Eigen::VectorXd The output vector from the vector field.
  */
  Eigen::VectorXd obstacleField(const ompl::base::State* base_state);

  /** \brief A function that generates a vector which always points the robot
    towards the goal.
    @param state Planner generated robot state.
    @return Eigen::VectorXd The output vector from the vector field.
  */
  Eigen::VectorXd goalField(const ompl::base::State* state);

  /** \brief A function that generates a vector which always points the robot
    away from the goal. Used to prove the concept that a planner can overcome
    high resistance, go "against the current."
    @param state Planner generated robot state.
    @return Eigen::VectorXd The output vector from the vector field.
  */
  Eigen::VectorXd negGoalField(const ompl::base::State* state);

  /** \brief A function that takes several other vector field and adds them up.
    For example, the goal field and obstacle field could be added to push the
    robot away from obstacles but in the direction of the goal.
    @param state Planner generated robot state.
    @return Eigen::VectorXd The output vector from the vector field.
  */
  Eigen::VectorXd totalField(const ompl::base::State* state);

  /** \brief Scale the vector based on how close it is to an obstacle. The input
    vector is the difference between a point on a robot and a point on the
    obstacle. The output vector is the inverse of this. The longer the vector,
    the farther the robot is from the obstacle, the smaller is becomes in the
    output of this function.
    @param vec The input vector that needs to be scaled.
    @return The scaled vector.
  */
  Eigen::Vector3d scaleToDist(Eigen::Vector3d vec);

  /** \brief Obtain a set of points from a specific mesh of a robot link.
    @param robot_state The robot state at given joint state.
    @param link_model The model of the link for which to extract pts.
    @param link_pts The points on the link model.
    @param num_pts The accumulated number of points. Keeps getting incremented
    each time this function is called.
  */
  void extractPtsFromModel(const moveit::core::RobotStatePtr& robot_state,
                           const moveit::core::LinkModel* link_model,
                           std::vector<Eigen::Vector3d>& link_pts,
                           std::size_t& num_pts);

  /** \brief Obtain points on a robot surface. This is used to understand how
    far away the robot is from obstacles.
    @param robot_state The robot state that specifies the joint configuration
    which will be used to understand the tf for the points on the robot surface.
    @param rob_pts The points on the robot surface.
    @return std::size_t The total number of points extracted from the robot
    surface.
  */
  std::size_t getPtsOnRobotSurface(
      const moveit::core::RobotStatePtr& robot_state,
      std::vector<std::vector<Eigen::Vector3d>>& rob_pts);

  /** \brief
    @param rob_pts The Points on the robot surface.
    @return std::vector<Eigen::Vector3d> A set of repulsive vectors per link.
  */
  std::vector<Eigen::Vector3d> getLinkToObsVec(
      const std::vector<std::vector<Eigen::Vector3d>>& rob_pts);

  /** \brief Obtain the obstacles in the robot's surroundings. These could be
    simulated obstacles or obstacles from the ContactPerception class.
    @param pt_on_rob If we are not using simulated obstacles then this point is
    used in the ContactPerception class to obstain obstacles close enough to
    this point.
    @return std::vector<Eigen::Vector3d> A vector of obstacle positions.
  */
  std::vector<Eigen::Vector3d> getObstacles(const Eigen::Vector3d& pt_on_rob);

  /** \brief Callback function for the topic to which the controller advertises
    its execution state.
    @param msg The ros msg.
  */
  void executionMonitorCallback(const tacbot::TrajExecutionMonitor& msg);

  /** \brief If the trajectory execution was not successful then update the
   * environment. This function add new obstacles to the environment at the
   * point where execution has been stopped. */
  void updateObstacles();

  void addSphericalObstacle(const Eigen::Vector3d& center, double radius);
  void addLineObstacle();
};
}  // namespace tacbot
#endif
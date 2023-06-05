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
#include <ompl/base/objectives/VFMagnitudeOptimizationObjective.h>
#include <ompl/base/objectives/VFUpstreamCriterionOptimizationObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/rrt/ClassicTRRT.h>
#include <ompl/geometric/planners/rrt/ContactTRRT.h>
#include <ompl/geometric/planners/rrt/VFRRT.h>

#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/geometric/planners/informedtrees/BITstar.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <Eigen/SVD>

// Local libraries, helper functions, and utilities
#include "base_planner.h"
#include "contact_perception.h"
#include "manipulability_measures.h"
#include "utilities.h"
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
class ContactPlanner : public BasePlanner {
 public:
  ContactPlanner();

  /** \brief Initialize the planning scene, monitoring, publishers and
   * subscribers.*/
  void init();

  /** \brief Changes the planner from the default one that is native to the
     moveit environment, such as RRT, to one that has been specifically created
     for the contact motion planning library, such as ContactTRRT.
  */
  void changePlanner() override;

  /** \brief Getter for the positions of the simulated obstacles in the robot's
    planning scene. This will generally be used for visualization and debug
    messages.
    @return std::vector<Eigen::Vector3d> of the positions (x,y,z) of the
    obstacles.
  */
  std::vector<tacbot::ObstacleGroup> getSimObstaclePos();

  /** \brief Obtain the obstacles in the robot's surroundings. These could be
    simulated obstacles or obstacles from the ContactPerception class.
    @param pt_on_rob If we are not using simulated obstacles then this point is
    used in the ContactPerception class to obstain obstacles close enough to
    this point.
    @return std::vector<Eigen::Vector3d> A vector of obstacle positions.
  */
  std::vector<Eigen::Vector3d> getObstacles(const Eigen::Vector3d& pt_on_rob);

  void analyzePlanResponse(BenchMarkData& benchmark_data);
  void setObstacleScene(std::size_t option);
  void setGoalState(std::size_t option);

  void setObjectiveName(std::string objective_name) {
    objective_name_ = objective_name;
  }

  /** \brief The class that handles point cloud processing of the surrounding
   * environment and transfers this information to the planner.*/
  std::shared_ptr<ContactPerception> contact_perception_;

 private:
  std::string objective_name_ = "FieldAlign";

  /** \brief The number of samples that have been processed by the contact
   * planner class. As TRRT or other class generates random samples, this class
   * processes these samples and with each sample this counter gets
   * incremented.*/
  std::size_t sample_state_count_ = 0;

  /** \brief Whether or not to use simulated obstacles or the ContactPerception
   * class to fill obstacles into the robot's planning scene.*/
  const bool use_sim_obstacles_ = true;

  /** \brief The posisitoins of the simulated obstacles. It's a vector of the
   * x,y,z positions of the obstacles in cartesian space.*/
  std::vector<Eigen::Vector3d> sim_obstacle_pos_;

  /** \brief The goal pose for the robot. The robot will try to move to this
   * state when planning a trajectory.*/
  std::vector<std::vector<Eigen::Vector3d>> goal_rob_pts_;
  void extractPtsFromGoalState();

  Eigen::Vector3d getAttractPt(std::size_t link_num, std::size_t pt_num);

  Eigen::VectorXd getRobtPtsVecDiffAvg(
      const std::vector<std::vector<Eigen::Vector3d>>& near_state_rob_pts,
      const std::vector<std::vector<Eigen::Vector3d>>& rand_state_rob_pts,
      const std::vector<Eigen::Vector3d>& link_to_obs_vec);

  Eigen::VectorXd obstacleFieldCartesian(const ompl::base::State* near_state,
                                         const ompl::base::State* rand_state);

  std::vector<std::pair<Eigen::Vector3d, double>> spherical_obstacles_;

  /** \brief Generate a vector within a vector field of obstacles based on the
    robot state. Obstacles will generate a repulsive field and if the robot
    state is close to the obstacles then the repulsive field should be stronger.
    @param The input state. Generally generated by the sampling planner such as
    VFRRT.
    @return Eigen::VectorXd The output vector from the vector field.
  */
  Eigen::VectorXd obstacleFieldConfigSpace(const ompl::base::State* base_state);

  Eigen::VectorXd obstacleFieldTaskSpace(const ompl::base::State* base_state);

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

  void addSphericalObstacle(const Eigen::Vector3d& center, double radius);

  bool linkNameToIdx(const std::string& link_name, std::size_t& idx);
};
}  // namespace tacbot
#endif
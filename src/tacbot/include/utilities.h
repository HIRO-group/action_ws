#ifndef TACBOT_UTILITIES_H
#define TACBOT_UTILITIES_H

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <iostream>
#include <vector>
namespace tacbot {

// should be in its own header file

struct TrajectoryAnalysisData {
  std::vector<double> total_depth;
  std::vector<std::vector<double>> depth_per_link;
};

struct PlanAnalysisData {
  std::size_t total_contact_count = 0;
  std::size_t num_contact_states = 0;
  std::size_t num_path_states = 0;
  double total_contact_depth = 0.0;
  double joint_path_len = 0.0;
  double ee_path_len = 0.0;

  TrajectoryAnalysisData trajectory_analysis;
};

struct BenchMarkData {
  std::size_t test_num = 0;
  std::size_t success = 0;
  double plan_time = 0.0;
  std::string file_name = "";
  PlanAnalysisData plan_analysis;
};

struct PointObstacle {
  PointObstacle() : pos(Eigen::Vector3d{0, 0, 0}) {}
  Eigen::Vector3d pos;
};

struct ObstacleGroup {
  ObstacleGroup() : center(Eigen::Vector3d{0, 0, 0}) {}
  std::string name = "";
  std::vector<tacbot::PointObstacle> point_obstacles;
  Eigen::Vector3d center;  // optional for simulated obstacles
  double radius = 0.0;     // optinal simulated spherical obstacles
  double cost = 1.0;       //
  double MAX_COST = 10;
};

namespace utilities {
/** \brief
  @param
  @return
*/

/** \brief Waits for the user to press a key + Enter to continue the process.
 * @return Whether the use wants to continue (true) or quit (false).
 */
bool promptUserInput();

/** \brief Calculate the pseudo inverse of a matrix. Taken from the franka_ros
  package.
  @param M_ matrix
  @param M_pinv_ pseudo inverse output
  @param damped setting, not sure what it does
*/
void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_,
                   bool damped = true);

/** \brief Convert an stl vector to eigen.
  @param stl_vec Stl vector
  @return Eigen vector
*/
Eigen::VectorXd toEigen(std::vector<double> stl_vec);

/** \brief Convert an eigen vector to an stl vector.
  @param eig_vec Eigen vector
  @return Stl vector
*/
std::vector<double> toStlVec(Eigen::VectorXd eig_vec);

/** \brief Convert an ompl state type to stl vector.
  @param vec_state The robot state.
  @param size The number of elements in the vector
  @return Stl vector.
*/
std::vector<double> toStlVec(
    const ompl::base::RealVectorStateSpace::StateType& vec_state,
    std::size_t size);

/** \brief Get the straight-line distance between two points.
  @param p1 Cartesian coordinates of point 1.
  @param  p2 Cartesian coordinates of point 2.
  @return double Distance.
*/
double getDistance(Eigen::Vector3d p1, Eigen::Vector3d p2);

/** \brief Print the information stores in the planner configuration. Used to
  see whether or not data has been read from a yaml file.
  @param planner_config_map Data structure which stores planner
  configurations.
*/
void printPlannerConfigMap(
    const planning_interface::PlannerConfigurationMap& planner_config_map);

/** \brief Print the joint trajectory.
  @param joint_trajectory Joint trajectory of a robot.
\*/
void printJointTrajectory(
    const trajectory_msgs::JointTrajectory& joint_trajectory);

/** \brief Print the information of the state space.
  @param state_space The state space.
*/
void printStateSpace(
    const ompl_interface::ModelBasedStateSpacePtr& state_space);

/** \brief Interpolate between the rows of points in the matrix. Assume that
  each row stores cartesian coordinates of a point. If the distance between
  these two points is too large then we interpolate between the points and
  insert another row between the previous two. The process continues until the
  desires distances between the points have been reached.
  @param mat The matrix which stores the points in rows.
*/
void interpolate(Eigen::MatrixXd& mat);

/** \brief Find the rows of one matrix in another matrix. Assumes that at one
  point haystack was created from needle and now we need to understand where
  the original points are.
  @param needle The rows which need to be found.
  @param haystack The rows in which we want to find an equivalent.
  @return The indexes in haystack where we find equivalent needle rows.
*/
std::vector<std::size_t> find(const Eigen::MatrixXd& needle,
                              const Eigen::MatrixXd& haystack);

/** \brief Helper function to pring out a ros msg.
  @param os Output stream
  @param pose Ros goemetry pose message.
  @return Output stream with the pose printed.
*/
std::ostream& operator<<(std::ostream& os, const geometry_msgs::Pose& pose);

/** \brief Creates a sample end-effector goal state, in cartesian space, for
   the robot to reach. Generally used to test the planner without having to
   look at the member variables of the Constraints message.
    @return moveit_msgs::Constraints The goal state and restrictions as
   specified by the moveit message.
*/
moveit_msgs::Constraints createPoseGoal();

void toControlTrajectory(const moveit_msgs::MotionPlanResponse& msg,
                         std::vector<std::array<double, 7>>& joint_waypoints,
                         std::vector<std::array<double, 7>>& joint_velocities);

bool linkNameToIdx(const std::string& link_name, std::size_t& idx);

template <typename T>
std::vector<T> slice(std::vector<T> const& v, int m, int n) {
  auto first = v.cbegin() + m;
  auto last = v.cbegin() + n + 1;

  std::vector<T> vec(first, last);
  return vec;
}

}  // namespace utilities
}  // namespace tacbot
#endif
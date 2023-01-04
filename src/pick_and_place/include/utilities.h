#ifndef UTILITIES_H
#define UTILITIES_H

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <iostream>
#include <vector>
namespace pick_and_place {
namespace utilities {
/** \brief
  @param
  @return
*/

/** \brief Waits for the user to press a key + Enter to continue the process.
 */
void promptAnyInput();

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
  @param planner_config_map Data structure which stores planner configurations.
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
  point haystack was created from needle and now we need to understand where the
  original points are.
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

}  // namespace utilities
}  // namespace pick_and_place
#endif
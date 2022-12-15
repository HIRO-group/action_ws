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

void promptAnyInput();

void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_,
                   bool damped = true);
Eigen::VectorXd toEigen(std::vector<double> stl_vec);
std::vector<double> toStlVec(Eigen::VectorXd eig_vec);
std::vector<double> toStlVec(
    const ompl::base::RealVectorStateSpace::StateType& vec_state,
    std::size_t size);
double getDistance(Eigen::Vector3d p1, Eigen::Vector3d p2);
void printPlannerConfigMap(
    const planning_interface::PlannerConfigurationMap& planner_config_map);
void printJointTrajectory(
    const trajectory_msgs::JointTrajectory& joint_trajectory);
void printStateSpace(
    const ompl_interface::ModelBasedStateSpacePtr& state_space);

std::ostream& operator<<(std::ostream& os, const geometry_msgs::Pose& pose);

}  // namespace utilities
}  // namespace pick_and_place
#endif
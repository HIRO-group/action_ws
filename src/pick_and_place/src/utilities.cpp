#include "utilities.h"

#include <Eigen/SVD>

constexpr char LOGNAME[] = "utilites";

namespace pick_and_place {
namespace utilities {
void promptAnyInput() {
  std::string user_input = " ";
  std::cout << "Press any key to continue..." << std::endl;
  std::cin >> user_input;
}

void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_,
                   bool damped) {
  double lambda_ = damped ? 0.2 : 0.0;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ =
      svd.singularValues();
  Eigen::MatrixXd S_ =
      M_;  // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
    S_(i, i) =
        (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() *
                            svd.matrixU().transpose());
}

Eigen::VectorXd toEigen(std::vector<double> stl_vec) {  // const
  std::size_t size = stl_vec.size();
  Eigen::VectorXd eig_vec = Eigen::VectorXd::Zero(size);
  for (std::size_t i = 0; i < size; i++) {
    eig_vec[i] = stl_vec[i];
  }
  return eig_vec;
}

std::vector<double> toStlVec(Eigen::VectorXd eig_vec) {  // const
  std::size_t size = eig_vec.size();
  std::vector<double> stl_vec(size, 0.0);
  for (std::size_t i = 0; i < size; i++) {
    stl_vec[i] = eig_vec[i];
  }
  return stl_vec;
}

std::vector<double> toStlVec(
    const ompl::base::RealVectorStateSpace::StateType& vec_state,
    std::size_t size) {  // const
  std::vector<double> stl_vec(size, 0.0);
  for (std::size_t i = 0; i < size; i++) {
    stl_vec[i] = vec_state[i];
  }
  return stl_vec;
}

double getDistance(Eigen::Vector3d p1, Eigen::Vector3d p2) {
  return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2) +
              pow(p2[2] - p1[2], 2));
}

void printStateSpace(
    const ompl_interface::ModelBasedStateSpacePtr& state_space) {
  int type = state_space->getType();
  ROS_INFO_NAMED(LOGNAME, "state type %d", type);
  double measure = state_space->getMeasure();
  ROS_INFO_NAMED(LOGNAME, "state measure %f", measure);
  int dim = state_space->getDimension();
  ROS_INFO_NAMED(LOGNAME, "dimension %d", dim);
  state_space->Diagram(std::cout);
  state_space->List(std::cout);
}

void printPlannerConfigMap(
    const planning_interface::PlannerConfigurationMap& planner_config_map) {
  for (auto config : planner_config_map) {
    ROS_INFO_NAMED(LOGNAME, "Map Name: %s", config.first.c_str());
    ROS_INFO_NAMED(LOGNAME, "\tGroup: %s", config.second.group.c_str());
    ROS_INFO_NAMED(LOGNAME, "\tName: %s", config.second.name.c_str());

    for (auto setting : config.second.config) {
      ROS_INFO_NAMED(LOGNAME, "\t\tSetting: %s", setting.first.c_str());
      ROS_INFO_NAMED(LOGNAME, "\t\tValue: %s", setting.second.c_str());
    }
  }
}

void printJointTrajectory(
    const trajectory_msgs::JointTrajectory& joint_trajectory) {
  ROS_INFO_NAMED(LOGNAME, "Num joints: %ld",
                 joint_trajectory.joint_names.size());
  ROS_INFO_NAMED(LOGNAME, "Num points: %ld", joint_trajectory.points.size());

  for (int i = 0; i < joint_trajectory.joint_names.size(); i++) {
    std::string name = joint_trajectory.joint_names[i];
    std::cout << name << " ";
  }

  for (int i = 0; i < joint_trajectory.points.size(); i++) {
    trajectory_msgs::JointTrajectoryPoint traj_point =
        joint_trajectory.points[i];
    std::cout << "\npositions" << std::endl;
    for (int k = 0; k < traj_point.positions.size(); k++) {
      double position = traj_point.positions[k];
      std::cout << position << " ";
    }

    std::cout << "\nvelocities" << std::endl;
    for (int k = 0; k < traj_point.velocities.size(); k++) {
      double velocity = traj_point.velocities[k];
      std::cout << velocity << " ";
    }

    std::cout << "\naccelerations" << std::endl;
    for (int k = 0; k < traj_point.accelerations.size(); k++) {
      double accel = traj_point.accelerations[k];
      std::cout << accel << " ";
    }

    std::cout << "\neffort" << std::endl;
    for (int k = 0; k < traj_point.effort.size(); k++) {
      double effort = traj_point.effort[k];
      std::cout << effort << " ";
    }
  }
}

void interpolate(Eigen::MatrixXd& mat) {
  std::cout << "mat\n: " << mat << std::endl;
  const double max_dist = 0.05;

  std::size_t i = 1;
  while (i < mat.rows() - 1) {
    // std::cout << "i: " << i << std::endl;
    // std::cout << "mat.rows(): " << mat.rows() << std::endl;
    // std::cout << "mat.cols(): " << mat.cols() << std::endl;
    // std::cout << "mat\n: " << mat << std::endl;

    Eigen::Vector3d p1(mat(i - 1, 0), mat(i - 1, 1), mat(i - 1, 2));
    Eigen::Vector3d p2(mat(i, 0), mat(i, 1), mat(i, 2));
    double dist = utilities::getDistance(p1, p2);
    // std::cout << "p1: " << p1.transpose() << std::endl;
    // std::cout << "p2: " << p2.transpose() << std::endl;
    // std::cout << "dist: " << dist << std::endl;

    if (dist > max_dist) {
      Eigen::MatrixXd imat = Eigen::MatrixXd::Zero(mat.rows() + 1, 3);
      imat.topRows(i) = mat.topRows(i);
      imat.bottomRows(mat.rows() - i) = mat.bottomRows(mat.rows() - i);
      Eigen::Vector3d ivec((p2[0] + p1[0]) / 2.0, (p2[1] + p1[1]) / 2.0,
                           (p2[2] + p1[2]) / 2.0);
      // std::cout << "ivec: " << ivec.transpose() << std::endl;

      imat(i, 0) = ivec[0];
      imat(i, 1) = ivec[1];
      imat(i, 2) = ivec[2];
      // std::cout << "mat\n: " << mat << std::endl;
      // std::cout << "imat\n: " << imat << std::endl;
      mat = imat;
    } else {
      i++;
    }
  }
  std::cout << "imat\n: " << mat << std::endl;
}

std::vector<std::size_t> find(const Eigen::MatrixXd& needle,
                              const Eigen::MatrixXd& haystack) {
  std::size_t link_idx = 0;
  std::vector<std::size_t> idx_out(needle.rows(), 0);

  for (std::size_t i = 0; i < haystack.rows(); i++) {
    Eigen::Vector3d pos(needle(link_idx, 0), needle(link_idx, 1),
                        needle(link_idx, 2));
    Eigen::Vector3d vec(haystack(i, 0), haystack(i, 1), haystack(i, 2));
    double dist = utilities::getDistance(pos, vec);
    if (dist < 0.0001) {
      idx_out[link_idx] = i;
      // std::cout << "findLinkIdx i: " << i << std::endl;
      link_idx++;
    }
  }
  // std::cout << "needle.rows(): " << needle.rows() <<
  // std::endl; std::cout << "idx_out.size(): " << idx_out.size() << std::endl;

  return idx_out;
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::Pose& pose) {
  os << "position (x,y,z): " << pose.position.x << ", " << pose.position.y
     << ", " << pose.position.z << "\n";
  os << "orientation (x,y,z,w): " << pose.orientation.x << ", "
     << pose.orientation.y << ", " << pose.orientation.z << ", "
     << pose.orientation.w << "\n";
  return os;
}

}  // namespace utilities
}  // namespace pick_and_place
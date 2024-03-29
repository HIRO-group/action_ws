#include "utilities.h"

#include <Eigen/SVD>

constexpr char LOGNAME[] = "utilites";

namespace tacbot {
namespace utilities {
bool promptUserInput() {
  std::cin.clear();
  // std::cin.ignore(100, '\n');
  std::cout << "Press any key to continue or q to quit ... " << std::endl;
  std::string user_input = " ";
  std::cin >> user_input;
  if (user_input == "q") {
    return false;
  } else {
    return true;
  }
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

moveit_msgs::Constraints createPoseGoal() {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "panda_link0";
  pose.pose.position.x = 0.5;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints(
          "panda_link8", pose, tolerance_pose, tolerance_angle);
  return pose_goal;
}

void toControlTrajectory(const moveit_msgs::MotionPlanResponse& msg,
                         std::vector<std::array<double, 7>>& joint_waypoints,
                         std::vector<std::array<double, 7>>& joint_velocities) {
  ROS_INFO_NAMED(LOGNAME, "Creating joint velovity trajectory. ");

  std::size_t num_pts = msg.trajectory.joint_trajectory.points.size();

  for (std::size_t pt_idx = 1; pt_idx < num_pts; pt_idx++) {
    // ROS_INFO_NAMED(LOGNAME, "Converting trajectory point number: %ld",
    // pt_idx);
    trajectory_msgs::JointTrajectoryPoint point =
        msg.trajectory.joint_trajectory.points[pt_idx];
    std::array<double, 7> joint_angles;
    std::array<double, 7> joint_velocity_pt;

    for (std::size_t jnt_idx = 0; jnt_idx < 7; jnt_idx++) {
      joint_angles[jnt_idx] = point.positions[jnt_idx];
      joint_velocity_pt[jnt_idx] = point.velocities[jnt_idx];
      // if (std::abs(point.velocities[jnt_idx] > 1)) {
      //   ROS_INFO_NAMED(LOGNAME, "Trajectory point number: %ld", pt_idx);
      //   ROS_INFO_NAMED(LOGNAME, "Velocity: %f", point.velocities[jnt_idx]);
      // }
    }
    joint_waypoints.emplace_back(joint_angles);
    joint_velocities.emplace_back(joint_velocity_pt);

    // ROS_INFO_NAMED(LOGNAME, "time_from_start: %f",
    //                point.time_from_start.toSec());

    trajectory_msgs::JointTrajectoryPoint point_a =
        msg.trajectory.joint_trajectory.points[pt_idx - 1];

    trajectory_msgs::JointTrajectoryPoint point_b =
        msg.trajectory.joint_trajectory.points[pt_idx];

    double time_a = point_a.time_from_start.toSec();
    double time_b = point_b.time_from_start.toSec();

    double time_diff = time_b - time_a;

    // ROS_INFO_NAMED(LOGNAME, "time_diff: %f", time_diff);
  }
}

bool linkNameToIdx(const std::string& link_name, std::size_t& idx) {
  std::vector<std::string> link_names{
      "panda_link0", "panda_link1", "panda_link2", "panda_link3", "panda_link4",
      "panda_link5", "panda_link6", "panda_link7", "panda_hand"};

  if (link_name == "panda_rightfinger" || link_name == "panda_leftfinger" ||
      link_name == "panda_rightfinger_sc" ||
      link_name == "panda_leftfinger_sc") {
    idx = 6;
    return true;
  }

  auto it = std::find(link_names.begin(), link_names.end(), link_name);
  if (it == link_names.end()) {
    // ROS_ERROR_NAMED(LOGNAME, "Unable to find the following link in model:
    // %s",
    //                 link_name.c_str());
    // return false;
  } else {
    idx = std::distance(link_names.begin(), it);
    // ROS_INFO_NAMED(LOGNAME, "link_name: %s, idx %ld", link_name.c_str(),
    // idx);
    if (idx > 6) idx = 6;
    return true;
  }

  std::vector<std::string> link_names2{
      "panda_link0_sc", "panda_link1_sc", "panda_link2_sc",
      "panda_link3_sc", "panda_link4_sc", "panda_link5_sc",
      "panda_link6_sc", "panda_link7_sc", "panda_hand_sc"};

  auto it2 = std::find(link_names2.begin(), link_names2.end(), link_name);
  if (it2 == link_names2.end()) {
    ROS_ERROR_NAMED(LOGNAME, "Unable to find the following link in model: %s",
                    link_name.c_str());
    // return false;
  } else {
    idx = std::distance(link_names2.begin(), it2);
    // ROS_INFO_NAMED(LOGNAME, "link_name: %s, idx %ld", link_name.c_str(),
    // idx);
    if (idx > 6) idx = 6;
    return true;
  }

  return false;
}

}  // namespace utilities
}  // namespace tacbot
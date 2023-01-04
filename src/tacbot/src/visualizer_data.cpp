#include "visualizer_data.h"

namespace tacbot {
void VisualizerData::setTotalNumRepulsePts(std::size_t num_pts) {
  cur_total_num_repulse_pts_ = num_pts;
}

void VisualizerData::saveOriginVec(const Eigen::Vector3d& origin,
                                   const Eigen::Vector3d& vec,
                                   std::size_t pt_num,
                                   std::size_t sample_state_count) {
  if (sample_state_count == repulsed_vec_at_link_.size()) {
    repulsed_vec_at_link_.emplace_back(
        Eigen::VectorXd::Zero(cur_total_num_repulse_pts_ * 3));
    repulsed_origin_at_link_.emplace_back(
        Eigen::VectorXd::Zero(cur_total_num_repulse_pts_ * 3));
  }

  Eigen::VectorXd& cur_repulsed = repulsed_vec_at_link_[sample_state_count];
  cur_repulsed[pt_num * 3] = vec[0];
  cur_repulsed[pt_num * 3 + 1] = vec[1];
  cur_repulsed[pt_num * 3 + 2] = vec[2];

  Eigen::VectorXd& cur_origin = repulsed_origin_at_link_[sample_state_count];
  cur_origin[pt_num * 3] = origin[0];
  cur_origin[pt_num * 3 + 1] = origin[1];
  cur_origin[pt_num * 3 + 2] = origin[2];
}

void VisualizerData::saveJointAngles(const std::vector<double>& joint_angles) {
  sample_joint_angles_.emplace_back(joint_angles);
}

void VisualizerData::saveRepulseAngles(const std::vector<double>& joint_angles,
                                       const Eigen::VectorXd& d_q_out) {
  saveJointAngles(joint_angles);
  sample_desired_angles_.emplace_back(
      utilities::toStlVec(utilities::toEigen(joint_angles) + d_q_out));
}

void VisualizerData::saveObstaclePos(
    const std::vector<Eigen::Vector3d>& obstacle_pos,
    std::size_t sample_state_count) {
  std::size_t num_saved = sample_obstacle_pos_.size();

  if (num_saved == sample_state_count) {
    sample_obstacle_pos_.emplace_back(obstacle_pos);
  } else {
    std::vector<Eigen::Vector3d>& cur_obs_vec =
        sample_obstacle_pos_[sample_state_count];
    cur_obs_vec.insert(std::end(cur_obs_vec), std::begin(obstacle_pos),
                       std::end(obstacle_pos));
  }
}

}  // namespace tacbot
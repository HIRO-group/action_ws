#ifndef VISUALIZER_DATA_H
#define VISUALIZER_DATA_H
#include <manipulability_measures.h>

#include <vector>

#include "utilities.h"

namespace pick_and_place {
struct VisualizerData {
  std::vector<std::vector<double>> sample_joint_angles_;
  std::vector<std::vector<double>> sample_desired_angles_;
  std::vector<std::vector<double>> sample_final_angles_;
  std::vector<std::vector<Eigen::Vector3d>> sample_obstacle_pos_;
  std::vector<std::vector<ManipulabilityMeasures>> manipulability_;
  std::vector<Eigen::VectorXd> repulsed_vec_at_link_;
  std::vector<Eigen::VectorXd> repulsed_origin_at_link_;
  std::size_t cur_total_num_repulse_pts_;

  void saveOriginVec(const Eigen::Vector3d& origin, const Eigen::Vector3d& vec,
                     std::size_t pt_num, std::size_t sample_state_count);

  void saveJointAngles(const std::vector<double>& joint_angles);

  void saveRepulseAngles(const std::vector<double>& joint_angles,
                         const Eigen::VectorXd& d_q_out);

  void saveObstaclePos(const std::vector<Eigen::Vector3d>& obstacle_pos,
                       std::size_t sample_state_count);

  void setTotalNumRepulsePts(std::size_t num_pts);
};

}  // namespace pick_and_place

#endif
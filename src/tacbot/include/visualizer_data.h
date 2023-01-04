#ifndef VISUALIZER_DATA_H
#define VISUALIZER_DATA_H
#include <manipulability_measures.h>

#include <vector>

#include "utilities.h"

namespace tacbot {

/** \class This struct is mainly used as a storage device. The ContactPlanner
 * stores its information here, and the Visualizer class will retrieve this
 * information from the ContactPlanner to visualize it.
 */
struct VisualizerData {
  /** \brief The robot joint angles as sampled by the planner.*/
  std::vector<std::vector<double>> sample_joint_angles_;

  /** \brief The robot's angles after a repulsion has been applied to them.*/
  std::vector<std::vector<double>> sample_desired_angles_;

  /** \brief The robot's angles if multiple vectors (attractive, repulsive) have
   * been applied to them.*/
  std::vector<std::vector<double>> sample_final_angles_;

  /** \brief The positions of the obstacles that were used to calculate
   * repulsion at each state.*/
  std::vector<std::vector<Eigen::Vector3d>> sample_obstacle_pos_;

  /** \brief The manipulability measure and ellipsoid information at each robot
   * state and at each link.*/
  std::vector<std::vector<ManipulabilityMeasures>> manipulability_;

  /** \brief The repulsed vectors at each robot state.*/
  std::vector<Eigen::VectorXd> repulsed_vec_at_link_;

  /** \brief The repulse vector origin at each robot state. */
  std::vector<Eigen::VectorXd> repulsed_origin_at_link_;

  /** \brief The number of repulse points at any given time. This number changes
   * during trajectory generation. It is used to resize vectors which store
   * information.*/
  std::size_t cur_total_num_repulse_pts_;

  /** \brief Stores the origin and direction of the repulsion vector for a robot
    state.
    @param origin The origin of the vector.
    @param vec The direciton of the vector.
    @param pt_num The vector number for the robot state. Since we sample a set
    of points over the robot surface, each state will have multiple vectors and
    this specifies the vector number in that state.
    @param sample_state_count The robot state number, sampled by the planner.
  */
  void saveOriginVec(const Eigen::Vector3d& origin, const Eigen::Vector3d& vec,
                     std::size_t pt_num, std::size_t sample_state_count);

  /** \brief Store the joint angles sampled by the planner.
    @param joint_angles
  */
  void saveJointAngles(const std::vector<double>& joint_angles);

  /** \brief Store the robot angles after a repulsion force has been applied to
    them.
    @param joint_angles The original joint angles.
    @param d_q_out The angles after repulsion applied.
  */
  void saveRepulseAngles(const std::vector<double>& joint_angles,
                         const Eigen::VectorXd& d_q_out);

  /** \brief Save the obstacle position at the robot's state.
    @param obstacle_pos The positions of obstacles in cartesian space.
    @param sample_state_count The robot state during which these obstacles act
    on the robot.
  */
  void saveObstaclePos(const std::vector<Eigen::Vector3d>& obstacle_pos,
                       std::size_t sample_state_count);

  /** \brief Save the total number of points that were sampled over the robot.
    @param num_pts The total number of points sampled across the robot's body.
  */
  void setTotalNumRepulsePts(std::size_t num_pts);
};

}  // namespace tacbot

#endif
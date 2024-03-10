#ifndef TACBOT_PERCEPTION_PLANNER_H
#define TACBOT_PERCEPTION_PLANNER_H

#include "base_planner.h"
#include "contact_perception.h"
#include "my_moveit_context.h"
#include "utilities.h"

namespace tacbot {

class PerceptionPlanner : public BasePlanner {
 public:
  PerceptionPlanner();

  /** \brief Changes the planner from the default one that is native to the
     moveit environment, such as RRT, to one that has been specifically selected
     for this perception planning class.
  */
  void changePlanner() override;

  void init() override;

  void setGoalState(std::size_t option);

  void setObstacleScene(std::size_t option);

  std::vector<tacbot::ObstacleGroup> getObstacles() { return obstacles_; };

  void createPandaBundleContext();

 protected:
  std::shared_ptr<MyMoveitContext> pandaBundleContext_;

  std::vector<tacbot::ObstacleGroup> obstacles_;

  /** \brief Whether or not to use simulated obstacles or the ContactPerception
   * class to fill obstacles into the robot's planning scene.*/
  const bool use_sim_obstacles_ = true;

  /** \brief The number of samples that have been processed by the contact
   * planner class. As TRRT or other class generates random samples, this class
   * processes these samples and with each sample this counter gets
   * incremented.*/
  std::size_t sample_state_count_ = 0;

  /** \brief The goal pose for the robot. The robot will try to move to this
   * state when planning a trajectory.*/
  std::vector<std::vector<Eigen::Vector3d>> goal_rob_pts_;

  /** \brief The posisitoins of the simulated obstacles. It's a vector of the
   * x,y,z positions of the obstacles in cartesian space.*/
  std::vector<Eigen::Vector3d> sim_obstacle_pos_;

  /** \brief The class that handles point cloud processing of the surrounding
   * environment and transfers this information to the planner.*/
  std::shared_ptr<ContactPerception> contact_perception_;

  void addPointObstacles(tacbot::ObstacleGroup& obstacle);

  void setCollisionChecker(std::string collision_checker_name);

  double getContactDepth(moveit::core::RobotState robot_state);

  double overlapMagnitude(const ompl::base::State* base_state);

  void sphericalCollisionPermission(bool is_allowed);

  bool findObstacleByName(const std::string& name,
                          tacbot::ObstacleGroup& obstacle);

  void tableCollisionPermission();

  Eigen::VectorXd getPerLinkContactDepth(moveit::core::RobotState robot_state);
  Eigen::VectorXd obstacleFieldDuo(const ompl::base::State* near_state,
                                   const ompl::base::State* rand_state);
  Eigen::VectorXd obstacleField(const ompl::base::State* rand_state);

  std::vector<Eigen::Vector3d> getLinkToObsVec(
      const std::vector<std::vector<Eigen::Vector3d>>& rob_pts);
  void extractPtsFromModel(const moveit::core::RobotStatePtr& robot_state,
                           const moveit::core::LinkModel* link_model,
                           std::vector<Eigen::Vector3d>& link_pts,
                           std::size_t& num_pts);
  std::size_t getPtsOnRobotSurface(
      const moveit::core::RobotStatePtr& robot_state,
      std::vector<std::vector<Eigen::Vector3d>>& rob_pts);
  Eigen::VectorXd obstacleFieldCartesian(const ompl::base::State* near_state,
                                         const ompl::base::State* rand_state);
  Eigen::VectorXd getRobtPtsVecDiffAvg(
      const std::vector<std::vector<Eigen::Vector3d>>& near_state_rob_pts,
      const std::vector<std::vector<Eigen::Vector3d>>& rand_state_rob_pts,
      const std::vector<Eigen::Vector3d>& link_to_obs_vec);

  /** \brief Scale the vector based on how close it is to an obstacle. The input
    vector is the difference between a point on a robot and a point on the
    obstacle. The output vector is the inverse of this. The longer the vector,
    the farther the robot is from the obstacle, the smaller is becomes in the
    output of this function.
    @param vec The input vector that needs to be scaled.
    @return The scaled vector.
  */
  Eigen::Vector3d scaleToDist(Eigen::Vector3d vec);

  Eigen::Vector3d getAttractPt(std::size_t link_num, std::size_t pt_num);
  /** \brief Obtain the obstacles in the robot's surroundings. These could be
  simulated obstacles or obstacles from the ContactPerception class.
  @param pt_on_rob If we are not using simulated obstacles then this point is
  used in the ContactPerception class to obstain obstacles close enough to
  this point.
  @return std::vector<Eigen::Vector3d> A vector of obstacle positions.
*/
  std::vector<Eigen::Vector3d> getObstacles(const Eigen::Vector3d& pt_on_rob);
};

}  // namespace tacbot
#endif
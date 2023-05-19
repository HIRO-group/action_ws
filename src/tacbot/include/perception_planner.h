#ifndef TACBOT_PERCEPTION_PLANNER_H
#define TACBOT_PERCEPTION_PLANNER_H

#include "base_planner.h"
#include "contact_perception.h"

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

  void addSphericalObstacle(const Eigen::Vector3d& center, double radius);

  /** \brief Getter for the positions of the simulated obstacles in the robot's
    planning scene. This will generally be used for visualization and debug
    messages.
    @return std::vector<Eigen::Vector3d> of the positions (x,y,z) of the
    obstacles.
  */
  std::vector<Eigen::Vector3d> getObstaclePos() { return sim_obstacle_pos_; };

  bool generatePlan(planning_interface::MotionPlanResponse& res) override;

 protected:
  /** \brief The posisitoins of the simulated obstacles. It's a vector of the
   * x,y,z positions of the obstacles in cartesian space.*/
  std::vector<Eigen::Vector3d> sim_obstacle_pos_;

  std::vector<std::pair<Eigen::Vector3d, double>> spherical_obstacles_;

  /** \brief The class that handles point cloud processing of the surrounding
   * environment and transfers this information to the planner.*/
  std::shared_ptr<ContactPerception> contact_perception_;

  void setCollisionChecker(std::string collision_checker_name);

  double getContactDepth(moveit::core::RobotState robot_state);

  double overlapMagnitude(const ompl::base::State* base_state);

  void sphericalCollisionPermission(bool is_allowed);
};
}  // namespace tacbot
#endif
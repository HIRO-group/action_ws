#ifndef TACBOT_PERCEPTION_PLANNER_H
#define TACBOT_PERCEPTION_PLANNER_H

#include "base_planner.h"
#include "contact_perception.h"
#include "my_moveit_context.h"

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
};

}  // namespace tacbot
#endif
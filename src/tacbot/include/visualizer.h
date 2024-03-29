#ifndef TACBOT_VISUALIZER_H
#define TACBOT_VISUALIZER_H

// ROS
#include <ros/ros.h>

#include "utilities.h"
#include "visualizer_data.h"

namespace tacbot {

struct g_hsv {
  float h;
  float s;
  float v;
};

/** \class The role of this class if to use VisualizerData and ContactPlanner to
 * send visualizing elements to rviz. This allows us to debug and see the
 * perception, planner, and controller system and how they operate.
 */
class Visualizer {
 public:
  /** \brief Constructor, initializes the publishers.
   */
  Visualizer(const std::shared_ptr<VisualizerData>& vis_data);

  /** \brief Visualize a repulsive vector which is applied by the obstacles to
     the robot.
      @param state_num The index number of the vector within an array.
  */
  void visualizeRepulseVec(std::size_t state_num);

  void visualizeNearRandVec(std::size_t state_num);

  /** \brief Visualize the origin of the repulsive vector.
      @param state_num The index of the origin within an array.
  */
  void visualizeRepulseOrigin(std::size_t state_num);

  void visualizePoints(const std::vector<geometry_msgs::Point>& points);

  /** \brief Visualize the obstacle in the scne.
      @param obstacle_pos The vecetor of obstacle positions in cartesian space.
  */
  void visualizeObstacleMarker(
      const std::vector<tacbot::ObstacleGroup>& obstacle_pos);

  /** \brief Visualize the robot joint start state, it's joint position when the
   * repulsed vectors are pplied, the vectors and their origins. Basically,
   * visualize everything that has to do with the repulsion mechanism of the
   * contact planner.
   */
  void visualizeRepulsedState(const std::vector<std::string>& names);

  /** \brief Visualize the goal state to which the planner wants the robot to go
     to. This should be the final state in the robot's trajectory.
  */
  void visualizeGoalState(const std::vector<std::string>& names,
                          const std::vector<double>& joint_goal_pos);

  /** \brief Visualize two robot states.
      @param joint_angles1 The robot's joint angles in the first state.
      @param joint_angles1 The robot's joint angles in the second state.
  */
  void visualizeTwoStates(const std::vector<std::string>& names,
                          std::vector<double> joint_angles1,
                          std::vector<double> joint_angles2);

  void visualizeTrajectory(const moveit_msgs::MotionPlanResponse& traj,
                           std::string name);
  void visualizeTrajectory(const robot_trajectory::RobotTrajectoryPtr& traj,
                           std::string name);

  void visualizeEEPath();

  void HSVToRGB(struct g_hsv& hsv, std_msgs::ColorRGBA& rgb);

  void setRGB(std_msgs::ColorRGBA& rgb, double r, double g, double b);

  void computeColorForValue(std_msgs::ColorRGBA& color, double gradientValue,
                            double maxValue);

 private:
  ros::NodeHandle nh_;
  std::size_t dof_ = 7;
  const std::string group_name_ = "panda_arm";

  /** \brief Publisher for the origin of the repulse vectors. */
  ros::Publisher robot_repulse_origin_pub_;

  /** \brief Publisher for the obstacles in the robot's space. */
  ros::Publisher obstacle_marker_pub_;

  /** \brief  Publisher for the vectors that show the repulsion of the robot
   * link from obstacles. */
  ros::Publisher arrow_pub_;

  ros::Publisher nearrand_pub_;

  ros::Publisher ee_path_pub_;

  /** \brief Publisher for the repulsed robot states. */
  ros::Publisher rep_state_publisher_;

  /** \brief Publisher for the final goal state of the robot's trajectory. */
  ros::Publisher goal_state_publisher_;

  /** \brief Publisher for all the trajectories that we desire to display.
   * Useful for comparing trajectories generated by different planners such as
   * RRT and TRRT. */
  std::vector<ros::Publisher> trajectory_publishers_;

  /** \brief We publish the repulsed states one by one in the sequence that they
   * were sampled by the planner. This index keeps track of which state's
   * properties we are visualizing at any given time. If there is an array of
   * vectors (such as repulse vectors) then we use this index to access a
   * specific vector within an array. */
  std::size_t viz_state_idx_ = 0;

  std::shared_ptr<VisualizerData> vis_data_;
};
}  // namespace tacbot

#endif
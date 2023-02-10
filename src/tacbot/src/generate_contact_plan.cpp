#include "contact_controller.h"
#include "contact_perception.h"
#include "contact_planner.h"
#include "panda_interface.h"
#include "utilities.h"

constexpr char LOGNAME[] = "generate_plan";

using namespace tacbot;

int main(int argc, char** argv) {
  ros::init(argc, argv, "generate_contact_plan");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  ROS_DEBUG_NAMED(LOGNAME, "Start!");

  std::shared_ptr<ContactPlanner> contact_planner =
      std::make_shared<ContactPlanner>();
  contact_planner->init();

  std::shared_ptr<Visualizer> visualizer = std::make_shared<Visualizer>();

  // visually confirm that the planning scene, obstacles, goal state are all
  // correct
  // utilities::promptAnyInput();

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  contact_planner->setCurToStartState(req);

  moveit_msgs::Constraints goal = contact_planner->createJointGoal();
  req.goal_constraints.push_back(goal);

  req.group_name = contact_planner->getGroupName();
  req.allowed_planning_time = 5.0;
  req.planner_id = contact_planner->getDefaultPlannerId();
  req.max_acceleration_scaling_factor = 0.5;
  req.max_velocity_scaling_factor = 0.5;

  contact_planner->createPlanningContext(req);

  const std::string PLANNER_NAME = "BITstar";
  const std::string OBJECTIVE_NAME = "UpstreamCost";
  const std::size_t OBSTACLE_SCENE_OPT = 1;
  const std::size_t GOAL_STATE_OPT = 1;
  contact_planner->setObstacleScene(OBSTACLE_SCENE_OPT);
  contact_planner->setGoalState(GOAL_STATE_OPT);

  visualizer->visualizeObstacleMarker(contact_planner->getSimObstaclePos());
  visualizer->setContactPlanner(contact_planner);
  visualizer->visualizeGoalState();

  // contact_planner->changePlanner(PLANNER_NAME, OBJECTIVE_NAME);
  contact_planner->generatePlan(res);

  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully. Error code: %d",
              res.error_code_.val);
    return 0;
  }

  // ROS_INFO_NAMED(LOGNAME, "Visualizing repulsed states.");
  // visualizer->visualizeRepulsedState();

  // ROS_INFO_NAMED(LOGNAME, "Visualizing all states in the tree.");
  // visualizer->visualizeTreeStates();
  // utilities::promptAnyInput();

  if (res.error_code_.val == res.error_code_.SUCCESS) {
    ROS_INFO_NAMED(LOGNAME, "Visualizing trajectory.");
    visualizer->visualizeTrajectory(res, "planned_path");
    utilities::promptAnyInput();
  }

  // if (res.error_code_.val == res.error_code_.SUCCESS) {
  //   ROS_INFO_NAMED(LOGNAME, "Checking for collisions on path.");
  //   contact_planner->runCollisionDetection();
  //   utilities::promptAnyInput();
  // }

  // if (res.error_code_.val == res.error_code_.SUCCESS) {
  //   ROS_INFO_NAMED(LOGNAME, "Executing trajectory.");
  //   contact_planner->executeTrajectory();
  //   contact_planner->monitorExecution();
  //   visualizer->visualizeObstacleMarker(contact_planner->getSimObstaclePos());

  //   utilities::promptAnyInput();
  // }

  PandaInterface panda_interface;
  panda_interface.initRobot();
  panda_interface.move_to_default_pose(panda_interface.robot_.get());
  franka::RobotState start_state = panda_interface.robot_.get()->readOnce();
  std::array<double, 16> position_d_s(start_state.O_T_EE);
  // Eigen::Quaterniond orientation_d(robot_state.rotation());

  std::vector<std::array<double, 7>> joint_waypoints;
  std::vector<std::array<double, 7>> joint_velocities;
  contact_planner->convertTraj(joint_waypoints, joint_velocities);

  // panda_interface.follow_joint_waypoints(panda_interface.robot_.get(),
  //                                        joint_waypoints, 0.3);
  panda_interface.follow_joint_velocities(panda_interface.robot_.get(),
                                          joint_velocities);

  franka::RobotState robot_state = panda_interface.robot_.get()->readOnce();
  Eigen::Affine3d initial_transform(
      Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  std::array<double, 7> robot_q = robot_state.q;
  Eigen::Vector3d position_d(initial_transform.translation());
  Eigen::Quaterniond orientation_d(initial_transform.rotation());
  std::cout << "joint positions" << std::endl;
  for (int i = 0; i < 7; i++) {
    std::cout << robot_q[i] << std::endl;
  }
  // std::array<double, 16> position_d(robot_state.O_T_EE);
  // Eigen::Quaterniond orientation_d(robot_state.rotation());
  // std::cout << "Start position:\n " << position_d_s[0] << " " <<
  // position_d_s[1]
  //           << " " << position_d_s[2] << " " << position_d_s[3] << "\n"
  //           << position_d_s[4] << " " << position_d_s[5] << " "
  //           << position_d_s[6] << " " << position_d_s[7] << "\n"
  //           << position_d_s[8] << " " << position_d_s[9] << " "
  //           << position_d_s[10] << " " << position_d_s[11] << "\n"
  //           << position_d_s[12] << " " << position_d_s[13] << " "
  //           << position_d_s[14] << " " << position_d_s[15] << "\n"
  //           << std::endl;
  // std::cout << "Final position:\n " << position_d[0] << " " << position_d[1]
  //           << " " << position_d[2] << " " << position_d[3] << "\n"
  //           << position_d[4] << " " << position_d[5] << " " << position_d[6]
  //           << " " << position_d[7] << "\n"
  //           << position_d[8] << " " << position_d[9] << " " << position_d[10]
  //           << " " << position_d[11] << "\n"
  //           << position_d[12] << " " << position_d[13] << " " <<
  //           position_d[14]
  //           << " " << position_d[15] << "\n"
  //           << std::endl;

  std::cout << "Finished!" << std::endl;

  return 0;
}
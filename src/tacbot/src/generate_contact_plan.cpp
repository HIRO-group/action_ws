#include "contact_planner.h"
#include "my_moveit_context.h"
#include "panda_interface.h"
#include "utilities.h"
#include "visualizer.h"

constexpr char LOGNAME[] = "generate_plan";

using namespace tacbot;

int main(int argc, char** argv) {
  ros::init(argc, argv, "generate_contact_plan");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  ROS_INFO_NAMED(LOGNAME, "Start!");

  std::shared_ptr<ContactPlanner> planner = std::make_shared<ContactPlanner>();
  ROS_INFO_NAMED(LOGNAME, "planner->init()");
  planner->init();

  ROS_INFO_NAMED(LOGNAME, "planner->getVisualizerData()");
  std::shared_ptr<Visualizer> visualizer =
      std::make_shared<Visualizer>(planner->getVisualizerData());

  ROS_INFO_NAMED(LOGNAME, "MyMoveitContext()");
  std::shared_ptr<MyMoveitContext> context = std::make_shared<MyMoveitContext>(
      planner->getPlanningSceneMonitor(), planner->getRobotModel());

  ROS_INFO_NAMED(LOGNAME, "setCurToStartState");
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  planner->setCurToStartState(req);

  ROS_INFO_NAMED(LOGNAME, "createJointGoal");
  moveit_msgs::Constraints goal = planner->createJointGoal();
  req.goal_constraints.push_back(goal);

  // utilities::promptUserInput();

  ROS_INFO_NAMED(LOGNAME, "visualizeGoalState");
  visualizer->visualizeGoalState(planner->getJointNames(),
                                 planner->getJointGoalPos());

  req.group_name = planner->getGroupName();
  req.allowed_planning_time = 60.0;
  req.planner_id = context->getPlannerId();
  req.max_acceleration_scaling_factor = 0.5;
  req.max_velocity_scaling_factor = 0.5;

  const std::string PLANNER_NAME = "ContactTRRTDuo";  // ContactTRRTDuo, RRTstar
  const std::string OBJECTIVE_NAME =
      "FieldAlign";  // FieldMagnitude or UpstreamCost or FieldAlign
  const std::size_t OBSTACLE_SCENE_OPT = 3;
  const std::size_t GOAL_STATE_OPT = 1;

  planner->setPlannerName(PLANNER_NAME);
  planner->setObjectiveName(OBJECTIVE_NAME);

  ROS_INFO_NAMED(LOGNAME, "setObstacleScene");
  planner->setObstacleScene(OBSTACLE_SCENE_OPT);
  ROS_INFO_NAMED(LOGNAME, "setGoalState");
  planner->setGoalState(GOAL_STATE_OPT);

  ROS_INFO_NAMED(LOGNAME, "visualizeObstacleMarker");
  visualizer->visualizeObstacleMarker(planner->getSimObstaclePos());

  utilities::promptUserInput();

  ROS_INFO_NAMED(LOGNAME, "createPlanningContext");
  context->createPlanningContext(req);

  ROS_INFO_NAMED(LOGNAME, "setPlanningContext");
  planner->setPlanningContext(context->getPlanningContext());

  ROS_INFO_NAMED(LOGNAME, "planner->changePlanner()");
  planner->changePlanner();

  ROS_INFO_NAMED(LOGNAME, "generatePlan");
  planner->generatePlan(res);

  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully. Error code: %d",
              res.error_code_.val);
    return 1;
  }
  moveit_msgs::MotionPlanResponse msg;
  res.getMessage(msg);
  visualizer->visualizeTrajectory(msg, "planned_path");
  visualizer->visualizeEEPath();

  utilities::promptUserInput();

  bool execute_trajectory = false;
  if (execute_trajectory == true) {
    PandaInterface panda_interface;
    panda_interface.init();
    panda_interface.move_to_default_pose(panda_interface.robot_.get());

    moveit_msgs::MotionPlanResponse traj_msg;
    res.getMessage(traj_msg);
    std::vector<std::array<double, 7>> joint_waypoints;
    std::vector<std::array<double, 7>> joint_velocities;
    utilities::toControlTrajectory(traj_msg, joint_waypoints, joint_velocities);

    panda_interface.follow_joint_velocities(panda_interface.robot_.get(),
                                            joint_velocities);
  }

  std::cout << "Finished!" << std::endl;

  return 0;
}

// #include "contact_perception.h"
// #include "planner.h"
// #include "panda_interface.h"
// #include "utilities.h"

// constexpr char LOGNAME[] = "generate_plan";

// using namespace tacbot;

// int main(int argc, char** argv) {
//   ros::init(argc, argv, "generate_contact_plan");
//   ros::AsyncSpinner spinner(1);
//   spinner.start();
//   ros::NodeHandle node_handle;

//   ROS_DEBUG_NAMED(LOGNAME, "Start!");

//   std::shared_ptr<ContactPlanner> planner =
//       std::make_shared<ContactPlanner>();
//   planner->init();
//   // utilities::promptUserInput();

//   std::shared_ptr<Visualizer> visualizer = std::make_shared<Visualizer>();

//   // visually confirm that the planning scene, obstacles, goal state are all
//   // correct
//   // Eigen::Vector3d vec(0, 0, 0);
//   // visualizer->visualizeObstacleMarker(planner->getObstacles(vec));
//   // utilities::promptUserInput();

//   planning_interface::MotionPlanRequest req;
//   planning_interface::MotionPlanResponse res;
//   planner->setCurToStartState(req);

//   moveit_msgs::Constraints goal = planner->createJointGoal();
//   req.goal_constraints.push_back(goal);

//   req.group_name = planner->getGroupName();
//   req.allowed_planning_time = 30.0;
//   req.planner_id = planner->getPlannerId();
//   req.max_acceleration_scaling_factor = 0.5;
//   req.max_velocity_scaling_factor = 0.5;

//   // planner->createPlanningContext(req);

//   const std::string PLANNER_NAME = "ContactTRRTDuo";  // ContactTRRTDuo
//   const std::string OBJECTIVE_NAME =
//       "FieldAlign";  // FieldMagnitude or UpstreamCost or FieldAlign
//   const std::size_t OBSTACLE_SCENE_OPT = 4;
//   const std::size_t GOAL_STATE_OPT = 1;
//   planner->setObstacleScene(OBSTACLE_SCENE_OPT);
//   planner->setGoalState(GOAL_STATE_OPT);

//   // Eigen::Vector3d origin(0, 0, 0);
//   //
//   visualizer->visualizeObstacleMarker(planner->getObstacles(origin));
//   //
//   visualizer->visualizeObstacleMarker(planner->getSimObstaclePos());
//   // visualizer->setContactPlanner(planner);
//   // visualizer->visualizeGoalState();

//   planner->changePlanner(PLANNER_NAME, OBJECTIVE_NAME);
//   planner->generatePlan(res);

//   if (res.error_code_.val != res.error_code_.SUCCESS) {
//     ROS_ERROR("Could not compute plan successfully. Error code: %d",
//               res.error_code_.val);
//   }

//   BenchMarkData benchmark_data;
//   planner->analyzePlanResponse(benchmark_data);

//   // ROS_INFO_NAMED(LOGNAME, "Visualizing repulsed states.");
//   // visualizer->visualizeRepulsedState();

//   // ROS_INFO_NAMED(LOGNAME, "Visualizing all states in the tree.");
//   // visualizer->visualizeTreeStates();
//   // utilities::promptUserInput();

//   // if (res.error_code_.val == res.error_code_.SUCCESS) {
//   //   ROS_INFO_NAMED(LOGNAME, "Visualizing trajectory.");
//   //   std::cout << "num pts: "
//   //             << planner->fast_plan_response_.trajectory
//   //                    .joint_trajectory.points.size()
//   //             << std::endl;
//   //   visualizer->visualizeTrajectory(planner->fast_plan_response_,
//   //                                   "planned_path");
//   //   visualizer->visualizeEEPath();
//   //   utilities::promptUserInput();
//   // }

//   // if (res.error_code_.val == res.error_code_.SUCCESS) {
//   //   ROS_INFO_NAMED(LOGNAME, "Checking for collisions on path.");
//   //   planner->runCollisionDetection();
//   //   utilities::promptUserInput();
//   // }

//   // if (res.error_code_.val == res.error_code_.SUCCESS) {
//   //   ROS_INFO_NAMED(LOGNAME, "Executing trajectory.");
//   //   planner->executeTrajectory();
//   //   planner->monitorExecution();
//   //
//   visualizer->visualizeObstacleMarker(planner->getSimObstaclePos());

//   //   utilities::promptUserInput();
//   // }

//   return 0;

//   PandaInterface panda_interface;
//   panda_interface.init();
//   panda_interface.move_to_default_pose(panda_interface.robot_.get());

//   moveit_msgs::MotionPlanResponse traj_msg;
//   res.getMessage(traj_msg);
//   std::vector<std::array<double, 7>> joint_waypoints;
//   std::vector<std::array<double, 7>> joint_velocities;
//   utilities::toControlTrajectory(traj_msg, joint_waypoints,
//   joint_velocities);

//   panda_interface.follow_joint_velocities(panda_interface.robot_.get(),
//                                           joint_velocities);

//   std::cout << "Finished!" << std::endl;

//   return 0;
// }
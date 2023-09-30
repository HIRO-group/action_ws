#include "contact_planner.h"
#include "my_moveit_context.h"
#include "panda_interface.h"
#include "utilities.h"
#include "visualizer.h"

constexpr char LOGNAME[] = "generate_plan";

using namespace tacbot;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "generate_contact_plan");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  ROS_INFO_NAMED(LOGNAME, "Start!");

  std::shared_ptr<ContactPlanner> planner = std::make_shared<ContactPlanner>();
  ROS_INFO_NAMED(LOGNAME, "planner->init()");
  planner->init();

  std::string PLANNER_NAME = "ContactTRRTDuo";  // ContactTRRTDuo, RRTstar
  std::string OBJECTIVE_NAME =
      "FieldAlign";  // FieldMagnitude or UpstreamCost or FieldAlign

  const std::size_t OBSTACLE_SCENE_OPT = 4;
  const std::size_t GOAL_STATE_OPT = 1;

  planner->setPlannerName(PLANNER_NAME);
  planner->setObjectiveName(OBJECTIVE_NAME);

  ROS_INFO_NAMED(LOGNAME, "setObstacleScene");
  planner->setObstacleScene(OBSTACLE_SCENE_OPT);
  ROS_INFO_NAMED(LOGNAME, "setGoalState");
  planner->setGoalState(GOAL_STATE_OPT);

  ROS_INFO_NAMED(LOGNAME, "contact_perception_->init()");
  planner->contact_perception_->init();

  ROS_INFO_NAMED(LOGNAME, "planner->getVisualizerData()");
  std::shared_ptr<Visualizer> visualizer =
      std::make_shared<Visualizer>(planner->getVisualizerData());

  ROS_INFO_NAMED(LOGNAME, "visualizeObstacleMarker");
  visualizer->visualizeObstacleMarker(planner->getSimObstaclePos());

  ROS_INFO_NAMED(LOGNAME, "MyMoveitContext()");
  std::shared_ptr<MyMoveitContext> context = std::make_shared<MyMoveitContext>(
      planner->getPlanningSceneMonitor(), planner->getRobotModel());
  context->setSimplifySolution(false);

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

  std::string planner_param;
  if (node_handle.getParam("demo_planner", planner_param)) {
    ROS_INFO("Got param: %s", planner_param.c_str());
  } else {
    ROS_ERROR("Failed to get param 'demo_planner'");
  }

  if (planner_param == "contact") {
    PLANNER_NAME = "ContactTRRTDuo";  // ContactTRRTDuo, RRTstar
    OBJECTIVE_NAME =
        "FieldAlign";  // FieldMagnitude or UpstreamCost or FieldAlign
  }

  // bool status = utilities::promptUserInput();
  // if (!status) {
  //   return 0;
  // }

  // Eigen::Vector3d start{0, 0, 0};
  // visualizer->visualizeObstacleMarker(planner->getObstacles(start));

  // status = utilities::promptUserInput();
  // if (!status) {
  //   return 0;
  // }

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

  if (!planner->parameterizePlan(res)) {
    return 1;
  }

  moveit_msgs::MotionPlanResponse msg;
  res.getMessage(msg);
  visualizer->visualizeTrajectory(msg, "planned_path");
  // visualizer->visualizeEEPath();

  bool execute = utilities::promptUserInput();
  if (!execute) {
    return 0;
  }

  bool execute_trajectory = true;
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
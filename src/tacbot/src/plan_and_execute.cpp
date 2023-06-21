#include "base_planner.h"
#include "my_moveit_context.h"
#include "panda_interface.h"
#include "perception_planner.h"
#include "utilities.h"
#include "visualizer.h"

constexpr char LOGNAME[] = "plan_and_execute";

using namespace tacbot;

int main(int argc, char** argv) {
  ros::init(argc, argv, "plan_and_execute");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  ROS_DEBUG_NAMED(LOGNAME, "Start!");

  std::shared_ptr<PerceptionPlanner> planner =
      std::make_shared<PerceptionPlanner>();
  ROS_DEBUG_NAMED(LOGNAME, "planner->init()");
  planner->init();

  ROS_DEBUG_NAMED(LOGNAME, "planner->getVisualizerData()");
  std::shared_ptr<Visualizer> visualizer =
      std::make_shared<Visualizer>(planner->getVisualizerData());

  ROS_DEBUG_NAMED(LOGNAME, "MyMoveitContext()");
  std::shared_ptr<MyMoveitContext> context = std::make_shared<MyMoveitContext>(
      planner->getPlanningSceneMonitor(), planner->getRobotModel());
  context->setSimplifySolution(true);

  ROS_DEBUG_NAMED(LOGNAME, "setCurToStartState");
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  planner->setCurToStartState(req);

  ROS_DEBUG_NAMED(LOGNAME, "createJointGoal");
  moveit_msgs::Constraints goal = planner->createJointGoal();
  req.goal_constraints.push_back(goal);

  ROS_DEBUG_NAMED(LOGNAME, "visualizeGoalState");
  visualizer->visualizeGoalState(planner->getJointNames(),
                                 planner->getJointGoalPos());

  ROS_DEBUG_NAMED(LOGNAME, "visualizeObstacleMarker");
  visualizer->visualizeObstacleMarker(planner->getObstacles());

  bool status = utilities::promptUserInput();
  if (!status) {
    return 0;
  }

  req.group_name = planner->getGroupName();
  req.allowed_planning_time = 10.0;
  req.planner_id = context->getPlannerId();
  req.max_acceleration_scaling_factor = 0.1;
  req.max_velocity_scaling_factor = 0.1;

  ROS_DEBUG_NAMED(LOGNAME, "createPlanningContext");
  context->createPlanningContext(req);

  ROS_DEBUG_NAMED(LOGNAME, "setPlanningContext");
  planner->setPlanningContext(context->getPlanningContext());

  ROS_DEBUG_NAMED(LOGNAME, "planner->changePlanner()");
  const std::string PLANNER_NAME = "BITstar";  //"BITstar"
  planner->setPlannerName(PLANNER_NAME);
  planner->changePlanner();

  ROS_DEBUG_NAMED(LOGNAME, "generatePlan");
  planner->generatePlan(res);

  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully. Error code: %d",
              res.error_code_.val);
    return 1;
  }

  moveit_msgs::MotionPlanResponse msg;
  res.getMessage(msg);
  visualizer->visualizeTrajectory(msg, "planned_path");
  visualizer->visualizeTrajectory(
      planner->getPlanningContext()->getRawTrajectory(), "raw_traj");

  ROS_INFO_NAMED(LOGNAME, "Press continue to exexute trajectory.");
  bool execute_trajectory = utilities::promptUserInput();
  if (!status) {
    return 0;
  }

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
    sleep(0.5);
  }

  std::cout << "Finished!" << std::endl;

  return 0;
}
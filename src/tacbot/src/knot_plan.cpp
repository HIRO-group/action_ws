#include "base_planner.h"
#include "my_moveit_context.h"
#include "panda_interface.h"
#include "perception_planner.h"
#include "utilities.h"
#include "visualizer.h"

constexpr char LOGNAME[] = "knot_plan";

using namespace tacbot;

int main(int argc, char **argv) {
  ros::init(argc, argv, "knot_plan");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  ROS_DEBUG_NAMED(LOGNAME, "Start!");

  OptimizationProblem problem;
  problem.optimize();

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

  std::vector<double> start{
      0, -0.785398163, 0, -2.35619449, 0, 1.57079632679, 0.785398163397};
  std::vector<double> wp1{-1.80753,  -1.22744, 1.94936,  -1.356,
                          -0.440299, 2.34236,  -0.313631};
  std::vector<double> wp2{-2.02094,  -1.10387, 1.9391,   -1.61801,
                          -0.440085, 1.69906,  -0.295163};
  std::vector<double> goal{-2.02408,   -1.06383, 1.8716,    -1.80128,
                           0.00569006, 0.713265, -0.0827766};

  std::vector<std::vector<double>> knots;
  knots.emplace_back(start);
  knots.emplace_back(wp1);
  knots.emplace_back(wp2);
  knots.emplace_back(goal);

  std::size_t num_jnts =
      planner->getJointModelGroup()->getActiveVariableCount();

  moveit_msgs::Constraints cons1;
  cons1.joint_constraints.resize(num_jnts);
  for (std::size_t i = 0; i < num_jnts; ++i) {
    cons1.joint_constraints[i].joint_name =
        planner->getJointModelGroup()->getVariableNames()[i];
    cons1.joint_constraints[i].position = knots[1].at(i);
    cons1.joint_constraints[i].tolerance_above = 0.001;
    cons1.joint_constraints[i].tolerance_below = 0.001;
    cons1.joint_constraints[i].weight = 1.0;
  }

  moveit_msgs::Constraints cons2;
  cons2.joint_constraints.resize(num_jnts);
  for (std::size_t i = 0; i < num_jnts; ++i) {
    cons2.joint_constraints[i].joint_name =
        planner->getJointModelGroup()->getVariableNames()[i];
    cons2.joint_constraints[i].position = knots[2].at(i);
    cons2.joint_constraints[i].tolerance_above = 0.001;
    cons2.joint_constraints[i].tolerance_below = 0.001;
    cons2.joint_constraints[i].weight = 1.0;
  }

  moveit_msgs::Constraints cons3;
  cons3.joint_constraints.resize(num_jnts);
  for (std::size_t i = 0; i < num_jnts; ++i) {
    cons3.joint_constraints[i].joint_name =
        planner->getJointModelGroup()->getVariableNames()[i];
    cons3.joint_constraints[i].position = knots[3].at(i);
    cons3.joint_constraints[i].tolerance_above = 0.001;
    cons3.joint_constraints[i].tolerance_below = 0.001;
    cons3.joint_constraints[i].weight = 1.0;
  }

  std::vector<moveit_msgs::Constraints> constraints;
  constraints.emplace_back(cons1);
  constraints.emplace_back(cons2);
  constraints.emplace_back(cons3);

  std::vector<planning_interface::MotionPlanResponse> responses;
  for (std::size_t i = 0; i < constraints.size(); i++) {
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    ROS_DEBUG_NAMED(LOGNAME, "setStartState");
    planner->setStartState(req, knots[i]);

    ROS_DEBUG_NAMED(LOGNAME, "req.goal_constraints");
    req.goal_constraints.push_back(constraints[i]);

    req.group_name = planner->getGroupName();
    req.allowed_planning_time = 5.0;
    req.planner_id = context->getPlannerId();
    req.max_acceleration_scaling_factor = 0.5;
    req.max_velocity_scaling_factor = 0.5;

    ROS_DEBUG_NAMED(LOGNAME, "visualizeGoalState");
    visualizer->visualizeGoalState(planner->getJointNames(),
                                   planner->getJointGoalPos());

    ROS_DEBUG_NAMED(LOGNAME, "visualizeObstacleMarker");
    visualizer->visualizeObstacleMarker(planner->getObstacles());

    ROS_DEBUG_NAMED(LOGNAME, "createPlanningContext");
    context->createPlanningContext(req);

    ROS_DEBUG_NAMED(LOGNAME, "setPlanningContext");
    planner->setPlanningContext(context->getPlanningContext());

    ROS_DEBUG_NAMED(LOGNAME, "planner->changePlanner()");
    const std::string PLANNER_NAME = "BITstar";  //"BITstar, QRRTStar"
    planner->setPlannerName(PLANNER_NAME);
    planner->changePlanner();

    ROS_DEBUG_NAMED(LOGNAME, "generatePlan");
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
    visualizer->visualizeTrajectory(
        planner->getPlanningContext()->getRawTrajectory(), "raw_traj");

    responses.emplace_back(res);

    // bool status = utilities::promptUserInput();
    // if (!status) {
    //   return 0;
    // }
  }

  ROS_INFO_NAMED(LOGNAME, "Press continue to execute trajectory.");
  bool execute = utilities::promptUserInput();
  if (!execute) {
    return 0;
  }

  PandaInterface panda_interface;
  panda_interface.init();
  panda_interface.move_to_default_pose(panda_interface.robot_.get());

  sleep(0.1);

  for (std::size_t i = 0; i < responses.size(); i++) {
    planning_interface::MotionPlanResponse res = responses[i];
    moveit_msgs::MotionPlanResponse traj_msg;
    res.getMessage(traj_msg);
    std::vector<std::array<double, 7>> joint_waypoints;
    std::vector<std::array<double, 7>> joint_velocities;
    utilities::toControlTrajectory(traj_msg, joint_waypoints, joint_velocities);

    panda_interface.follow_joint_velocities(panda_interface.robot_.get(),
                                            joint_velocities);

    sleep(0.1);

    // bool status = utilities::promptUserInput();
    // if (!status) {
    //   return 0;
    // }
  }

  std::cout << "Finished!" << std::endl;

  return 0;
}
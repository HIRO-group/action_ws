

// local

#include "base_planner.h"
#include "my_moveit_context.h"
#include "panda_interface.h"
#include "perception_planner.h"
#include "utilities.h"
#include "visualizer.h"

using namespace tacbot;
constexpr char LOGNAME[] = "knot_plan";

int main(int argc, char** argv) {
  ros::init(argc, argv, "knot_plan");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  ROS_DEBUG_NAMED(LOGNAME, "Start!");

  std::shared_ptr<PerceptionPlanner> planner =
      std::make_shared<PerceptionPlanner>();
  ROS_DEBUG_NAMED(LOGNAME, "planner->init()");
  planner->init();
  planner->setObstacleScene(0);

  std::vector<double> pos{-0.00015798826144131084, -0.7855155831866529,
                          4.077083616493837e-05,   -2.356070629900656,
                          3.513825316048269e-05,   1.5712720386575345,
                          0.7853804904391213};
  planner->solveFK(pos);

  ROS_DEBUG_NAMED(LOGNAME, "planner->getVisualizerData()");
  std::shared_ptr<Visualizer> visualizer =
      std::make_shared<Visualizer>(planner->getVisualizerData());

  ROS_DEBUG_NAMED(LOGNAME, "MyMoveitContext()");
  std::shared_ptr<MyMoveitContext> context = std::make_shared<MyMoveitContext>(
      planner->getPlanningSceneMonitor(), planner->getRobotModel());
  context->setSimplifySolution(true);

  std::vector<geometry_msgs::Point> waypoint_grid;

  double start_x = 0.3;
  double start_y = 0.0;
  double start_z = 0.244;

  double goal_x = 0.4;
  double goal_y = 0.2;

  double x_offset = 0.05;
  double y_offset = 0.05;
  double z_offset = 0.05;

  double num_x_pts = 2;
  double num_y_pts = 2;

  for (double x = start_x; x <= goal_x; x += x_offset) {
    for (double y = start_y; y <= goal_y; y += y_offset) {
      geometry_msgs::Point pt1;
      pt1.x = x;
      pt1.y = y;
      pt1.z = start_z + z_offset;
      waypoint_grid.emplace_back(pt1);

      geometry_msgs::Point pt2 = pt1;
      pt2.z = pt1.z - z_offset;
      waypoint_grid.emplace_back(pt2);

      waypoint_grid.emplace_back(pt1);
    }
  }

  visualizer->visualizePoints(waypoint_grid);
  bool status = utilities::promptUserInput();
  if (!status) {
    return 0;
  }

  std::size_t num_waypoints = waypoint_grid.size();
  ROS_DEBUG_NAMED(LOGNAME, "num_waypoints: %ld", num_waypoints);
  std::vector<std::vector<double>> knots;
  std::vector<double> start{
      0, -0.785398163, 0, -2.35619449, 0, 1.57079632679, 0.785398163397};
  knots.emplace_back(start);
  visualizer->visualizeGoalState(planner->getJointNames(), knots[0]);
  for (std::size_t i = 0; i < num_waypoints; i++) {
    geometry_msgs::Point pt = waypoint_grid[i];
    geometry_msgs::Pose ik_pose;

    ik_pose.position.x = pt.x;
    ik_pose.position.y = pt.y;
    ik_pose.position.z = pt.z;
    ik_pose.orientation.x = 0.92388;
    ik_pose.orientation.y = -0.382683;
    ik_pose.orientation.z = 0.0;
    ik_pose.orientation.w = 0.0;

    std::vector<double> state;
    if (!planner->solveIK(ik_pose, start, state)) {
      return 0;
    }
    knots.emplace_back(state);
  }

  std::vector<moveit_msgs::Constraints> constraints;
  std::size_t num_knots = knots.size();
  for (std::size_t i = 1; i < num_knots; i++) {
    moveit_msgs::Constraints goal = planner->createJointGoal(knots[i]);
    constraints.emplace_back(goal);
  }

  std::vector<planning_interface::MotionPlanResponse> responses;
  for (std::size_t i = 0; i < constraints.size(); i++) {
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    ROS_DEBUG_NAMED(LOGNAME, "setStartState");
    std::vector<double> start_state = knots[i];
    planner->setStartState(req, start_state);

    ROS_DEBUG_NAMED(LOGNAME, "req.goal_constraints");
    req.goal_constraints.push_back(constraints[i]);

    req.group_name = planner->getGroupName();
    req.allowed_planning_time = 5.0;
    req.planner_id = context->getPlannerId();
    req.max_acceleration_scaling_factor = 0.5;
    req.max_velocity_scaling_factor = 0.5;

    ROS_DEBUG_NAMED(LOGNAME, "visualizeGoalState");
    visualizer->visualizeGoalState(planner->getJointNames(), knots[i]);

    ROS_DEBUG_NAMED(LOGNAME, "visualizeObstacleMarker");
    visualizer->visualizeObstacleMarker(planner->getObstacles());

    ROS_DEBUG_NAMED(LOGNAME, "createPlanningContext");
    context->createPlanningContext(req);

    ROS_DEBUG_NAMED(LOGNAME, "setPlanningContext");
    planner->setPlanningContext(context->getPlanningContext());

    ROS_DEBUG_NAMED(LOGNAME, "planner->changePlanner()");
    const std::string PLANNER_NAME =
        "RRTConnect";  //"BITstar, QRRTStar, RRTConnect"
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

    context->getPlanningContext()->clear();
    // status = utilities::promptUserInput();
    // if (!status) {
    //   return 0;
    // }
  }

  ROS_DEBUG_NAMED(LOGNAME, "Continue to trajectory execution.");
  bool execute = utilities::promptUserInput();
  if (!execute) {
    return 0;
  }

  PandaInterface panda_interface;
  panda_interface.init();
  panda_interface.move_to_default_pose(panda_interface.robot_.get());

  sleep(1.1);

  std::size_t counter = 1;

  for (std::size_t i = 0; i < responses.size(); i++) {
    planning_interface::MotionPlanResponse res = responses[i];
    moveit_msgs::MotionPlanResponse traj_msg;
    res.getMessage(traj_msg);
    std::vector<std::array<double, 7>> joint_waypoints;
    std::vector<std::array<double, 7>> joint_velocities;
    utilities::toControlTrajectory(traj_msg, joint_waypoints, joint_velocities);

    panda_interface.follow_joint_velocities(panda_interface.robot_.get(),
                                            joint_velocities);

    if (i == counter) {
      sleep(3.0);
      counter += 3;
    }

    sleep(0.1);

    // bool status = utilities::promptUserInput();
    // if (!status) {
    //   return 0;
    // }
  }

  std::cout << "Finished!" << std::endl;

  return 0;
}
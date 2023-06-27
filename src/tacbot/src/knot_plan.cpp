#include "base_planner.h"
#include "my_moveit_context.h"
#include "panda_interface.h"
#include "perception_planner.h"
// #include "pilz_industrial_motion_planner/cartesian_limits_aggregator.h"
// #include "pilz_industrial_motion_planner/joint_limits_aggregator.h"
// #include "pilz_industrial_motion_planner/tip_frame_getter.h"
// #include "pilz_industrial_motion_planner/trajectory_blend_request.h"
// #include
// "pilz_industrial_motion_planner/trajectory_blender_transition_window.h"
#include "utilities.h"
#include "visualizer.h"

constexpr char LOGNAME[] = "knot_plan";

using namespace tacbot;

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

  ROS_DEBUG_NAMED(LOGNAME, "planner->getVisualizerData()");
  std::shared_ptr<Visualizer> visualizer =
      std::make_shared<Visualizer>(planner->getVisualizerData());

  ROS_DEBUG_NAMED(LOGNAME, "MyMoveitContext()");
  std::shared_ptr<MyMoveitContext> context = std::make_shared<MyMoveitContext>(
      planner->getPlanningSceneMonitor(), planner->getRobotModel());
  context->setSimplifySolution(false);

  std::vector<double> start{
      0, -0.785398163, 0, -2.35619449, 0, 1.57079632679, 0.785398163397};
  std::vector<double> wp1{-1.0304, -1.04185, 1.0716, -1.72948,
                          0.66649, 0.724355, 1.38011};
  std::vector<double> wp2{-1.5304, -1.44185, 1.4716, -1.72948,
                          0.66649, 0.724355, 1.38011};
  std::vector<double> goal{-2.0304, -1.44185, 1.4716, -1.72948,
                           0.66649, 0.724355, 1.38011};

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
    req.allowed_planning_time = 10.0;
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

    moveit_msgs::MotionPlanResponse msg;
    res.getMessage(msg);
    visualizer->visualizeTrajectory(msg, "planned_path");
    visualizer->visualizeTrajectory(
        planner->getPlanningContext()->getRawTrajectory(), "raw_traj");

    responses.emplace_back(res);

    bool status = utilities::promptUserInput();
    if (!status) {
      return 0;
    }
  }

  // pilz_industrial_motion_planner::JointLimitsContainer
  //     aggregated_limit_active_joints;

  // aggregated_limit_active_joints = pilz_industrial_motion_planner::
  //     JointLimitsAggregator::getAggregatedLimits(
  //         node_handle, planner->getRobotModel()->getActiveJointModels());

  // // Obtain cartesian limits
  // pilz_industrial_motion_planner::CartesianLimit cartesian_limit =
  //     pilz_industrial_motion_planner::CartesianLimitsAggregator::
  //         getAggregatedLimits(node_handle);

  // pilz_industrial_motion_planner::LimitsContainer limits;
  // limits.setJointLimits(aggregated_limit_active_joints);
  // limits.setCartesianLimits(cartesian_limit);

  // PlanComponentsBuilder plan_comp_builder_;
  // plan_comp_builder_.setModel(planner->getRobotModel());
  // plan_comp_builder_.setBlender(
  //     std::unique_ptr<pilz_industrial_motion_planner::TrajectoryBlender>(
  //         new
  //         pilz_industrial_motion_planner::TrajectoryBlenderTransitionWindow(
  //             limits)));

  ROS_INFO_NAMED(LOGNAME, "Press continue to execute trajectory.");
  bool execute = utilities::promptUserInput();
  if (!execute) {
    return 0;
  }

  PandaInterface panda_interface;
  panda_interface.init();
  panda_interface.move_to_default_pose(panda_interface.robot_.get());

  for (std::size_t i = 0; i < responses.size(); i++) {
    planning_interface::MotionPlanResponse res = responses[i];
    moveit_msgs::MotionPlanResponse traj_msg;
    res.getMessage(traj_msg);
    std::vector<std::array<double, 7>> joint_waypoints;
    std::vector<std::array<double, 7>> joint_velocities;
    utilities::toControlTrajectory(traj_msg, joint_waypoints, joint_velocities);

    panda_interface.follow_joint_velocities(panda_interface.robot_.get(),
                                            joint_velocities);

    bool status = utilities::promptUserInput();
    if (!status) {
      return 0;
    }
  }

  std::cout << "Finished!" << std::endl;

  return 0;
}
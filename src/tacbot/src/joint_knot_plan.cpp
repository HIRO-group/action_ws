

// local
#include <fstream>
#include <nlohmann/json.hpp>

#include "base_planner.h"
#include "my_moveit_context.h"
#include "panda_interface.h"
#include "perception_planner.h"
#include "utilities.h"
#include "visualizer.h"

using json = nlohmann::json;

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
  context->setSimplifySolution(true);

  std::string package_path = ros::package::getPath("tacbot");
  // std::cout << "package_path " << package_path << std::endl;

  // Specify a relative file path within the package
  std::string relative_path = package_path + "/bags/states.json";

  // Open the JSON file for reading
  std::ifstream input_file(relative_path);

  // Check if the file is open
  if (!input_file.is_open()) {
    std::cerr << "Failed to open the JSON file." << std::endl;
    return 1;
  }

  // Parse the JSON data from the file
  json json_data;
  input_file >> json_data;

  // Close the file
  input_file.close();

  // Check if the "joint_states" key exists
  if (json_data.find("joint_states") == json_data.end()) {
    std::cerr << "The 'joint_states' key was not found in the JSON file."
              << std::endl;
    return 1;
  }

  std::vector<std::vector<double>> knots;
  // Extract and process the arrays of 7-DOF joint states
  for (const auto& joint_state : json_data["joint_states"]) {
    std::vector<double> joint_states_7dof =
        joint_state.get<std::vector<double>>();

    knots.emplace_back(joint_states_7dof);

    // Print the 7-DOF joint states
    std::cout << "7-DOF Joint States: ";
    for (const double& value : joint_states_7dof) {
      std::cout << value << " ";
    }
    std::cout << std::endl;
  }

  std::size_t num_jnts =
      planner->getJointModelGroup()->getActiveVariableCount();

  std::vector<moveit_msgs::Constraints> constraints;
  for (std::size_t i = 1; i < knots.size(); i++) {
    moveit_msgs::Constraints goal_constraint;
    goal_constraint.joint_constraints.resize(num_jnts);
    for (std::size_t j = 0; j < num_jnts; ++j) {
      goal_constraint.joint_constraints[j].joint_name =
          planner->getJointModelGroup()->getVariableNames()[j];
      goal_constraint.joint_constraints[j].position = knots[i].at(j);
      goal_constraint.joint_constraints[j].tolerance_above = 0.001;
      goal_constraint.joint_constraints[j].tolerance_below = 0.001;
      goal_constraint.joint_constraints[j].weight = 1.0;
    }
    constraints.emplace_back(goal_constraint);
  }

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

    context->getPlanningContext()->clear();
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
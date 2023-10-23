

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

void writeJointDataToJson(
    const std::vector<std::array<double, 7>>& jointPoints,
    const std::vector<std::array<double, 7>>& jointVelocities,
    const std::string& filename) {
  json data;  // Create a JSON object

  // Convert and add joint points to the JSON object
  json points;
  for (const auto& point : jointPoints) {
    json pointArray;
    for (const auto& value : point) {
      pointArray.push_back(value);
    }
    points.push_back(pointArray);
  }
  data["joint_states"] = points;

  // Add joint velocities to the JSON object
  json velocities;
  for (const auto& point : jointVelocities) {
    json pointArray;
    for (const auto& value : point) {
      pointArray.push_back(value);
    }
    velocities.push_back(pointArray);
  }

  data["joint_velocities"] = velocities;

  std::string package_path = ros::package::getPath("tacbot");
  // std::cout << "package_path " << package_path << std::endl;

  // Specify a relative file path within the package
  std::string relative_path = package_path + "/bags/" + filename;

  // Write the JSON data to a file
  std::ofstream file(relative_path);
  file << std::setw(4) << data << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "knot_plan");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  ROS_INFO_NAMED(LOGNAME, "Start!");

  std::shared_ptr<PerceptionPlanner> planner =
      std::make_shared<PerceptionPlanner>();
  ROS_INFO_NAMED(LOGNAME, "planner->init()");
  planner->init();
  planner->setObstacleScene(0);

  ROS_INFO_NAMED(LOGNAME, "planner->getVisualizerData()");
  std::shared_ptr<Visualizer> visualizer =
      std::make_shared<Visualizer>(planner->getVisualizerData());

  ROS_INFO_NAMED(LOGNAME, "MyMoveitContext()");
  std::shared_ptr<MyMoveitContext> context = std::make_shared<MyMoveitContext>(
      planner->getPlanningSceneMonitor(), planner->getRobotModel());
  context->setSimplifySolution(true);

  std::string package_path = ros::package::getPath("tacbot");
  // std::cout << "package_path " << package_path << std::endl;

  // Specify a relative file path within the package
  std::string relative_path = package_path + "/bags/joint_states.json";

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

  std::cout << "knots.size() " << knots.size() << std::endl;
  std::cout << "constraints.size() " << constraints.size() << std::endl;

  std::vector<Eigen::Vector3d> ee_path_pts;
  std::vector<planning_interface::MotionPlanResponse> responses;
  for (std::size_t i = 0; i < constraints.size(); i++) {
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    ROS_INFO_NAMED(LOGNAME, "setStartState");
    planner->setStartState(req, knots[i]);

    ROS_INFO_NAMED(LOGNAME, "req.goal_constraints");
    req.goal_constraints.push_back(constraints[i]);

    req.group_name = planner->getGroupName();
    req.allowed_planning_time = 5.0;
    req.planner_id = context->getPlannerId();
    req.max_acceleration_scaling_factor = 0.5;
    req.max_velocity_scaling_factor = 0.5;

    ROS_INFO_NAMED(LOGNAME, "visualizeGoalState");
    visualizer->visualizeGoalState(planner->getJointNames(), knots[i]);

    ROS_INFO_NAMED(LOGNAME, "visualizeObstacleMarker");
    visualizer->visualizeObstacleMarker(planner->getObstacles());

    ROS_INFO_NAMED(LOGNAME, "createPlanningContext");
    context->createPlanningContext(req);

    ROS_INFO_NAMED(LOGNAME, "setPlanningContext");
    planner->setPlanningContext(context->getPlanningContext());

    ROS_INFO_NAMED(LOGNAME, "planner->changePlanner()");
    const std::string PLANNER_NAME = "RRTConnect";  //"BITstar, QRRTStar"
    planner->setPlannerName(PLANNER_NAME);
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

    ROS_INFO_NAMED(LOGNAME, "Calculate end-effector path.");
    planner->calculateEEPath();
    std::vector<Eigen::Vector3d> ee_pts =
        planner->getVisualizerData()->ee_path_pts_;
    ee_path_pts.insert(ee_path_pts.end(), ee_pts.begin(), ee_pts.end());
    ROS_INFO_NAMED(LOGNAME, "Visualize end-effector path.");
    visualizer->visualizeEEPath();

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

  ROS_INFO_NAMED(LOGNAME, "Visualize full end-effector path.");
  planner->getVisualizerData()->ee_path_pts_ = ee_path_pts;
  visualizer->visualizeEEPath();

  ROS_INFO_NAMED(LOGNAME, "Press continue to execute trajectory.");
  bool execute = utilities::promptUserInput();
  if (!execute) {
    return 0;
  }

  // PandaInterface panda_interface;
  // panda_interface.init();
  // panda_interface.move_to_default_pose(panda_interface.robot_.get());

  sleep(0.1);

  std::vector<std::array<double, 7>> traj_waypoints;
  std::vector<std::array<double, 7>> traj_velocities;

  for (std::size_t i = 0; i < responses.size(); i++) {
    planning_interface::MotionPlanResponse res = responses[i];
    moveit_msgs::MotionPlanResponse traj_msg;
    res.getMessage(traj_msg);
    std::vector<std::array<double, 7>> joint_waypoints;
    std::vector<std::array<double, 7>> joint_velocities;
    utilities::toControlTrajectory(traj_msg, joint_waypoints, joint_velocities);

    traj_waypoints.insert(traj_waypoints.end(), joint_waypoints.begin(),
                          joint_waypoints.end());

    traj_velocities.insert(traj_velocities.end(), joint_velocities.begin(),
                           joint_velocities.end());

    // panda_interface.follow_joint_velocities(panda_interface.robot_.get(),
    //                                         joint_velocities);

    sleep(0.1);

    // bool status = utilities::promptUserInput();
    // if (!status) {
    //   return 0;
    // }
  }

  writeJointDataToJson(traj_waypoints, traj_velocities, "trajectory.json");

  std::cout << "Finished!" << std::endl;

  return 0;
}

#include <ros/ros.h>
// MoveIt
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/VFRRT.h>

#include "ompl/geometric/planners/est/EST.h"

constexpr char LOGNAME[] = "pick_and_place_node";

void printPlannerConfigMap(
    const planning_interface::PlannerConfigurationMap& planner_config_map) {
  for (auto config : planner_config_map) {
    ROS_INFO_NAMED(LOGNAME, "Map Name: %s", config.first.c_str());
    ROS_INFO_NAMED(LOGNAME, "\tGroup: %s", config.second.group.c_str());
    ROS_INFO_NAMED(LOGNAME, "\tName: %s", config.second.name.c_str());

    for (auto setting : config.second.config) {
      ROS_INFO_NAMED(LOGNAME, "\t\tSetting: %s", setting.first.c_str());
      ROS_INFO_NAMED(LOGNAME, "\t\tValue: %s", setting.second.c_str());
    }
  }
}

void printJointTrajectory(
    const trajectory_msgs::JointTrajectory& joint_trajectory) {
  ROS_INFO_NAMED(LOGNAME, "Num joints: %ld",
                 joint_trajectory.joint_names.size());
  ROS_INFO_NAMED(LOGNAME, "Num points: %ld", joint_trajectory.points.size());

  for (int i = 0; i < joint_trajectory.joint_names.size(); i++) {
    std::string name = joint_trajectory.joint_names[i];
    std::cout << name << " ";
  }

  for (int i = 0; i < joint_trajectory.points.size(); i++) {
    trajectory_msgs::JointTrajectoryPoint traj_point =
        joint_trajectory.points[i];
    std::cout << "\npositions" << std::endl;
    for (int k = 0; k < traj_point.positions.size(); k++) {
      double position = traj_point.positions[k];
      std::cout << position << " ";
    }

    std::cout << "\nvelocities" << std::endl;
    for (int k = 0; k < traj_point.velocities.size(); k++) {
      double velocity = traj_point.velocities[k];
      std::cout << velocity << " ";
    }

    std::cout << "\naccelerations" << std::endl;
    for (int k = 0; k < traj_point.accelerations.size(); k++) {
      double accel = traj_point.accelerations[k];
      std::cout << accel << " ";
    }

    std::cout << "\neffort" << std::endl;
    for (int k = 0; k < traj_point.effort.size(); k++) {
      double effort = traj_point.effort[k];
      std::cout << effort << " ";
    }
  }
}

void promptAnyInput() {
  std::cout << std::endl;
  std::cout << "Press any key to continue: ";
  getchar();
}

void addPlannerConfigurationSettings(
    planning_interface::PlannerConfigurationMap& planner_config_map,
    const std::string& group_name,
    const std::map<std::string, std::string>& setting_map) {
  planning_interface::PlannerConfigurationSettings planner_settings;
  planner_settings.group = group_name;
  planner_settings.name = setting_map.at("type");
  planner_settings.config = setting_map;
  std::string key = group_name + "[" + setting_map.at("type") + "]";
  planner_config_map[key] = planner_settings;
}

void setCurToStartState(
    planning_interface::MotionPlanRequest& req,
    const moveit::core::RobotStatePtr robot_state,
    const moveit::core::JointModelGroup* joint_model_group) {
  req.start_state.joint_state.header.stamp = ros::Time::now();
  req.start_state.joint_state.name = joint_model_group->getVariableNames();

  std::vector<double> start_joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, start_joint_values);
  req.start_state.joint_state.position = start_joint_values;

  start_joint_values.clear();
  robot_state->copyJointGroupVelocities(joint_model_group, start_joint_values);
  req.start_state.joint_state.velocity = start_joint_values;

  start_joint_values.clear();
  robot_state->copyJointGroupAccelerations(joint_model_group,
                                           start_joint_values);
  req.start_state.joint_state.effort = start_joint_values;
}

std::unique_ptr<ompl_interface::OMPLInterface> getOMPLInterface(
    const moveit::core::RobotModelConstPtr& model, const ros::NodeHandle& nh) {
  std::unique_ptr<ompl_interface::OMPLInterface> ompl_interface =
      std::make_unique<ompl_interface::OMPLInterface>(model, nh);
  return ompl_interface;
}

ompl_interface::ModelBasedPlanningContextPtr createPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const moveit::core::RobotModelPtr& robot_model,
    const moveit_msgs::MotionPlanRequest& req, const ros::NodeHandle& nh) {
  const std::string group_name = "panda_arm";
  ompl_interface::ModelBasedStateSpaceSpecification space_spec(robot_model,
                                                               group_name);
  ompl_interface::ModelBasedStateSpacePtr state_space =
      std::make_shared<ompl_interface::JointModelStateSpace>(space_spec);
  state_space->computeLocations();

  std::unique_ptr<ompl_interface::OMPLInterface> ompl_interface =
      getOMPLInterface(robot_model, nh);

  planning_interface::PlannerConfigurationMap pconfig_map =
      ompl_interface->getPlannerConfigurations();
  // printPlannerConfigMap(pconfig_map);

  ompl_interface::PlanningContextManager planning_context_manager =
      ompl_interface->getPlanningContextManager();

  auto pc = pconfig_map.end();
  pc = pconfig_map.find(req.planner_id);
  if (pc == pconfig_map.end()) {
    ROS_WARN_NAMED(LOGNAME,
                   "Cannot find planning configuration for group '%s' using "
                   "planner '%s'. Will use defaults instead.",
                   req.group_name.c_str(), req.planner_id.c_str());
    return ompl_interface::ModelBasedPlanningContextPtr();
  } else {
    ROS_INFO_NAMED(
        LOGNAME, "Found planning configuration for group %s, using planner %s.",
        req.group_name.c_str(), req.planner_id.c_str());
  }

  planning_interface::PlannerConfigurationSettings pconfig_settings =
      pc->second;

  ompl_interface::ModelBasedPlanningContextSpecification context_spec;
  context_spec.config_ = pconfig_settings.config;
  context_spec.planner_selector_ =
      planning_context_manager.getPlannerSelector();
  context_spec.constraint_sampler_manager_ =
      std::make_shared<constraint_samplers::ConstraintSamplerManager>();
  context_spec.state_space_ = state_space;

  ompl::geometric::SimpleSetupPtr ompl_simple_setup =
      std::make_shared<ompl::geometric::SimpleSetup>(state_space);
  context_spec.ompl_simple_setup_ = ompl_simple_setup;

  ompl_interface::ModelBasedPlanningContextPtr context =
      std::make_shared<ompl_interface::ModelBasedPlanningContext>(group_name,
                                                                  context_spec);

  unsigned int max_goal_samples_ = 10;
  unsigned int max_state_sampling_attempts_ = 4;
  unsigned int max_goal_sampling_attempts_ = 1000;
  unsigned int max_planning_threads_ = 4;
  double max_solution_segment_length_ = 0.0;
  unsigned int minimum_waypoint_count_ = 2;

  context->setMaximumPlanningThreads(max_planning_threads_);
  context->setMaximumGoalSamples(max_goal_samples_);
  context->setMaximumStateSamplingAttempts(max_state_sampling_attempts_);
  context->setMaximumGoalSamplingAttempts(max_goal_sampling_attempts_);
  if (max_solution_segment_length_ >
      std::numeric_limits<double>::epsilon()) {  // wtf is this
    context->setMaximumSolutionSegmentLength(max_solution_segment_length_);
  }
  context->setMinimumWaypointCount(minimum_waypoint_count_);

  context->clear();

  moveit::core::RobotStatePtr start_state =
      planning_scene->getCurrentStateUpdated(req.start_state);

  context->setPlanningScene(planning_scene);
  context->setMotionPlanRequest(req);
  context->setCompleteInitialState(*start_state);

  context->setPlanningVolume(req.workspace_parameters);
  moveit_msgs::MoveItErrorCodes error_code;
  if (!context->setPathConstraints(req.path_constraints, &error_code)) {
    ROS_ERROR_NAMED(LOGNAME, "context->setPathConstraints() error");
    return ompl_interface::ModelBasedPlanningContextPtr();
  }

  if (!context->setGoalConstraints(req.goal_constraints, req.path_constraints,
                                   &error_code)) {
    ROS_ERROR_NAMED(LOGNAME, "context->setGoalConstraints() error");
    return ompl_interface::ModelBasedPlanningContextPtr();
  }

  bool use_constraints_approximation = true;
  try {
    context->configure(nh, use_constraints_approximation);
    ROS_INFO_NAMED(LOGNAME, "%s: New planning context is set.",
                   context->getName().c_str());
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  } catch (ompl::Exception& ex) {
    ROS_ERROR_NAMED(LOGNAME, "OMPL encountered an error: %s", ex.what());
    context.reset();
  }

  return context;
}

ompl::base::PlannerPtr createPlanner(
    const ompl::base::SpaceInformationPtr& si) {
  ompl::base::PlannerPtr planner =
      std::make_shared<ompl::geometric::RRTConnect>(si);
  return planner;
}

void changePlanner(ompl_interface::ModelBasedPlanningContextPtr& context) {
  ompl::geometric::SimpleSetupPtr simple_setup = context->getOMPLSimpleSetup();
  ompl::base::SpaceInformationPtr si = simple_setup->getSpaceInformation();
  ompl::base::PlannerPtr planner = createPlanner(si);
  simple_setup->setPlanner(planner);
}

bool generatePlan(const ompl_interface::ModelBasedPlanningContextPtr& context,
                  planning_interface::MotionPlanResponse& res) {
  bool is_solved = context->solve(res);
  std::size_t state_count = res.trajectory_->getWayPointCount();
  ROS_DEBUG_STREAM("Motion planner reported a solution path with "
                   << state_count << " states");
  ROS_INFO_NAMED(LOGNAME, "Is plan generated? %d", is_solved);
  return is_solved;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pick_and_place_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  ROS_DEBUG_NAMED(LOGNAME, "Start!");

  // Initialization
  // ^^^^^^^^^^^^^^
  // ^^^^^^^^^^^^^^
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader("robot_description"));

  planning_scene_monitor::PlanningSceneMonitorPtr psm(
      new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

  /* listen for planning scene messages on topic /XXX and apply them to
                       the internal planning scene accordingly */
  psm->startSceneMonitor();
  /* listens to changes of world geometry, collision objects, and (optionally)
   * octomaps */
  psm->startWorldGeometryMonitor();
  /* listen to joint state updates as well as changes in attached collision
     objects and update the internal planning scene accordingly*/
  psm->startStateMonitor();

  /* We can also use the RobotModelLoader to get a robot model which contains
   * the robot's kinematic information */
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

  const std::string group_name = "panda_arm";

  /* We can get the most up to date robot state from the PlanningSceneMonitor
     by locking the internal planning scene for reading. This lock ensures
     that the underlying scene isn't updated while we are reading it's state.
     RobotState's are useful for computing the forward and inverse kinematics
     of the robot among many other uses */

  planning_scene_monitor::CurrentStateMonitorPtr csm = psm->getStateMonitor();
  csm->enableCopyDynamics(true);

  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
      planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

  /* Create a JointModelGroup to keep track of the current robot pose and
     planning group. The Joint Model group is useful for dealing with one set
     of joints at a time such as a left arm or a end effector */
  const moveit::core::JointModelGroup* joint_model_group =
      robot_state->getJointModelGroup(group_name);

  const std::vector<std::string>& link_model_names =
      joint_model_group->getLinkModelNames();
  ROS_INFO_NAMED(LOGNAME, "end effector name %s\n",
                 link_model_names.back().c_str());

  robot_state->setToDefaultValues(joint_model_group, "ready");
  robot_state->update();
  psm->updateSceneWithCurrentState();

  ROS_INFO_NAMED(LOGNAME, "Robot State Positions");
  robot_state->printStatePositions();

  // // We can now setup the PlanningPipeline object, which will use the ROS
  // // parameter server to determine the set of request adapters and the
  // planning
  // // plugin to use
  // planning_pipeline::PlanningPipelinePtr planning_pipeline(
  //     new planning_pipeline::PlanningPipeline(
  //         robot_model, node_handle, "planning_plugin",
  //         "request_adapters"));

  // const planning_interface::PlannerManagerPtr planner_manager =
  //     planning_pipeline->getPlannerManager();

  // Pose Goal
  // ^^^^^^^^^
  // ^^^^^^^^^
  // We will now create a motion plan request for the right arm of the Panda
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  setCurToStartState(req, robot_state, joint_model_group);

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "panda_link0";
  pose.pose.position.x = 0.5;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  req.group_name = group_name;
  req.allowed_planning_time = 5.0;
  req.planner_id = "panda_arm[RRT]";

  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints(
          "panda_link8", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  // Before planning, we will need a Read Only lock on the planning scene so
  // that it does not modify the world representation while planning
  planning_scene_monitor::LockedPlanningSceneRO lscene(psm);

  ompl_interface::ModelBasedPlanningContextPtr context =
      createPlanningContext(lscene, robot_model, req, node_handle);

  changePlanner(context);

  generatePlan(context, res);

  // /* Now, call the pipeline and check whether planning was successful. */
  // planning_pipeline->generatePlan(lscene, req, res);

  // /* Now, call the pipeline and check whether planning was successful. */
  // /* Check that the planning was successful */
  // if (res.error_code_.val != res.error_code_.SUCCESS) {
  //   ROS_ERROR("Could not compute plan successfully");
  //   return 0;
  // }

  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  // ^^^^^^^^^^^^^^^^^^^^
  // The package MoveItVisualTools provides many capabilities for visualizing
  // objects, robots, and trajectories in RViz as well as debugging tools such
  // as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(
      "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC, psm);
  visual_tools.deleteAllMarkers();

  ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>(
          "/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(),
                                     joint_model_group);
  visual_tools.trigger();

  // Execute Trajectory
  // ^^^^^^^^^^^^^^^^^^^^
  // ^^^^^^^^^^^^^^^^^^^^

  promptAnyInput();

  std::cout << "Finished!" << std::endl;

  return 0;
}

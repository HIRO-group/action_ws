// ROS
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

constexpr char LOGNAME[] = "moveit_baseline_plan";

const std::vector<double> joint_goal_pos_ = {-1.0, 0.7, 0.7, -1.0,
                                             -0.7, 2.0, 0.0};
const std::string group_name_ = "panda_arm";

void promptAnyInput() {
  std::string user_input = " ";
  std::cout << "Press any key to continue..." << std::endl;
  std::cin >> user_input;
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

moveit_msgs::Constraints createPoseGoal() {
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

  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints(
          "panda_link8", pose, tolerance_pose, tolerance_angle);
  return pose_goal;
}

moveit_msgs::Constraints createJointGoal(
    const moveit::core::RobotStatePtr& robot_state,
    const moveit::core::JointModelGroup* joint_model_group) {
  moveit::core::RobotState goal_state(*robot_state);
  goal_state.setJointGroupPositions(joint_model_group, joint_goal_pos_);
  double tolerance = 0.001;
  moveit_msgs::Constraints joint_goal =
      kinematic_constraints::constructGoalConstraints(
          goal_state, joint_model_group, tolerance);
  return joint_goal;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "moveit_baseline_plan");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  ROS_DEBUG_NAMED(LOGNAME, "Start!");

  // Start
  // ^^^^^
  // Setting up to start using a planning pipeline is pretty easy. Before we can
  // load the planner, we need two objects, a RobotModel and a PlanningScene.
  //
  // We will start by instantiating a `RobotModelLoader`_ object, which will
  // look up the robot description on the ROS parameter server and construct a
  // :moveit_core:`RobotModel` for us to use.
  //
  // .. _RobotModelLoader:
  //     http://docs.ros.org/noetic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader("robot_description"));

  // Using the RobotModelLoader, we can construct a planing scene monitor that
  // will create a planning scene, monitors planning scene diffs, and apply the
  // diffs to it's internal planning scene. We then call startSceneMonitor,
  // startWorldGeometryMonitor and startStateMonitor to fully initialize the
  // planning scene monitor
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

  /* We can get the most up to date robot state from the PlanningSceneMonitor by
     locking the internal planning scene for reading. This lock ensures that the
     underlying scene isn't updated while we are reading it's state.
     RobotState's are useful for computing the forward and inverse kinematics of
     the robot among many other uses */
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
      planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

  /* Create a JointModelGroup to keep track of the current robot pose and
     planning group. The Joint Model group is useful for dealing with one set of
     joints at a time such as a left arm or a end effector */
  const moveit::core::JointModelGroup* joint_model_group =
      robot_state->getJointModelGroup(group_name_);

  // We can now setup the PlanningPipeline object, which will use the ROS
  // parameter server to determine the set of request adapters and the planning
  // plugin to use
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
      new planning_pipeline::PlanningPipeline(
          robot_model, node_handle, "planning_plugin", "request_adapters"));

  // Pose Goal
  // ^^^^^^^^^
  // ^^^^^^^^^
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  robot_state->setToDefaultValues(joint_model_group, "ready");
  psm->updateSceneWithCurrentState();
  setCurToStartState(req, robot_state, joint_model_group);

  req.goal_constraints.clear();
  moveit_msgs::Constraints goal =
      createJointGoal(robot_state, joint_model_group);
  req.goal_constraints.push_back(goal);

  req.group_name = group_name_;
  req.allowed_planning_time = 2.0;
  req.planner_id = "panda_arm[RRT]";

  // Before planning, we will need a Read Only lock on the planning scene so
  // that it does not modify the world representation while planning
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    /* Now, call the pipeline and check whether planning was successful. */
    planning_pipeline->generatePlan(lscene, req, res);
  }
  /* Now, call the pipeline and check whether planning was successful. */
  /* Check that the planning was successful */
  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>(
          "/tacbot/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);

  // Execute Trajectory
  // ^^^^^^^^^^^^^^^^^^^^
  // ^^^^^^^^^^^^^^^^^^^^

  // insert code here

  promptAnyInput();

  std::cout << "Finished!" << std::endl;

  return 0;
}

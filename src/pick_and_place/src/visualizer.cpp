#include "visualizer.h"

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <ompl/geometric/SimpleSetup.h>
#include <visualization_msgs/MarkerArray.h>

constexpr char LOGNAME[] = "visualizer";

namespace pick_and_place {
Visualizer::Visualizer() {
  robot_marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("repulse_origin", 1, true);
  obstacle_marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("obstacle", 1, true);
  arrow_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vector_field", 1, true);
  manipulability_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("manipulability", 1, true);
  rep_state_publisher_ =
      nh_.advertise<moveit_msgs::DisplayTrajectory>("repulsed_state", 1, true);
  tree_states_publisher_ =
      nh_.advertise<moveit_msgs::DisplayTrajectory>("tree_states", 1, true);
  goal_state_publisher_ =
      nh_.advertise<moveit_msgs::DisplayTrajectory>("goal_state", 1, true);
}

void Visualizer::setContactPlanner(
    const std::shared_ptr<ContactPlanner>& contact_planner) {
  contact_planner_ = contact_planner;
}

void Visualizer::visualizeManipVec(std::size_t state_num) {
  visualization_msgs::MarkerArray marker_array;

  if (contact_planner_->vis_data_.repulsed_origin_at_link_.size() <=
          state_num ||
      contact_planner_->vis_data_.manipulability_.size() <= state_num) {
    ROS_DEBUG_NAMED(
        LOGNAME,
        "Insufficient data stored to vizualize manipulability vectors.");
    return;
  }

  auto repulsed_origin_at_link =
      contact_planner_->vis_data_.repulsed_origin_at_link_[state_num];
  std::vector<ManipulabilityMeasures> manip_per_joint =
      contact_planner_->vis_data_.manipulability_[state_num];

  for (std::size_t link_num = 0; link_num < dof_; link_num++) {
    Eigen::Vector3d origin(repulsed_origin_at_link[link_num * 3],
                           repulsed_origin_at_link[link_num * 3 + 1],
                           repulsed_origin_at_link[link_num * 3 + 2]);
    ManipulabilityMeasures manip = manip_per_joint[link_num];

    for (std::size_t vec_num = 0; vec_num < manip.eigen_values.size();
         vec_num++) {
      Eigen::Vector3d dir =
          manip.getVector(vec_num) * manip.eigen_values(vec_num) * 0.2;
      dir = dir;
      Eigen::Vector3d vec = origin + dir;

      uint32_t shape = visualization_msgs::Marker::ARROW;

      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time::now();
      // marker.ns = "basic_shapes";
      marker.id = link_num * 3 + vec_num;
      marker.type = shape;

      // Set the marker action.  Options are ADD, DELETE, and DELETEALL
      marker.action = visualization_msgs::Marker::ADD;

      geometry_msgs::Point start_pt;
      start_pt.x = origin[0];
      start_pt.y = origin[1];
      start_pt.z = origin[2];

      geometry_msgs::Point end_pt;
      end_pt.x = vec[0];
      end_pt.y = vec[1];
      end_pt.z = vec[2];

      marker.points.push_back(start_pt);
      marker.points.push_back(end_pt);

      marker.scale.x = 0.01;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;

      if (manip.pass) {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
      } else {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
      }

      marker.lifetime = ros::Duration();
      marker_array.markers.push_back(marker);
    }
  }
  manipulability_pub_.publish(marker_array);
}

void Visualizer::visualizeRepulseVec(std::size_t state_num) {
  if (contact_planner_->vis_data_.repulsed_origin_at_link_.size() <=
          state_num ||
      contact_planner_->vis_data_.repulsed_vec_at_link_.size() <= state_num) {
    ROS_DEBUG_NAMED(LOGNAME,
                    "Insufficient data stored to vizualize repulsive vector.");
    return;
  }

  visualization_msgs::MarkerArray marker_array;
  auto repulsed_origin_at_link =
      contact_planner_->vis_data_.repulsed_origin_at_link_[state_num];
  auto repulsed_vec_at_link =
      contact_planner_->vis_data_.repulsed_vec_at_link_[state_num];

  // std::cout << "repulsed_origin_at_link.size(): "
  //           << repulsed_origin_at_link.size() << std::endl;

  std::size_t max_num = (int)(repulsed_origin_at_link.size() / 3);
  for (std::size_t i = 0; i < max_num; i++) {
    // std::cout << "i: " << i << std::endl;
    Eigen::Vector3d origin(repulsed_origin_at_link[i * 3],
                           repulsed_origin_at_link[i * 3 + 1],
                           repulsed_origin_at_link[i * 3 + 2]);

    Eigen::Vector3d dir(repulsed_vec_at_link[i * 3],
                        repulsed_vec_at_link[i * 3 + 1],
                        repulsed_vec_at_link[i * 3 + 2]);

    Eigen::Vector3d vec = origin + dir;

    uint32_t shape = visualization_msgs::Marker::ARROW;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    // marker.ns = "basic_shapes";
    marker.id = i;
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and DELETEALL
    marker.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point start_pt;
    start_pt.x = origin[0];
    start_pt.y = origin[1];
    start_pt.z = origin[2];

    geometry_msgs::Point end_pt;
    end_pt.x = vec[0];
    end_pt.y = vec[1];
    end_pt.z = vec[2];

    marker.points.push_back(start_pt);
    marker.points.push_back(end_pt);

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    marker_array.markers.push_back(marker);
  }
  arrow_pub_.publish(marker_array);
}

void Visualizer::visualizeRepulseOrigin(std::size_t state_num) {
  if (contact_planner_->vis_data_.sample_joint_angles_.size() <= state_num) {
    ROS_DEBUG_NAMED(LOGNAME,
                    "Insufficient data stored to vizualize repulsion origin.");
    return;
  }

  visualization_msgs::MarkerArray marker_array;
  auto repulsed_origin_at_link =
      contact_planner_->vis_data_.repulsed_origin_at_link_[state_num];
  std::size_t max_num = (int)(repulsed_origin_at_link.size() / 3);

  for (std::size_t i = 0; i < max_num; i++) {
    // std::cout << "translation for " << link_names[i] << "\n"
    //           << translation << std::endl;

    Eigen::Vector3d origin(repulsed_origin_at_link[i * 3],
                           repulsed_origin_at_link[i * 3 + 1],
                           repulsed_origin_at_link[i * 3 + 2]);

    uint32_t shape = visualization_msgs::Marker::SPHERE;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    // marker.ns = "basic_shapes";
    marker.id = i;
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and DELETEALL
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = origin[0];
    marker.pose.position.y = origin[1];
    marker.pose.position.z = origin[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_array.markers.push_back(marker);
  }
  robot_marker_pub_.publish(marker_array);
}

void Visualizer::visualizeObstacleMarker(
    const std::vector<Eigen::Vector3d>& obstacle_pos) {
  visualization_msgs::MarkerArray marker_array;

  for (std::size_t i = 0; i < obstacle_pos.size(); i++) {
    uint32_t shape = visualization_msgs::Marker::SPHERE;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    // marker.ns = "basic_shapes";
    marker.id = i;
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and DELETEALL
    marker.action = visualization_msgs::Marker::ADD;

    Eigen::Vector3d pos = obstacle_pos[i];

    marker.pose.position.x = pos[0];
    marker.pose.position.y = pos[1];
    marker.pose.position.z = pos[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_array.markers.push_back(marker);
  }
  obstacle_marker_pub_.publish(marker_array);
}

void Visualizer::visualizeTrajectory(
    const planning_interface::MotionPlanResponse& res, std::string name) {
  // first publish the final path
  moveit_msgs::DisplayTrajectory display_final_traj;
  moveit_msgs::MotionPlanResponse resp_final_traj;
  ros::Publisher pub_final_traj;

  pub_final_traj = nh_.advertise<moveit_msgs::DisplayTrajectory>(name, 1, true);
  res.getMessage(resp_final_traj);
  display_final_traj.trajectory_start = resp_final_traj.trajectory_start;
  display_final_traj.trajectory.push_back(resp_final_traj.trajectory);
  pub_final_traj.publish(display_final_traj);
  trajectory_publishers_.push_back(pub_final_traj);

  moveit_msgs::DisplayTrajectory display_raw_traj;
  moveit_msgs::MotionPlanResponse resp_raw_traj;
  ros::Publisher pub_raw_traj;

  // next publish the raw path aka the path without smoothing
  pub_raw_traj =
      nh_.advertise<moveit_msgs::DisplayTrajectory>(name + "_raw", 1, true);
  robot_trajectory::RobotTrajectory trajectory =
      contact_planner_->context_->getRawTrajectory();
  trajectory.getRobotTrajectoryMsg(resp_raw_traj.trajectory);

  display_raw_traj.trajectory_start = resp_raw_traj.trajectory_start;
  display_raw_traj.trajectory.push_back(resp_raw_traj.trajectory);
  pub_raw_traj.publish(display_raw_traj);
  trajectory_publishers_.push_back(pub_raw_traj);
}

void Visualizer::visualizeTreeStates() {
  ompl::geometric::SimpleSetupPtr simple_setup =
      contact_planner_->context_->getOMPLSimpleSetup();
  ompl::base::SpaceInformationPtr si = simple_setup->getSpaceInformation();
  ompl::base::PlannerPtr planner = simple_setup->getPlanner();

  ompl::base::PlannerData planner_data(si);
  planner->getPlannerData(planner_data);

  ompl::base::StateStoragePtr storage = planner_data.extractStateStorage();
  std::size_t num_states = storage->size();
  std::cout << "num_states: " << num_states << std::endl;
  std::vector<const ompl::base::State*> states = storage->getStates();

  std::vector<std::vector<double>> joint_states;
  for (std::size_t i = 0; i < num_states; i++) {
    const ompl::base::State* state = states[i];
    const ompl::base::RealVectorStateSpace::StateType& vec_state =
        *state->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<double> joint_angles = utilities::toStlVec(vec_state, dof_);
    joint_states.emplace_back(joint_angles);
  }

  const std::size_t num_display_states = joint_states.size();
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::MotionPlanResponse response;

  sensor_msgs::JointState joint_start_state;
  std::vector<std::string> names =
      contact_planner_->joint_model_group_->getActiveJointModelNames();

  std::size_t num_joints = names.size();
  std::vector<double> joint_angles = joint_states[0];
  std::cout << utilities::toEigen(joint_angles).transpose() << std::endl;
  joint_start_state.name = names;
  joint_start_state.position = joint_angles;
  joint_start_state.velocity = std::vector<double>(num_joints, 0.0);
  joint_start_state.effort = std::vector<double>(num_joints, 0.0);

  moveit_msgs::RobotState robot_start_state;
  robot_start_state.joint_state = joint_start_state;

  response.group_name = group_name_;
  response.trajectory_start = robot_start_state;
  display_trajectory.trajectory_start = response.trajectory_start;

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = names;

  for (std::size_t i = 0; i < num_display_states; i++) {
    trajectory_msgs::JointTrajectoryPoint point;
    joint_angles = joint_states[i];

    point.positions = joint_angles;
    point.velocities = std::vector<double>(num_joints, 0.0);
    point.accelerations = std::vector<double>(num_joints, 0.0);
    point.effort = std::vector<double>(num_joints, 0.0);
    point.time_from_start = ros::Duration(1.5);
    joint_trajectory.points.push_back(point);
  }
  moveit_msgs::RobotTrajectory robot_trajectory;
  robot_trajectory.joint_trajectory = joint_trajectory;
  response.trajectory = robot_trajectory;
  display_trajectory.trajectory.push_back(response.trajectory);
  tree_states_publisher_.publish(display_trajectory);
}

void Visualizer::visualizeGoalState() {
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::MotionPlanResponse response;

  sensor_msgs::JointState joint_start_state;
  std::vector<std::string> names =
      contact_planner_->joint_model_group_->getActiveJointModelNames();

  std::size_t num_joints = names.size();
  std::vector<double> joint_angles = contact_planner_->joint_goal_pos_;
  std::cout << utilities::toEigen(joint_angles).transpose() << std::endl;
  joint_start_state.name = names;
  joint_start_state.position = joint_angles;
  joint_start_state.velocity = std::vector<double>(num_joints, 0.0);
  joint_start_state.effort = std::vector<double>(num_joints, 0.0);

  moveit_msgs::RobotState robot_start_state;
  robot_start_state.joint_state = joint_start_state;

  response.group_name = group_name_;
  response.trajectory_start = robot_start_state;
  display_trajectory.trajectory_start = response.trajectory_start;

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = names;

  trajectory_msgs::JointTrajectoryPoint point;
  joint_angles = contact_planner_->joint_goal_pos_;

  point.positions = joint_angles;
  point.velocities = std::vector<double>(num_joints, 0.0);
  point.accelerations = std::vector<double>(num_joints, 0.0);
  point.effort = std::vector<double>(num_joints, 0.0);
  point.time_from_start = ros::Duration(1.5);
  joint_trajectory.points.push_back(point);

  moveit_msgs::RobotTrajectory robot_trajectory;
  robot_trajectory.joint_trajectory = joint_trajectory;
  response.trajectory = robot_trajectory;
  display_trajectory.trajectory.push_back(response.trajectory);
  goal_state_publisher_.publish(display_trajectory);
}

void Visualizer::visualizeRepulsedState() {
  std::string user_input = " ";

  while (user_input != "q") {
    if (contact_planner_->vis_data_.sample_joint_angles_.size() <=
            viz_state_idx_ ||
        contact_planner_->vis_data_.sample_desired_angles_.size() <=
            viz_state_idx_) {
      ROS_DEBUG_NAMED(LOGNAME,
                      "Insufficient data stored to vizualize repulsed states.");
      return;
    }

    std::cout << "Displaying repulsed state with idx: " << viz_state_idx_
              << std::endl;

    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit_msgs::MotionPlanResponse response;

    sensor_msgs::JointState joint_start_state;
    std::vector<std::string> names =
        contact_planner_->joint_model_group_->getActiveJointModelNames();

    std::size_t num_joints = names.size();
    std::vector<double> joint_angles =
        contact_planner_->vis_data_.sample_joint_angles_[viz_state_idx_];
    std::cout << utilities::toEigen(joint_angles).transpose() << std::endl;
    joint_start_state.name = names;
    joint_start_state.position = joint_angles;
    joint_start_state.velocity = std::vector<double>(num_joints, 0.0);
    joint_start_state.effort = std::vector<double>(num_joints, 0.0);

    moveit_msgs::RobotState robot_start_state;
    robot_start_state.joint_state = joint_start_state;

    response.group_name = group_name_;
    response.trajectory_start = robot_start_state;

    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = names;

    trajectory_msgs::JointTrajectoryPoint point;

    joint_angles =
        contact_planner_->vis_data_.sample_joint_angles_[viz_state_idx_];
    point.positions = joint_angles;
    point.velocities = std::vector<double>(num_joints, 0.0);
    point.accelerations = std::vector<double>(num_joints, 0.0);
    point.effort = std::vector<double>(num_joints, 0.0);
    point.time_from_start = ros::Duration(0.1);
    joint_trajectory.points.push_back(point);

    joint_angles =
        contact_planner_->vis_data_.sample_desired_angles_[viz_state_idx_];
    std::cout << utilities::toEigen(joint_angles).transpose() << std::endl;

    point.positions = joint_angles;
    point.velocities = std::vector<double>(num_joints, 0.0);
    point.accelerations = std::vector<double>(num_joints, 0.0);
    point.effort = std::vector<double>(num_joints, 0.0);
    point.time_from_start = ros::Duration(1.5);
    joint_trajectory.points.push_back(point);

    moveit_msgs::RobotTrajectory robot_trajectory;
    robot_trajectory.joint_trajectory = joint_trajectory;

    response.trajectory = robot_trajectory;

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    rep_state_publisher_.publish(display_trajectory);

    visualizeRepulseVec(viz_state_idx_);
    visualizeRepulseOrigin(viz_state_idx_);
    visualizeManipVec(viz_state_idx_);
    visualizeObstacleMarker(
        contact_planner_->vis_data_.sample_obstacle_pos_[viz_state_idx_]);

    viz_state_idx_++;

    std::cout << "Press 'q' to exit this visualization or 'c' to go to the "
                 "next state "
              << std::endl;
    std::cin >> user_input;
  }
}

}  // namespace pick_and_place
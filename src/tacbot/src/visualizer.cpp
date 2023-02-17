#include "visualizer.h"

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <ompl/geometric/SimpleSetup.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>
#include <thread>

constexpr char LOGNAME[] = "visualizer";

namespace tacbot {
Visualizer::Visualizer() {
  robot_repulse_origin_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("repulse_origin", 1, true);
  obstacle_marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("obstacle", 1, true);
  arrow_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vector_field", 1, true);
  nearrand_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("nearrand_field", 1, true);
  arrow_avg_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vec_avg_field", 1, true);
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
    ROS_INFO_NAMED(
        LOGNAME,
        "Insufficient data stored to vizualize manipulability vectors.");
    return;
  }

  auto repulsed_origin_at_link =
      contact_planner_->vis_data_.repulsed_origin_at_link_[state_num];
  std::vector<ManipulabilityMeasures> manip_per_joint =
      contact_planner_->vis_data_.manipulability_[state_num];
  std::size_t max_num = (int)(repulsed_origin_at_link.size() / 3);

  for (std::size_t link_num = 0; link_num < max_num; link_num++) {
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

void Visualizer::visualizeAvgRepulseVec(std::size_t state_num) {
  if (contact_planner_->vis_data_.repulsed_vec_avg_at_link_.size() <=
          state_num ||
      contact_planner_->vis_data_.repulsed_origin_at_link_.size() <=
          state_num) {
    ROS_INFO_NAMED(LOGNAME,
                   "Insufficient data stored to vizualize repulsive vector.");
    return;
  }

  visualization_msgs::MarkerArray marker_array;
  auto repulsed_origin_at_link =
      contact_planner_->vis_data_.repulsed_origin_at_link_[state_num];
  std::vector<Eigen::Vector3d> repulsed_vec_at_link =
      contact_planner_->vis_data_.repulsed_vec_avg_at_link_[state_num];

  // std::cout << "repulsed_origin_at_link.size(): "
  //           << repulsed_origin_at_link.size() << std::endl;

  for (std::size_t i = 0; i < repulsed_vec_at_link.size(); i++) {
    // std::cout << "i: " << i << std::endl;
    Eigen::Vector3d origin(repulsed_origin_at_link[i * 3],
                           repulsed_origin_at_link[i * 3 + 1],
                           repulsed_origin_at_link[i * 3 + 2]);

    Eigen::Vector3d dir = repulsed_vec_at_link[i];

    // std::cout << "i: " << i << std::endl;
    // std::cout << "origin: " << origin.transpose() << std::endl;
    // std::cout << "dir: " << dir.transpose() << std::endl;

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
    marker.color.g = 0.5f;
    marker.color.b = 0.5f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    marker_array.markers.push_back(marker);
  }
  arrow_avg_pub_.publish(marker_array);
}

void Visualizer::visualizeRepulseVec(std::size_t state_num) {
  if (contact_planner_->vis_data_.repulsed_origin_at_link_.size() <=
          state_num ||
      contact_planner_->vis_data_.repulsed_vec_at_link_.size() <= state_num) {
    ROS_INFO_NAMED(LOGNAME,
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

    // std::cout << "i: " << i << std::endl;
    // std::cout << "origin: " << origin.transpose() << std::endl;
    // std::cout << "dir: " << dir.transpose() << std::endl;

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

void Visualizer::visualizeNearRandVec(std::size_t state_num) {
  if (contact_planner_->vis_data_.nearrand_origin_at_link_.size() <=
          state_num ||
      contact_planner_->vis_data_.nearrand_vec_at_link_.size() <= state_num) {
    ROS_INFO_NAMED(LOGNAME,
                   "Insufficient data stored to vizualize nearrand vector.");
    return;
  }

  visualization_msgs::MarkerArray marker_array;
  auto repulsed_origin_at_link =
      contact_planner_->vis_data_.nearrand_origin_at_link_[state_num];
  auto repulsed_vec_at_link =
      contact_planner_->vis_data_.nearrand_vec_at_link_[state_num];

  // std::cout << "repulsed_origin_at_link.size(): "
  //           << repulsed_origin_at_link.size() << std::endl;
  std::cout
      << "nearrand_dot_at_link: "
      << contact_planner_->vis_data_.nearrand_dot_at_link[state_num].transpose()
      << std::endl;

  std::size_t max_num = (int)(repulsed_origin_at_link.size() / 3);
  for (std::size_t i = 0; i < max_num; i++) {
    // std::cout << "i: " << i << std::endl;
    Eigen::Vector3d origin(repulsed_origin_at_link[i * 3],
                           repulsed_origin_at_link[i * 3 + 1],
                           repulsed_origin_at_link[i * 3 + 2]);

    Eigen::Vector3d dir(repulsed_vec_at_link[i * 3],
                        repulsed_vec_at_link[i * 3 + 1],
                        repulsed_vec_at_link[i * 3 + 2]);

    // std::cout << "i: " << i << std::endl;
    // std::cout << "origin: " << origin.transpose() << std::endl;
    // std::cout << "dir: " << dir.transpose() << std::endl;

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
    marker.color.g = 0.5f;
    marker.color.b = 0.5f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    marker_array.markers.push_back(marker);
  }
  nearrand_pub_.publish(marker_array);
}

void Visualizer::visualizeRepulseOrigin(std::size_t state_num) {
  if (contact_planner_->vis_data_.sample_joint_angles_.size() <= state_num) {
    ROS_INFO_NAMED(LOGNAME,
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

    marker.scale.x = 0.025;
    marker.scale.y = 0.025;
    marker.scale.z = 0.025;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_array.markers.push_back(marker);
  }
  robot_repulse_origin_pub_.publish(marker_array);
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

void Visualizer::visualizeTrajectory(
    const moveit_msgs::MotionPlanResponse& resp_final_traj, std::string name) {
  // first publish the final path
  moveit_msgs::DisplayTrajectory display_final_traj;
  ros::Publisher pub_final_traj;

  pub_final_traj = nh_.advertise<moveit_msgs::DisplayTrajectory>(name, 1, true);
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

void Visualizer::visualizeVertices() {
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

  std::size_t num_edges = planner_data.numEdges();
  std::size_t num_vertices = planner_data.numVertices();
  ROS_INFO_NAMED(LOGNAME, "num_edges %ld", num_edges);
  ROS_INFO_NAMED(LOGNAME, "num_vertices %ld", num_vertices);
  ROS_INFO_NAMED(LOGNAME, "Computing edge weights.");
  planner_data.computeEdgeWeights(*contact_planner_->optimization_objective_);
  std::string user_input = " ";

  for (std::size_t i = 0; i < num_vertices; i++) {
    if (user_input == "q") {
      break;
    }

    std::cout << "vertex: " << i << std::endl;

    std::map<unsigned int, const ompl::base::PlannerDataEdge*> edgeMap;
    int num_outgoing = planner_data.getEdges(i, edgeMap);
    if (num_outgoing == 0) {
      continue;
    }

    for (auto edgeElem : edgeMap) {
      if (user_input == "q") {
        break;
      }
      ompl::base::Cost* cost;
      planner_data.getEdgeWeight(i, edgeElem.first, cost);
      std::cout << "Cost: " << cost->value() << std::endl;

      ompl::base::PlannerDataVertex ver1 = planner_data.getVertex(i);
      ompl::base::PlannerDataVertex ver2 =
          planner_data.getVertex(edgeElem.first);

      const ompl::base::State* state1 = ver1.getState();
      const ompl::base::State* state2 = ver2.getState();

      const ompl::base::RealVectorStateSpace::StateType& vec_state1 =
          *state1->as<ompl::base::RealVectorStateSpace::StateType>();
      std::vector<double> joint_angles1 = utilities::toStlVec(vec_state1, dof_);

      const ompl::base::State* state = states[i];
      const ompl::base::RealVectorStateSpace::StateType& vec_state2 =
          *state2->as<ompl::base::RealVectorStateSpace::StateType>();
      std::vector<double> joint_angles2 = utilities::toStlVec(vec_state2, dof_);

      visualizeTwoStates(joint_angles1, joint_angles2);

      std::cout << "Press 'q' to exit this visualization or 'c' to go to the "
                   "next state "
                << std::endl;
      std::cin >> user_input;
    }
  }
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

void Visualizer::visualizeTwoStates(std::vector<double> joint_angles1,
                                    std::vector<double> joint_angles2) {
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::MotionPlanResponse response;

  sensor_msgs::JointState joint_start_state;
  std::vector<std::string> names =
      contact_planner_->joint_model_group_->getActiveJointModelNames();

  std::cout << "joint_angles1: "
            << utilities::toEigen(joint_angles1).transpose() << std::endl;
  std::size_t num_joints = names.size();
  joint_start_state.name = names;
  joint_start_state.position = joint_angles1;
  joint_start_state.velocity = std::vector<double>(num_joints, 0.0);
  joint_start_state.effort = std::vector<double>(num_joints, 0.0);

  moveit_msgs::RobotState robot_start_state;
  robot_start_state.joint_state = joint_start_state;

  response.group_name = group_name_;
  response.trajectory_start = robot_start_state;

  trajectory_msgs::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = names;

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = joint_angles1;
  point.velocities = std::vector<double>(num_joints, 0.0);
  point.accelerations = std::vector<double>(num_joints, 0.0);
  point.effort = std::vector<double>(num_joints, 0.0);
  point.time_from_start = ros::Duration(0.1);
  joint_trajectory.points.push_back(point);

  std::cout << "joint_angles2: "
            << utilities::toEigen(joint_angles2).transpose() << std::endl;
  point.positions = joint_angles2;
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
}

void Visualizer::visualizeRepulsedState() {
  std::string user_input = " ";

  std::size_t num_sample_angles =
      contact_planner_->vis_data_.sample_joint_angles_.size();
  std::size_t num_desired_angles =
      contact_planner_->vis_data_.sample_desired_angles_.size();

  std::cout << "num_sample_angles: " << num_sample_angles << std::endl;

  while (user_input != "q") {
    if (num_sample_angles <= viz_state_idx_ ||
        num_desired_angles <= viz_state_idx_) {
      ROS_INFO_NAMED(LOGNAME,
                     "Insufficient data stored to vizualize repulsed states.");
      return;
    }

    std::cout << "Displaying repulsed state with idx: " << viz_state_idx_
              << std::endl;

    visualizeTwoStates(
        contact_planner_->vis_data_.sample_joint_angles_[viz_state_idx_],
        contact_planner_->vis_data_.sample_desired_angles_[viz_state_idx_]);

    visualizeRepulseVec(viz_state_idx_);
    visualizeNearRandVec(viz_state_idx_);
    visualizeAvgRepulseVec(viz_state_idx_);
    visualizeRepulseOrigin(viz_state_idx_);
    visualizeManipVec(viz_state_idx_);
    visualizeObstacleMarker(
        contact_planner_->vis_data_.sample_obstacle_pos_[viz_state_idx_]);

    viz_state_idx_ += 1;

    std::cout << "Press 'q' to exit this visualization or 'c' to go to the "
                 "next state "
              << std::endl;
    std::cin >> user_input;

    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

}  // namespace tacbot
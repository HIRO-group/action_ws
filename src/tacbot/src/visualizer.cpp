#include "visualizer.h"

#include <moveit_msgs/DisplayTrajectory.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>
#include <thread>

constexpr char LOGNAME[] = "visualizer";

const float MAX_RGB = 1.0f;
const float MAX_HUE_VALUE = 270.0f;

namespace tacbot {
Visualizer::Visualizer(const std::shared_ptr<VisualizerData>& vis_data)
    : vis_data_(vis_data) {
  robot_repulse_origin_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("repulse_origin", 1, true);
  obstacle_marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("obstacle", 1, true);
  arrow_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("vector_field", 1, true);
  nearrand_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("nearrand_field", 1, true);
  ee_path_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("ee_path_pub", 1, true);
  rep_state_publisher_ =
      nh_.advertise<moveit_msgs::DisplayTrajectory>("repulsed_state", 1, true);
  goal_state_publisher_ =
      nh_.advertise<moveit_msgs::DisplayTrajectory>("goal_state", 1, true);
}

void Visualizer::visualizeEEPath() {
  if (vis_data_->ee_path_pts_.size() <= 0) {
    ROS_INFO_NAMED(LOGNAME,
                   "Insufficient data stored to vizualize ee path pts.");
    return;
  }

  std::vector<Eigen::Vector3d> ee_path_pts = vis_data_->ee_path_pts_;
  std::size_t num_pts = ee_path_pts.size();

  visualization_msgs::MarkerArray marker_array;
  std::cout << "num_pts ee_path_pts: " << num_pts << std::endl;

  for (std::size_t i = 0; i < num_pts; i++) {
    // std::cout << "i: " << i << std::endl;
    Eigen::Vector3d pt = ee_path_pts[i];

    uint32_t shape = visualization_msgs::Marker::SPHERE;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    // marker.ns = "basic_shapes";
    marker.id = i;
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and DELETEALL
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = pt[0];
    marker.pose.position.y = pt[1];
    marker.pose.position.z = pt[2];

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.1 * i / num_pts + 0.001;
    marker.scale.y = 0.1 * i / num_pts + 0.001;
    marker.scale.z = 0.1 * i / num_pts + 0.001;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0 * i / num_pts + 0.001;

    marker.lifetime = ros::Duration();
    marker_array.markers.push_back(marker);
  }
  ee_path_pub_.publish(marker_array);
}

void Visualizer::visualizeRepulseVec(std::size_t state_num) {
  if (vis_data_->repulsed_origin_at_link_.size() <= state_num ||
      vis_data_->repulsed_vec_at_link_.size() <= state_num) {
    ROS_INFO_NAMED(LOGNAME,
                   "Insufficient data stored to vizualize repulsive vector.");
    return;
  }

  visualization_msgs::MarkerArray marker_array;
  auto repulsed_origin_at_link = vis_data_->repulsed_origin_at_link_[state_num];
  auto repulsed_vec_at_link = vis_data_->repulsed_vec_at_link_[state_num];

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
  if (vis_data_->nearrand_origin_at_link_.size() <= state_num ||
      vis_data_->nearrand_vec_at_link_.size() <= state_num) {
    ROS_INFO_NAMED(LOGNAME,
                   "Insufficient data stored to vizualize nearrand vector.");
    return;
  }

  visualization_msgs::MarkerArray marker_array;
  auto repulsed_origin_at_link = vis_data_->nearrand_origin_at_link_[state_num];
  auto repulsed_vec_at_link = vis_data_->nearrand_vec_at_link_[state_num];

  // std::cout << "repulsed_origin_at_link.size(): "
  //           << repulsed_origin_at_link.size() << std::endl;
  std::cout << "nearrand_dot_at_link: "
            << vis_data_->nearrand_dot_at_link[state_num].transpose()
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
  if (vis_data_->sample_joint_angles_.size() <= state_num) {
    ROS_INFO_NAMED(LOGNAME,
                   "Insufficient data stored to vizualize repulsion origin.");
    return;
  }

  visualization_msgs::MarkerArray marker_array;
  auto repulsed_origin_at_link = vis_data_->repulsed_origin_at_link_[state_num];
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

// Functions to set colour of points while displaying obstacle map and cost map
void Visualizer::HSVToRGB(struct g_hsv& hsv, std_msgs::ColorRGBA& rgb) {
  float c = hsv.s * hsv.v;
  float hp = hsv.h / 60.0;
  float x = c * (1.0f - fabs(fmod(hp, 2.0f) - 1.0f));
  float m = hsv.v - c;
  if (hp < 0 || hp >= 6) {
    setRGB(rgb, 0, 0, 0);
  } else if (hp < 1) {
    setRGB(rgb, c, x, 0);
  } else if (hp < 2) {
    setRGB(rgb, x, c, 0);
  } else if (hp < 3) {
    setRGB(rgb, 0, c, x);
  } else if (hp < 4) {
    setRGB(rgb, 0, x, c);
  } else if (hp < 5) {
    setRGB(rgb, x, 0, c);
  } else {
    setRGB(rgb, c, 0, x);
  }
  rgb.r = (rgb.r + m) * MAX_RGB;
  rgb.g = (rgb.g + m) * MAX_RGB;
  rgb.b = (rgb.b + m) * MAX_RGB;
  // ROS_INFO("HSV %f, %f, %f, C %f, hp %f x %f, m %f, RGB %f, %f, %f", hsv.h,
  // hsv.s, hsv.v, c, hp, x, m, rgb.r, rgb.g, rgb.b);
}

void Visualizer::setRGB(std_msgs::ColorRGBA& rgb, double r, double g,
                        double b) {
  rgb.r = r;
  rgb.g = g;
  rgb.b = b;
}

void Visualizer::computeColorForValue(std_msgs::ColorRGBA& color,
                                      double gradientValue, double maxValue) {
  color.a = 1.0;
  g_hsv hsv;
  hsv.h = (1 - gradientValue / maxValue) * MAX_HUE_VALUE;
  hsv.s = 1.0;
  hsv.v = 1.0;
  HSVToRGB(hsv, color);
}

void Visualizer::visualizeObstacleMarker(
    const std::vector<tacbot::ObstacleGroup>& obstacles) {
  visualization_msgs::MarkerArray marker_array;

  for (std::size_t k = 0; k < obstacles.size(); k++) {
    tacbot::ObstacleGroup obstacle = obstacles[k];
    std::vector<tacbot::PointObstacle> point_obstacles =
        obstacle.point_obstacles;
    for (std::size_t i = 0; i < point_obstacles.size(); i++) {
      tacbot::PointObstacle point_obst = point_obstacles[i];
      uint32_t shape = visualization_msgs::Marker::SPHERE;

      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time::now();
      // marker.ns = "basic_shapes";
      marker.id = marker_array.markers.size();
      marker.type = shape;

      // Set the marker action.  Options are ADD, DELETE, and DELETEALL
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = point_obst.pos[0];
      marker.pose.position.y = point_obst.pos[1];
      marker.pose.position.z = point_obst.pos[2];
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;

      computeColorForValue(marker.color, obstacle.cost, obstacle.MAX_COST);

      // marker.color.r = 1.0f;
      // marker.color.g = 0.0f;
      // marker.color.b = 0.0f;
      // marker.color.a = 1.0;

      marker.lifetime = ros::Duration();

      marker_array.markers.push_back(marker);
    }
    obstacle_marker_pub_.publish(marker_array);
  }
}

void Visualizer::visualizeTrajectory(
    const moveit_msgs::MotionPlanResponse& traj, std::string name) {
  // first publish the final path
  moveit_msgs::DisplayTrajectory display_traj;
  ros::Publisher traj_pub;

  traj_pub = nh_.advertise<moveit_msgs::DisplayTrajectory>(name, 1, true);
  display_traj.trajectory_start = traj.trajectory_start;
  display_traj.trajectory.push_back(traj.trajectory);
  traj_pub.publish(display_traj);
  trajectory_publishers_.push_back(traj_pub);

  // moveit_msgs::DisplayTrajectory display_raw_traj;
  // moveit_msgs::MotionPlanResponse resp_raw_traj;
  // ros::Publisher pub_raw_traj;

  // next publish the raw path aka the path without smoothing
  // pub_raw_traj =
  //     nh_.advertise<moveit_msgs::DisplayTrajectory>(name + "_raw", 1, true);
  // robot_trajectory::RobotTrajectory trajectory =
  //     contact_planner_->context_->getRawTrajectory();
  // trajectory.getRobotTrajectoryMsg(resp_raw_traj.trajectory);

  // display_raw_traj.trajectory_start = resp_raw_traj.trajectory_start;
  // display_raw_traj.trajectory.push_back(resp_raw_traj.trajectory);
  // pub_raw_traj.publish(display_raw_traj);
  // trajectory_publishers_.push_back(pub_raw_traj);
}

void Visualizer::visualizeTrajectory(
    const robot_trajectory::RobotTrajectoryPtr& traj, std::string name) {
  // first publish the final path
  moveit_msgs::DisplayTrajectory display_traj;
  moveit_msgs::MotionPlanResponse moveit_traj;
  ros::Publisher traj_pub;

  robot_trajectory::RobotTrajectory trajectory = *traj;
  trajectory.getRobotTrajectoryMsg(moveit_traj.trajectory);

  traj_pub = nh_.advertise<moveit_msgs::DisplayTrajectory>(name, 1, true);
  display_traj.trajectory_start = moveit_traj.trajectory_start;
  display_traj.trajectory.push_back(moveit_traj.trajectory);
  traj_pub.publish(display_traj);
  trajectory_publishers_.push_back(traj_pub);
}

void Visualizer::visualizeGoalState(const std::vector<std::string>& names,
                                    const std::vector<double>& joint_goal_pos) {
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::MotionPlanResponse response;

  sensor_msgs::JointState joint_start_state;

  std::size_t num_joints = names.size();
  std::vector<double> joint_angles = joint_goal_pos;
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
  joint_angles = joint_goal_pos;

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

void Visualizer::visualizeTwoStates(const std::vector<std::string>& names,
                                    std::vector<double> joint_angles1,
                                    std::vector<double> joint_angles2) {
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::MotionPlanResponse response;

  sensor_msgs::JointState joint_start_state;

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

void Visualizer::visualizeRepulsedState(const std::vector<std::string>& names) {
  std::string user_input = " ";

  std::size_t num_sample_angles = vis_data_->sample_joint_angles_.size();
  std::size_t num_desired_angles = vis_data_->sample_desired_angles_.size();

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

    visualizeTwoStates(names, vis_data_->sample_joint_angles_[viz_state_idx_],
                       vis_data_->sample_desired_angles_[viz_state_idx_]);
    visualizeRepulseVec(viz_state_idx_);
    visualizeNearRandVec(viz_state_idx_);
    visualizeRepulseOrigin(viz_state_idx_);
    // visualizeObstacleMarker(vis_data_->sample_obstacle_pos_[viz_state_idx_]);

    viz_state_idx_ += 1;

    std::cout << "Press 'q' to exit this visualization or 'c' to go to the "
                 "next state "
              << std::endl;
    std::cin >> user_input;
  }
}

}  // namespace tacbot
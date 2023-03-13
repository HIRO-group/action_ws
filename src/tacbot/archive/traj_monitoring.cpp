
trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
    "/joint_position_controller/contact_trajectory", 1);

execution_monitor_sub_ =
    nh_.subscribe("/joint_position_controller/trajectory_monitor", 1,
                  &ContactPlanner::executionMonitorCallback, this);

void ContactPlanner::executionMonitorCallback(const TrajExecutionMonitor& msg) {
  monitor_mtx_.lock();
  monitor_msg_ = msg;
  monitor_mtx_.unlock();
}

void ContactPlanner::monitorExecution() {
  moveit_msgs::MotionPlanResponse msg;
  plan_response_.getMessage(msg);
  std::size_t num_pts = msg.trajectory.joint_trajectory.points.size();
  ROS_INFO_NAMED(LOGNAME, "Monitoring trajectory with num pts: %ld", num_pts);
  ros::Rate rate(30);  // hz
  int is_valid = 1;
  while (ros::ok() && monitor_msg_.pt_idx <= num_pts && is_valid != -1) {
    monitor_mtx_.lock();
    ROS_INFO_NAMED(LOGNAME, "monitor_msg_.pt_idx %d", monitor_msg_.pt_idx);
    is_valid = monitor_msg_.is_valid;
    ROS_INFO_NAMED(LOGNAME, "is_valid %d", is_valid);
    if (is_valid == -1) {
      ROS_ERROR_NAMED(LOGNAME, "Execution failure on trajectory point %d",
                      monitor_msg_.pt_idx);
      execution_monitor_sub_.shutdown();
    }
    monitor_mtx_.unlock();

    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO_NAMED(LOGNAME, "Finished trajectory execution monitoring.");
  updateObstacles();
}

void ContactPlanner::executeTrajectory() {
  moveit_msgs::MotionPlanResponse msg;
  plan_response_.getMessage(msg);
  std::size_t num_pts = msg.trajectory.joint_trajectory.points.size();
  ROS_INFO_NAMED(LOGNAME, "Executing trajectory with num pts: %ld", num_pts);

  trajectory_pub_.publish(msg.trajectory.joint_trajectory);
}

/** \brief Publishes the generated trajectory. The controller subscribes to
 * this topic.*/
ros::Publisher trajectory_pub_;

/** \brief The planner subsribes to this topic to monitor whether there are
 * any error during execution. If there are, then the planner needs to update
 * its scene information.*/
ros::Subscriber execution_monitor_sub_;

/** \brief Mutex for the monitor_msg_.*/
std::mutex monitor_mtx_;

/** \brief Each time the executionMonitorCallback gets called, it writes to
 * this member. The planner monitors this variable after execution, which is
 * why we use a mutex for this member.*/
tacbot::TrajExecutionMonitor monitor_msg_;

void ContactPlanner::updateObstacles() {
  if (monitor_msg_.is_valid != -1) {
    ROS_INFO_NAMED(LOGNAME,
                   "Trajectory execution was valid. Nothing to update.");
  }

  std::size_t error_link_num = 0;
  const double tau_thresh = 50.0;

  for (std::size_t i = 0; i < monitor_msg_.franka_state.q.size(); i++) {
    double tau_J = monitor_msg_.franka_state.tau_J[i];
    if (std::abs(tau_J) > tau_thresh) {
      error_link_num = i;
      break;
    }
  }

  moveit_msgs::MotionPlanResponse msg;
  plan_response_.getMessage(msg);

  trajectory_msgs::JointTrajectoryPoint point =
      msg.trajectory.joint_trajectory.points[monitor_msg_.pt_idx];
  std::vector<double> joint_angles(dof_, 0.0);
  for (std::size_t i = 0; i < dof_; i++) {
    joint_angles[i] = point.positions[i];
  }

  moveit::core::RobotStatePtr robot_state =
      std::make_shared<moveit::core::RobotState>(*robot_state_);
  robot_state->setJointGroupPositions(joint_model_group_, joint_angles);

  std::vector<std::vector<Eigen::Vector3d>> rob_pts;
  std::size_t num_pts = getPtsOnRobotSurface(robot_state, rob_pts);

  std::vector<Eigen::Vector3d> link_pts = rob_pts[error_link_num];
  std::size_t num_link_pts = link_pts.size();

  for (std::size_t i = 0; i < num_link_pts; i++) {
    sim_obstacle_pos_.emplace_back(link_pts[i]);
  }

  ROS_INFO_NAMED(LOGNAME,
                 "Finished updating obstacle positions based on trajectory "
                 "execution.");
}

/** \brief If the trajectory execution was not successful then update the
 * environment. This function add new obstacles to the environment at the
 * point where execution has been stopped. */
void updateObstacles();

/** \brief Visualize vertices consecutively for all robot states and
display
   their cost.
*/
void visualizeVertices();

/** \brief Visualize all the robot joint states that are sampled by a
planner.
 * Depending on the number of states, this visualization can quickly
 overwhelm
 * rviz and should be used sparingly.
 */
void visualizeTreeStates();

void Visualizer::visualizeManipVec(std::size_t state_num) {
  visualization_msgs::MarkerArray marker_array;

  if (vis_data_.repulsed_origin_at_link_.size() <= state_num ||
      vis_data_.manipulability_.size() <= state_num) {
    ROS_INFO_NAMED(
        LOGNAME,
        "Insufficient data stored to vizualize manipulability vectors.");
    return;
  }

  auto repulsed_origin_at_link = vis_data_.repulsed_origin_at_link_[state_num];
  std::vector<ManipulabilityMeasures> manip_per_joint =
      vis_data_.manipulability_[state_num];
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

      std::cout << "Press 'q' to exit this visualization or 'c' to go to the
                   "
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

  for (std::size_t i = 0; i < num_display_states; i += 50) {
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
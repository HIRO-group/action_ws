#include "contact_planner.h"

constexpr char LOGNAME[] = "contact_planner";

namespace pick_and_place {

ContactPlanner::ContactPlanner() {
  // fill goal position
  joint_goal_pos_ = std::vector<double>{-1.0, 0.7, 0.7, -1.0, -0.7, 2.0, 0.0};

  // fill obstacle positions
  contact_perception_ = std::make_shared<ContactPerception>();
  sim_obstacle_pos_.emplace_back(Eigen::Vector3d{0.4, 0.0, 0.6});
  sim_obstacle_pos_.emplace_back(Eigen::Vector3d{0.5, 0.1, 0.2});
  sim_obstacle_pos_.emplace_back(Eigen::Vector3d{0.5, 0.0, 0.5});
  sim_obstacle_pos_.emplace_back(Eigen::Vector3d{0.6, 0.2, 0.6});
}

void ContactPlanner::setCurToStartState(
    planning_interface::MotionPlanRequest& req) {
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));
  robot_state->setToDefaultValues(joint_model_group_, "ready");
  psm_->updateSceneWithCurrentState();

  req.start_state.joint_state.header.stamp = ros::Time::now();
  req.start_state.joint_state.name = joint_model_group_->getVariableNames();

  std::vector<double> start_joint_values;
  robot_state->copyJointGroupPositions(joint_model_group_, start_joint_values);
  req.start_state.joint_state.position = start_joint_values;

  start_joint_values.clear();
  robot_state->copyJointGroupVelocities(joint_model_group_, start_joint_values);
  req.start_state.joint_state.velocity = start_joint_values;

  start_joint_values.clear();
  robot_state->copyJointGroupAccelerations(joint_model_group_,
                                           start_joint_values);
  req.start_state.joint_state.effort = start_joint_values;
}

std::unique_ptr<ompl_interface::OMPLInterface> ContactPlanner::getOMPLInterface(
    const moveit::core::RobotModelConstPtr& model, const ros::NodeHandle& nh) {
  std::unique_ptr<ompl_interface::OMPLInterface> ompl_interface =
      std::make_unique<ompl_interface::OMPLInterface>(model, nh);
  return ompl_interface;
}

void ContactPlanner::setPlanningContextParams(
    ompl_interface::ModelBasedPlanningContextPtr& context) {
  unsigned int max_goal_samples_ = 10;
  unsigned int max_state_sampling_attempts_ = 4;
  unsigned int max_goal_sampling_attempts_ = 1000;
  unsigned int max_planning_threads_ = 4;
  double max_solution_segment_length_ = 0.0;
  unsigned int minimum_waypoint_count_ = 2;
  double goal_threshold_ = 0.1;
  bool simplify_solution_ = false;
  bool interpolate_ = true;
  bool hybridize_ = false;

  context->setMaximumPlanningThreads(max_planning_threads_);
  context->setMaximumGoalSamples(max_goal_samples_);
  context->setMaximumStateSamplingAttempts(max_state_sampling_attempts_);
  context->setMaximumGoalSamplingAttempts(max_goal_sampling_attempts_);
  context->setMaximumSolutionSegmentLength(max_solution_segment_length_);
  context->setMinimumWaypointCount(minimum_waypoint_count_);
  context->setGoalThreshold(goal_threshold_);
  context->simplifySolutions(simplify_solution_);
  context->setInterpolation(interpolate_);
  context->setHybridize(hybridize_);
}

planning_interface::PlannerConfigurationSettings
ContactPlanner::getPlannerConfigSettings(
    const planning_interface::PlannerConfigurationMap& pconfig_map,
    const std::string& planner_id) {
  auto pc = pconfig_map.end();
  pc = pconfig_map.find(planner_id);
  if (pc == pconfig_map.end()) {
    ROS_ERROR_NAMED(LOGNAME,
                    "Cannot find planning configuration for planner %s ",
                    planner_id.c_str());
    return planning_interface::PlannerConfigurationSettings();
  } else {
    ROS_INFO_NAMED(LOGNAME, "Found planning configuration for planner %s.",
                   planner_id.c_str());
  }
  return pc->second;
}

void ContactPlanner::createPlanningContext(
    const moveit_msgs::MotionPlanRequest& req, const ros::NodeHandle& nh) {
  planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);

  ompl_interface::ModelBasedStateSpaceSpecification space_spec(robot_model_,
                                                               group_name_);
  ompl_interface::ModelBasedStateSpacePtr state_space =
      std::make_shared<ompl_interface::JointModelStateSpace>(space_spec);
  state_space->computeLocations();

  std::unique_ptr<ompl_interface::OMPLInterface> ompl_interface =
      getOMPLInterface(robot_model_, nh);

  planning_interface::PlannerConfigurationMap pconfig_map =
      ompl_interface->getPlannerConfigurations();

  planning_interface::PlannerConfigurationSettings pconfig_settings =
      getPlannerConfigSettings(pconfig_map, req.planner_id);

  ompl_interface::PlanningContextManager planning_context_manager =
      ompl_interface->getPlanningContextManager();

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

  context_ = std::make_shared<ompl_interface::ModelBasedPlanningContext>(
      group_name_, context_spec);

  setPlanningContextParams(context_);

  context_->clear();

  moveit::core::RobotStatePtr start_state =
      lscene->getCurrentStateUpdated(req.start_state);

  context_->setPlanningScene(lscene);
  context_->setMotionPlanRequest(req);
  context_->setCompleteInitialState(*start_state);

  context_->setPlanningVolume(req.workspace_parameters);
  moveit_msgs::MoveItErrorCodes error_code;
  if (!context_->setPathConstraints(req.path_constraints, &error_code)) {
    ROS_ERROR_NAMED(LOGNAME, "context_->setPathConstraints() error: %d",
                    error_code.val);
    context_ = ompl_interface::ModelBasedPlanningContextPtr();
  }

  if (!context_->setGoalConstraints(req.goal_constraints, req.path_constraints,
                                    &error_code)) {
    ROS_ERROR_NAMED(LOGNAME, "context_->setGoalConstraints() error %d",
                    error_code.val);
    context_ = ompl_interface::ModelBasedPlanningContextPtr();
  }

  bool use_constraints_approximation = true;
  try {
    context_->configure(nh, use_constraints_approximation);
    ROS_INFO_NAMED(LOGNAME, "%s: New planning context_ is set.",
                   context_->getName().c_str());
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  } catch (ompl::Exception& ex) {
    ROS_ERROR_NAMED(LOGNAME, "OMPL encountered an error: %s", ex.what());
    context_.reset();
  }
}

ompl_interface::ModelBasedPlanningContextPtr
ContactPlanner::getPlanningContext() {
  return context_;
}

moveit_msgs::Constraints ContactPlanner::createJointGoal() {
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));
  moveit::core::RobotState goal_state(*robot_state);
  goal_state.setJointGroupPositions(joint_model_group_, joint_goal_pos_);
  double tolerance = 0.001;
  moveit_msgs::Constraints joint_goal =
      kinematic_constraints::constructGoalConstraints(
          goal_state, joint_model_group_, tolerance);
  return joint_goal;
}

Eigen::Vector3d ContactPlanner::scaleToDist(Eigen::Vector3d vec) {
  double y_max = 1.0;
  double D = 10.0;
  double dist_max = 0.5;
  // std::cout << "vec.norm() " << vec.norm() << std::endl;
  Eigen::Vector3d vec_out = Eigen::VectorXd::Zero(3);

  // if (vec.norm() > 0.4) {
  //   return vec_out;
  // }

  vec_out = (vec * y_max) / (D * vec.squaredNorm() + 1.0);

  // std::cout << "vec.squaredNorm() " << vec.squaredNorm() << std::endl;
  // std::cout << "before scale " << vec.transpose() << std::endl;
  // std::cout << "vec_out " << vec_out.transpose() << std::endl;
  return vec_out;
}

Eigen::MatrixXd ContactPlanner::getLinkPositions(
    moveit::core::RobotStatePtr robot_state) {
  std::vector<std::string> link_names = joint_model_group_->getLinkModelNames();
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  Eigen::MatrixXd positions(link_names.size(), 3);
  for (std::size_t i = 0; i < link_names.size(); i++) {
    const moveit::core::LinkModel* link_model =
        robot_state->getLinkModel(link_names[i]);
    tf = robot_state->getGlobalLinkTransform(link_model);
    auto translation = tf.translation();
    positions(i, 0) = (double)translation.x();
    positions(i, 1) = (double)translation.y();
    positions(i, 2) = (double)translation.z();
  }
  return positions;
}

void ContactPlanner::interpolateLinkPositions(Eigen::MatrixXd& mat) {
  std::size_t i = 1;
  while (i < mat.rows() - 1) {
    // std::cout << "i: " << i << std::endl;
    // std::cout << "mat.rows(): " << mat.rows() << std::endl;
    // std::cout << "mat.cols(): " << mat.cols() << std::endl;
    // std::cout << "mat\n: " << mat << std::endl;

    Eigen::Vector3d p1(mat(i - 1, 0), mat(i - 1, 1), mat(i - 1, 2));
    Eigen::Vector3d p2(mat(i, 0), mat(i, 1), mat(i, 2));
    double dist = utilities::getDistance(p1, p2);
    // std::cout << "p1: " << p1.transpose() << std::endl;
    // std::cout << "p2: " << p2.transpose() << std::endl;
    // std::cout << "dist: " << dist << std::endl;

    if (dist > 0.05) {
      Eigen::MatrixXd imat = Eigen::MatrixXd::Zero(mat.rows() + 1, 3);
      imat.topRows(i) = mat.topRows(i);
      imat.bottomRows(mat.rows() - i) = mat.bottomRows(mat.rows() - i);
      Eigen::Vector3d ivec((p2[0] + p1[0]) / 2.0, (p2[1] + p1[1]) / 2.0,
                           (p2[2] + p1[2]) / 2.0);
      // std::cout << "ivec: " << ivec.transpose() << std::endl;

      imat(i, 0) = ivec[0];
      imat(i, 1) = ivec[1];
      imat(i, 2) = ivec[2];
      // std::cout << "mat\n: " << mat << std::endl;
      // std::cout << "imat\n: " << imat << std::endl;
      mat = imat;
    } else {
      i++;
    }
  }
  // std::cout << "imat\n: " << mat << std::endl;
}

std::vector<std::size_t> ContactPlanner::findLinkIdx(
    const Eigen::MatrixXd& link_positions, const Eigen::MatrixXd& imat) {
  std::size_t link_idx = 0;
  std::vector<std::size_t> idx_out(link_positions.rows(), 0);

  for (std::size_t i = 0; i < imat.rows(); i++) {
    Eigen::Vector3d pos(link_positions(link_idx, 0),
                        link_positions(link_idx, 1),
                        link_positions(link_idx, 2));
    Eigen::Vector3d vec(imat(i, 0), imat(i, 1), imat(i, 2));
    double dist = utilities::getDistance(pos, vec);
    if (dist < 0.0001) {
      idx_out[link_idx] = i;
      // std::cout << "findLinkIdx i: " << i << std::endl;
      link_idx++;
    }
  }
  // std::cout << "link_positions.rows(): " << link_positions.rows() <<
  // std::endl; std::cout << "idx_out.size(): " << idx_out.size() << std::endl;

  return idx_out;
}

std::vector<Eigen::Vector3d> ContactPlanner::getObstacles(
    const Eigen::Vector3d& pt_on_rob) {
  std::vector<Eigen::Vector3d> obstacles;
  if (use_sim_obstacles_) {
    vis_data_.saveObstaclePos(sim_obstacle_pos_, sample_state_count_);
    return sim_obstacle_pos_;
  }

  bool status = contact_perception_->extractNearPts(pt_on_rob, obstacles);
  if (status) {
    vis_data_.saveObstaclePos(obstacles, sample_state_count_);
    return obstacles;
  }

  // Store an empty obstacle vector when no obstacles have been found in
  // proximity. This ensures that the size of the stored obstacles in an array
  // are equal to the number of states that we considered.
  vis_data_.saveObstaclePos(std::vector<Eigen::Vector3d>{},
                            sample_state_count_);

  // This yields a zero repulsion vector and will mean no repulsion will be
  // applied by the vectors filed.
  obstacles.emplace_back(pt_on_rob);
  return obstacles;
}

std::vector<Eigen::Vector3d> ContactPlanner::getLinkToObsVec(
    const Eigen::MatrixXd& rob_pts,
    const std::vector<std::size_t>& link_idx_arr) {
  std::size_t num_links = link_idx_arr.size();
  std::size_t num_rob_pts = rob_pts.rows();

  // std::cout << "num_links: " << num_links << std::endl;
  // std::cout << "num_rob_pts: " << num_rob_pts << std::endl;

  std::vector<Eigen::Vector3d> link_to_obs_vec(num_links,
                                               Eigen::Vector3d::Zero(3));

  for (std::size_t i = 1; i < num_links; i++) {
    std::size_t link1_idx = link_idx_arr[i - 1];
    std::size_t link2_idx = link_idx_arr[i];
    // std::cout << "i: " << i << std::endl;
    // std::cout << "link1_idx: " << link1_idx << std::endl;
    // std::cout << "link2_idx: " << link2_idx << std::endl;

    Eigen::MatrixXd pts_link_vec =
        Eigen::MatrixXd::Zero(link2_idx - link1_idx, 3);

    for (std::size_t j = link1_idx; j < link2_idx; j++) {
      Eigen::Vector3d pt_on_rob(rob_pts(j, 0), rob_pts(j, 1), rob_pts(j, 2));
      // std::cout << "j: " << j << std::endl;

      std::vector<Eigen::Vector3d> obstacle_pos = getObstacles(pt_on_rob);

      std::size_t num_obstacles = obstacle_pos.size();

      Eigen::MatrixXd pt_to_obs = Eigen::MatrixXd::Zero(num_obstacles, 3);
      for (std::size_t k = 0; k < num_obstacles; k++) {
        Eigen::Vector3d vec = pt_on_rob - obstacle_pos[k];
        vec = scaleToDist(vec);

        pt_to_obs(k, 0) = vec[0];
        pt_to_obs(k, 1) = vec[1];
        pt_to_obs(k, 2) = vec[2];
      }

      double av_x = pt_to_obs.col(0).mean();
      double av_y = pt_to_obs.col(1).mean();
      double av_z = pt_to_obs.col(2).mean();
      Eigen::Vector3d pt_to_obs_av(av_x, av_y, av_z);

      pts_link_vec(link2_idx - j - 1, 0) = pt_to_obs_av[0];
      pts_link_vec(link2_idx - j - 1, 1) = pt_to_obs_av[1];
      pts_link_vec(link2_idx - j - 1, 2) = pt_to_obs_av[2];

      // std::cout << "pt_to_obs_av: " << pt_to_obs_av.transpose() << std::endl;
      vis_data_.saveOriginVec(pt_on_rob, pt_to_obs_av, num_rob_pts, j,
                              sample_state_count_);
    }
    // std::cout << "pts_link_vec.rows(): " << pts_link_vec.rows() << std::endl;
    double av_x = pts_link_vec.col(0).mean();
    double av_y = pts_link_vec.col(1).mean();
    double av_z = pts_link_vec.col(2).mean();
    Eigen::Vector3d to_obs_avg(av_x, av_y, av_z);
    // std::cout << "to_obs_avg: " << to_obs_avg.transpose() << std::endl;

    link_to_obs_vec[i - 1] = to_obs_avg;
  }
  return link_to_obs_vec;
}

Eigen::VectorXd ContactPlanner::obstacleField(
    const ompl::base::State* base_state) {
  const ompl::base::RealVectorStateSpace::StateType& vec_state =
      *base_state->as<ompl::base::RealVectorStateSpace::StateType>();
  std::vector<double> joint_angles = utilities::toStlVec(vec_state, dof_);

  moveit::core::RobotStatePtr robot_state =
      std::make_shared<moveit::core::RobotState>(*robot_state_);
  robot_state->setJointGroupPositions(joint_model_group_, joint_angles);
  Eigen::MatrixXd rob_pts = getLinkPositions(robot_state);

  interpolateLinkPositions(rob_pts);
  // std::cout << "rob_pts\n: " << rob_pts << std::endl;
  // std::cout << "num_rob_pts: " << num_rob_pts << std::endl;

  std::vector<std::size_t> link_idx_arr =
      findLinkIdx(getLinkPositions(robot_state), rob_pts);

  std::vector<Eigen::Vector3d> link_to_obs_vec =
      getLinkToObsVec(rob_pts, link_idx_arr);

  Eigen::MatrixXd jacobian = robot_state->getJacobian(joint_model_group_);

  // std::vector<ManipulabilityMeasures> manip_per_joint;

  // std::cout << "jacobian:\n " << jacobian << std::endl;
  // Eigen::VectorXd d_q_out = toEigen(joint_angles);
  Eigen::VectorXd d_q_out = Eigen::VectorXd::Zero(dof_);
  for (std::size_t i = 0; i < dof_; i++) {
    Eigen::MatrixXd link_jac = jacobian.block(0, 0, 3, i + 1);
    // std::cout << "link_jac:\n " << link_jac << std::endl;

    Eigen::MatrixXd jac_pinv_;
    utilities::pseudoInverse(link_jac, jac_pinv_);
    // std::cout << "jac_pinv_:\n " << jac_pinv_ << std::endl;
    Eigen::Vector3d vec = link_to_obs_vec[i];
    // std::cout << "vec:\n " << vec << std::endl;
    Eigen::Vector3d rob_vec =
        utilities::toEigen(std::vector<double>{vec[0], vec[1], vec[2]});

    Eigen::MatrixXd j_t = link_jac.transpose();
    Eigen::MatrixXd j_sq = link_jac * j_t;

    // Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(j_sq);
    // ManipulabilityMeasures manip;
    // manip.eigen_values = eigensolver.eigenvalues().real();
    // manip.eigen_vectors = eigensolver.eigenvectors().real();

    // bool is_valid = false;
    // for (std::size_t i = 0; i < manip.eigen_values.size(); i++) {
    //   double eig_val = manip.eigen_values(i);
    //   // std::cout << "eig_val " << eig_val << std::endl;
    //   Eigen::Vector3d eig_vec = manip.getVector(i);
    //   // double ab = eig_vec.dot(rob_vec);
    //   // double aa = eig_vec.dot(eig_vec);
    //   // double bb = rob_vec.dot(rob_vec);

    //   double sim1 = std::pow(eig_vec.dot(rob_vec), 2) /
    //                 (eig_vec.dot(eig_vec) * rob_vec.dot(rob_vec));
    //   double sim2 = std::pow(eig_vec.dot(-1 * rob_vec), 2) /
    //                 (eig_vec.dot(eig_vec) * -1 * rob_vec.dot(-1 * rob_vec));
    //   double sim = std::max(sim1, sim2);
    //   std::cout << "eig_vec " << eig_vec.transpose() << std::endl;
    //   std::cout << "rob_vec " << rob_vec.transpose() << std::endl;
    //   std::cout << "sim " << sim << std::endl;

    //   const double VAL_THRESH = 0.2;
    //   const double SIM_THRESH = 0.2;
    //   if (eig_val > VAL_THRESH && sim > SIM_THRESH) {
    //     std::cout << "thresh passed" << std::endl;
    //     is_valid = true;
    //   }
    // }

    // is_valid = true;
    // manip.pass = is_valid;
    // manip_per_joint.emplace_back(manip);

    // double repulse_angle = 0.0;
    // if (is_valid) {
    Eigen::VectorXd d_q = jac_pinv_ * rob_vec;
    // std::cout << "d_q:\n " << d_q << std::endl;
    // repulse_angle = d_q[i];
    // }

    d_q_out[i] = d_q[i];
  }

  // d_q_out = d_q_out * 0.5;
  // d_q_out.normalize();
  // std::cout << "d_q_out:\n " << d_q_out.transpose() << std::endl;

  // manipulability_.emplace_back(manip_per_joint);
  vis_data_.saveRepulseAngles(joint_angles, d_q_out);
  sample_state_count_++;
  return d_q_out;
}

Eigen::VectorXd ContactPlanner::goalField(const ompl::base::State* state) {
  const ompl::base::RealVectorStateSpace::StateType& x =
      *state->as<ompl::base::RealVectorStateSpace::StateType>();
  Eigen::VectorXd v(7);
  v[0] = joint_goal_pos_[0] - x[0];
  v[1] = joint_goal_pos_[1] - x[1];
  v[2] = joint_goal_pos_[2] - x[2];
  v[3] = joint_goal_pos_[3] - x[3];
  v[4] = joint_goal_pos_[4] - x[4];
  v[5] = joint_goal_pos_[5] - x[5];
  v[6] = joint_goal_pos_[6] - x[6];
  v.normalize();
  return v;
}

Eigen::VectorXd ContactPlanner::negGoalField(const ompl::base::State* state) {
  const ompl::base::RealVectorStateSpace::StateType& x =
      *state->as<ompl::base::RealVectorStateSpace::StateType>();
  Eigen::VectorXd v(7);
  v[0] = x[0] - joint_goal_pos_[0];
  v[1] = x[1] - joint_goal_pos_[1];
  v[2] = x[2] - joint_goal_pos_[2];
  v[3] = x[3] - joint_goal_pos_[3];
  v[4] = x[4] - joint_goal_pos_[4];
  v[5] = x[5] - joint_goal_pos_[5];
  v[6] = x[6] - joint_goal_pos_[6];
  v.normalize();
  return v;
}

Eigen::VectorXd ContactPlanner::totalField(const ompl::base::State* state) {
  const ompl::base::RealVectorStateSpace::StateType& x =
      *state->as<ompl::base::RealVectorStateSpace::StateType>();
  Eigen::VectorXd goal_vec = goalField(state);
  std::cout << "\ngoal_vec\n" << goal_vec.transpose() << std::endl;
  Eigen::VectorXd obstacle_vec = obstacleField(state);
  std::cout << "obstacle_vec\n" << obstacle_vec.transpose() << std::endl;
  Eigen::VectorXd total_vec = goal_vec + obstacle_vec;
  std::cout << "total_vec\n" << total_vec.transpose() << std::endl;
  total_vec.normalize();
  return total_vec;
}

void ContactPlanner::changePlanner() {
  ompl::geometric::SimpleSetupPtr simple_setup = context_->getOMPLSimpleSetup();
  ompl::base::SpaceInformationPtr si = simple_setup->getSpaceInformation();

  ompl::geometric::PathSimplifierPtr simplifier =
      simple_setup->getPathSimplifier();

  simplifier->setSimplificationType(
      ompl::geometric::SimplificationType::SMOOTH_COST);

  std::function<Eigen::VectorXd(const ompl::base::State*)> vFieldFunc =
      std::bind(&ContactPlanner::obstacleField, this, std::placeholders::_1);

  // double exploration = 0.5;
  // double initial_lambda = 0.01;
  // unsigned int update_freq = 30;
  // ompl::base::PlannerPtr planner = std::make_shared<ompl::geometric::CVFRRT>(
  //     si, vFieldFunc, exploration, initial_lambda, update_freq);

  simple_setup->setOptimizationObjective(
      std::make_shared<ompl::base::VFUpstreamCriterionOptimizationObjective>(
          si, vFieldFunc));

  ompl::base::PlannerPtr planner =
      std::make_shared<ompl::geometric::ClassicTRRT>(
          simple_setup->getSpaceInformation());

  simple_setup->setPlanner(planner);
}

bool ContactPlanner::generatePlan(planning_interface::MotionPlanResponse& res) {
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  bool is_solved = context_->solve(res);
  if (is_solved) {
    std::size_t state_count = res.trajectory_->getWayPointCount();
    ROS_DEBUG_STREAM("Motion planner reported a solution path with "
                     << state_count << " states");
  }
  ROS_INFO_NAMED(LOGNAME, "Is plan generated? %d", is_solved);
  return is_solved;
}

moveit_msgs::Constraints ContactPlanner::createPoseGoal() {
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

void ContactPlanner::init() {
  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      "robot_description");

  robot_model_ = robot_model_loader_->getModel();

  psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      robot_model_loader_);

  psm_->publishDebugInformation(true);

  ROS_INFO_NAMED(LOGNAME, "startSceneMonitor");
  psm_->startSceneMonitor();

  ROS_INFO_NAMED(LOGNAME, "startWorldGeometryMonitor");
  psm_->startWorldGeometryMonitor();

  ROS_INFO_NAMED(LOGNAME, "startStateMonitor");
  psm_->startStateMonitor();

  ROS_INFO_NAMED(LOGNAME, "startWorldGeometryMonitor");
  psm_->startWorldGeometryMonitor(planning_scene_monitor::PlanningSceneMonitor::
                                      DEFAULT_COLLISION_OBJECT_TOPIC,
                                  planning_scene_monitor::PlanningSceneMonitor::
                                      DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
                                  false /* skip octomap monitor */);

  ROS_INFO_NAMED(LOGNAME, "startPublishingPlanningScene");
  psm_->startPublishingPlanningScene(
      planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  ROS_INFO_NAMED(LOGNAME, "providePlanningSceneService");
  psm_->providePlanningSceneService();

  kinematics_metrics_ = std::make_shared<kinematics_metrics::KinematicsMetrics>(
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getRobotModel());

  planning_scene_monitor::CurrentStateMonitorPtr csm = psm_->getStateMonitor();
  csm->enableCopyDynamics(true);

  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
      planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState()));

  joint_model_group_ = robot_model_->getJointModelGroup(group_name_);

  robot_state->setToDefaultValues(joint_model_group_, "ready");
  psm_->updateSceneWithCurrentState();

  // ROS_INFO_NAMED(LOGNAME, "Robot State Positions");
  // robot_state->printStatePositions();
  robot_state_ = std::make_shared<moveit::core::RobotState>(*robot_state);

  contact_perception_->init();
}

std::string ContactPlanner::getGroupName() { return group_name_; }

std::vector<Eigen::Vector3d> ContactPlanner::getSimObstaclePos() {
  return sim_obstacle_pos_;
};

}  // namespace pick_and_place
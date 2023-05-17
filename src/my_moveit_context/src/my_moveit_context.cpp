#include "my_moveit_context.h"

constexpr char LOGNAME[] = "my_moveit_context";

MyMoveitContext::MyMoveitContext(
    const planning_scene_monitor::PlanningSceneMonitorPtr& psm,
    const moveit::core::RobotModelPtr& robot_model)
    : psm_(psm), robot_model_(robot_model) {}

void MyMoveitContext::createPlanningContext(
    const moveit_msgs::MotionPlanRequest& req) {
  planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);

  ompl_interface::ModelBasedStateSpaceSpecification space_spec(robot_model_,
                                                               group_name_);
  ompl_interface::ModelBasedStateSpacePtr state_space =
      std::make_shared<ompl_interface::JointModelStateSpace>(space_spec);
  state_space->computeLocations();

  std::unique_ptr<ompl_interface::OMPLInterface> ompl_interface =
      getOMPLInterface(robot_model_, nh_);

  planning_interface::PlannerConfigurationMap pconfig_map =
      ompl_interface->getPlannerConfigurations();

  planning_interface::PlannerConfigurationSettings pconfig_settings =
      getPlannerConfigSettings(pconfig_map, planner_id_);

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
    context_->configure(nh_, use_constraints_approximation);
    ROS_INFO_NAMED(LOGNAME, "%s: New planning context_ is set.",
                   context_->getName().c_str());
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  } catch (ompl::Exception& ex) {
    ROS_ERROR_NAMED(LOGNAME, "OMPL encountered an error: %s", ex.what());
    context_.reset();
  }
}

ompl_interface::ModelBasedPlanningContextPtr
MyMoveitContext::getPlanningContext() {
  return context_;
}

void MyMoveitContext::setPlanningContextParams(
    ompl_interface::ModelBasedPlanningContextPtr& context) {
  unsigned int max_goal_samples_ = 10;
  unsigned int max_state_sampling_attempts_ = 4;
  unsigned int max_goal_sampling_attempts_ = 1000;
  unsigned int max_planning_threads_ = 4;
  double max_solution_segment_length_ = 0.0;
  unsigned int minimum_waypoint_count_ = 30;
  double goal_threshold_ = 0.01;
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

std::unique_ptr<ompl_interface::OMPLInterface>
MyMoveitContext::getOMPLInterface(const moveit::core::RobotModelConstPtr& model,
                                  const ros::NodeHandle& nh) {
  std::unique_ptr<ompl_interface::OMPLInterface> ompl_interface =
      std::make_unique<ompl_interface::OMPLInterface>(model, nh);
  return ompl_interface;
}

planning_interface::PlannerConfigurationSettings
MyMoveitContext::getPlannerConfigSettings(
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

std::string MyMoveitContext::getPlannerId() { return planner_id_; }

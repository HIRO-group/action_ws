#ifndef MY_MOVEIT_CONTEXT_H
#define MY_MOVEIT_CONTEXT_H

#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <ros/ros.h>

class MyMoveitContext {
 public:
  void createPlanningContext(const moveit_msgs::MotionPlanRequest& req);
  ompl_interface::ModelBasedPlanningContextPtr getPlanningContext();

 private:
  ros::NodeHandle nh_;

  /** \brief Default robot being used.*/
  const std::string group_name_ = "panda_arm";

  /** \brief Default planner set to the planning context.*/
  std::string planner_id_ = "panda_arm[RRT]";

  planning_scene_monitor::PlanningSceneMonitorPtr psm_;

  moveit::core::RobotModelPtr robot_model_;

  ompl_interface::ModelBasedPlanningContextPtr context_;

  void setPlanningContextParams(
      ompl_interface::ModelBasedPlanningContextPtr& context);

  std::unique_ptr<ompl_interface::OMPLInterface> getOMPLInterface(
      const moveit::core::RobotModelConstPtr& model, const ros::NodeHandle& nh);

  planning_interface::PlannerConfigurationSettings getPlannerConfigSettings(
      const planning_interface::PlannerConfigurationMap& pconfig_map,
      const std::string& planner_id);
};

#endif
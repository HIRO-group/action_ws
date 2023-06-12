#ifndef MY_MOVEIT_CONTEXT_H
#define MY_MOVEIT_CONTEXT_H

#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <ros/ros.h>

/**
     @anchor MyMoveitContext

     @par Short description
     This is a class that holds the planning configuration of you
   planner,optimizer, problem definition, etc. Usually, this class is nested
   within the motion planning pipeline class and cannot be configured. Here, we
   extract all the relevant setup and parameter setup for maximum customization.
*/

class MyMoveitContext {
 public:
  MyMoveitContext(const planning_scene_monitor::PlanningSceneMonitorPtr& psm,
                  const moveit::core::RobotModelPtr& robot_model);

  void createPlanningContext(const moveit_msgs::MotionPlanRequest& req);

  ompl_interface::ModelBasedPlanningContextPtr getPlanningContext();

  /** \brief Get the planner that is set into the context at class
    initialization. This will be RRT or some other default class that is native
    to the moveit envrionment. This class will be used for planner generation
    unless changePlanner is called.
    @return string of the planner name. refer to ompl_planning.yaml for a full
    list of available planner.
  */
  std::string getPlannerId();

  void setSimplifySolution(bool simplify_solution);

 private:
  ros::NodeHandle nh_;

  /** \brief Default robot being used.*/
  const std::string group_name_ = "panda_arm";

  /** \brief Default planner set to the planning context.*/
  std::string planner_id_ = "panda_arm[RRTConnect]";

  bool simplify_solution_ = true;

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
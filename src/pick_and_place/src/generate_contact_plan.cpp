#include "contact_planner.h"

constexpr char LOGNAME[] = "generate_plan";

int main(int argc, char** argv) {
  ros::init(argc, argv, "pick_and_place_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  ROS_DEBUG_NAMED(LOGNAME, "Start!");

  ContactPlanner c_planner;
  c_planner.init();

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  c_planner.setCurToStartState(req);

  moveit_msgs::Constraints goal = c_planner.createJointGoal();
  req.goal_constraints.push_back(goal);

  req.group_name = c_planner.getGroupName();
  req.allowed_planning_time = 10.0;
  req.planner_id = "panda_arm[RRT]";

  ompl_interface::ModelBasedPlanningContextPtr context =
      c_planner.createPlanningContext(req, node_handle);

  c_planner.changePlanner(context);

  c_planner.generatePlan(context, res);

  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully. Error code: %d",
              res.error_code_.val);
    // return 0;
  }

  ROS_INFO_NAMED(LOGNAME, "Visualizing goal state.");
  c_planner.visualizeGoalState();

  ROS_INFO_NAMED(LOGNAME, "Visualizing repulsed states.");
  c_planner.visualizeRepulsedState();

  // ROS_INFO_NAMED(LOGNAME, "Visualizing all states in the tree.");
  // c_planner.visualizeTreeStates();

  ROS_INFO_NAMED(LOGNAME, "Visualizing trajectory.");
  c_planner.visualizeTrajectory(res);

  c_planner.promptAnyInput();
  std::cout << "Finished!" << std::endl;

  return 0;
}
#include "contact_controller.h"
#include "contact_perception.h"
#include "contact_planner.h"
#include "utilities.h"

constexpr char LOGNAME[] = "generate_plan";

using namespace pick_and_place;

int main(int argc, char** argv) {
  ros::init(argc, argv, "generate_contact_plan");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  ROS_DEBUG_NAMED(LOGNAME, "Start!");

  std::shared_ptr<ContactPlanner> contact_planner =
      std::make_shared<ContactPlanner>();
  contact_planner->init();

  std::shared_ptr<Visualizer> visualizer = std::make_shared<Visualizer>();
  visualizer->visualizeObstacleMarker(contact_planner->getSimObstaclePos());
  visualizer->setContactPlanner(contact_planner);
  visualizer->visualizeGoalState();

  // visually confirm that the planning scene, obstacles, goal state are all
  // correct
  utilities::promptAnyInput();

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  contact_planner->setCurToStartState(req);

  moveit_msgs::Constraints goal = contact_planner->createJointGoal();
  req.goal_constraints.push_back(goal);

  req.group_name = contact_planner->getGroupName();
  req.allowed_planning_time = 1.0;
  req.planner_id = contact_planner->getDefaultPlannerId();
  req.max_acceleration_scaling_factor = 0.1;
  req.max_velocity_scaling_factor = 0.1;

  contact_planner->createPlanningContext(req);

  // contact_planner->changePlanner();

  contact_planner->generatePlan(res);

  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully. Error code: %d",
              res.error_code_.val);
  }

  ROS_INFO_NAMED(LOGNAME, "Visualizing repulsed states.");
  visualizer->visualizeRepulsedState();

  // ROS_INFO_NAMED(LOGNAME, "Visualizing all states in the tree.");
  // visualizer->visualizeTreeStates();
  // utilities::promptAnyInput();

  if (res.error_code_.val == res.error_code_.SUCCESS) {
    ROS_INFO_NAMED(LOGNAME, "Visualizing trajectory.");
    visualizer->visualizeTrajectory(res, "planned_path");
    utilities::promptAnyInput();
  }

  if (res.error_code_.val == res.error_code_.SUCCESS) {
    ROS_INFO_NAMED(LOGNAME, "Executing trajectory.");
    contact_planner->executeTrajectory();
    contact_planner->monitorExecution();
    visualizer->visualizeObstacleMarker(contact_planner->getSimObstaclePos());

    utilities::promptAnyInput();
  }
  std::cout << "Finished!" << std::endl;

  return 0;
}
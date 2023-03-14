#include "contact_planner.h"
#include "my_moveit_context.h"
#include "panda_interface.h"
#include "utilities.h"
#include "visualizer.h"

constexpr char LOGNAME[] = "generate_plan";

using namespace tacbot;

int main(int argc, char** argv) {
  ros::init(argc, argv, "generate_contact_plan");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  ROS_DEBUG_NAMED(LOGNAME, "Start!");

  std::shared_ptr<ContactPlanner> contact_planner =
      std::make_shared<ContactPlanner>();
  contact_planner->init();

  std::shared_ptr<Visualizer> visualizer =
      std::make_shared<Visualizer>(contact_planner->getVisualizerData());

  std::shared_ptr<MyMoveitContext> context = std::make_shared<MyMoveitContext>(
      contact_planner->getPlanningSceneMonitor(),
      contact_planner->getRobotModel());

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  contact_planner->setCurToStartState(req);

  moveit_msgs::Constraints goal = contact_planner->createJointGoal();
  req.goal_constraints.push_back(goal);

  visualizer->visualizeGoalState(contact_planner->getJointNames(),
                                 contact_planner->getJointGoalPos());

  req.group_name = contact_planner->getGroupName();
  req.allowed_planning_time = 30.0;
  req.planner_id = context->getPlannerId();
  req.max_acceleration_scaling_factor = 0.5;
  req.max_velocity_scaling_factor = 0.5;

  context->createPlanningContext(req);

  contact_planner->setPlanningContext(context->getPlanningContext());

  contact_planner->generatePlan(res);

  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully. Error code: %d",
              res.error_code_.val);
    return 1;
  }

  moveit_msgs::MotionPlanResponse msg;
  res.getMessage(msg);
  visualizer->visualizeTrajectory(msg, "planned_path");

  utilities::promptUserInput();

  bool execute_trajectory = false;
  if (execute_trajectory == true) {
    PandaInterface panda_interface;
    panda_interface.init();
    panda_interface.move_to_default_pose(panda_interface.robot_.get());

    moveit_msgs::MotionPlanResponse traj_msg;
    res.getMessage(traj_msg);
    std::vector<std::array<double, 7>> joint_waypoints;
    std::vector<std::array<double, 7>> joint_velocities;
    utilities::toControlTrajectory(traj_msg, joint_waypoints, joint_velocities);

    panda_interface.follow_joint_velocities(panda_interface.robot_.get(),
                                            joint_velocities);
  }

  std::cout << "Finished!" << std::endl;

  return 0;
}
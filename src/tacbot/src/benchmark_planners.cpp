#include "contact_planner.h"
#include "utilities.h"

constexpr char LOGNAME[] = "generate_plan";

using namespace tacbot;

struct BenchMarkData {
  std::size_t test_num = 0;
  std::size_t success = 0;
  double plan_time = 0.0;
  std::string file_name = "";
  PlanAnalysisData plan_analysis;
};

void saveData(const BenchMarkData& benchmark_data) {
  std::fstream file(benchmark_data.file_name, std::ios::out | std::ios::app);
  if (file.is_open()) {
    file << benchmark_data.test_num;
    file << ",";
    file << benchmark_data.success;
    file << ",";
    file << benchmark_data.plan_time;
    file << ",";
    file << benchmark_data.plan_analysis.total_contact_count;
    file << ",";
    file << benchmark_data.plan_analysis.num_contact_states;
    file << ",";
    file << benchmark_data.plan_analysis.num_path_states;
    file << ",";
    file << benchmark_data.plan_analysis.total_contact_depth;
    file << ",";
    file << benchmark_data.plan_analysis.joint_path_len;
    file << ",";
    file << benchmark_data.plan_analysis.ee_path_len;
    file << "\n";
    file.close();
  } else {
    OMPL_ERROR("Unable to open file for writing.");
  }
}

void initDataFile(const BenchMarkData& benchmark_data) {
  std::fstream file(benchmark_data.file_name, std::ios::out | std::ios::trunc);

  if (file.is_open()) {
    file << "test_num";
    file << ",";
    file << "success";
    file << ",";
    file << "plan_time";
    file << ",";
    file << "total_contact_count";
    file << ",";
    file << "num_contact_states";
    file << ",";
    file << "num_path_states";
    file << ",";
    file << "total_contact_depth";
    file << ",";
    file << "joint_path_len";
    file << ",";
    file << "ee_path_len";
    file << "\n";
    file.close();
  } else {
    OMPL_ERROR("Unable to open file for writing.");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "benchmark_planners");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  ROS_INFO_NAMED(LOGNAME, "Start!");

  const std::size_t NUM_PLANNING_ATTEMPTS = 10;
  const std::size_t MAX_PLANNING_TIME = 10;

  const std::string PLANNER_NAME = "ContactTRRTDuo";
  const std::string OBJECTIVE_NAME =
      "FieldAlign";  // FieldMagnitude or UpstreamCost or FieldAlign
  const std::size_t OBSTACLE_SCENE_OPT = 1;
  const std::size_t GOAL_STATE_OPT = 1;

  BenchMarkData benchmark_data;
  std::string test_name = PLANNER_NAME + "_" + OBJECTIVE_NAME + "_" + "OBST_" +
                          std::to_string(OBSTACLE_SCENE_OPT) + "_" + "GOAL_" +
                          std::to_string(GOAL_STATE_OPT);
  std::string file_path = "/home/nn/action_ws/src/tacbot/scripts/";
  benchmark_data.file_name = file_path + test_name + ".csv";

  initDataFile(benchmark_data);

  for (std::size_t i = 0; i < NUM_PLANNING_ATTEMPTS; i++) {
    std::shared_ptr<ContactPlanner> contact_planner =
        std::make_shared<ContactPlanner>();
    contact_planner->init();
    contact_planner->setObstacleScene(OBSTACLE_SCENE_OPT);
    contact_planner->setGoalState(GOAL_STATE_OPT);

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    contact_planner->setCurToStartState(req);

    moveit_msgs::Constraints goal = contact_planner->createJointGoal();
    req.goal_constraints.push_back(goal);

    req.group_name = contact_planner->getGroupName();
    req.allowed_planning_time = MAX_PLANNING_TIME;
    req.planner_id = contact_planner->getDefaultPlannerId();
    req.max_acceleration_scaling_factor = 0.1;
    req.max_velocity_scaling_factor = 0.1;

    contact_planner->createPlanningContext(req);
    contact_planner->changePlanner(PLANNER_NAME, OBJECTIVE_NAME);
    contact_planner->generatePlan(res);

    if (res.error_code_.val != res.error_code_.SUCCESS) {
      ROS_ERROR("Could not compute plan successfully. Error code: %d",
                res.error_code_.val);
      benchmark_data.success = 0;
    } else {
      benchmark_data.success = 1;
    }

    benchmark_data.plan_time = res.planning_time_;
    benchmark_data.test_num = i + 1;

    PlanAnalysisData plan_analysis;
    contact_planner->analyzePlanResponse(plan_analysis);
    benchmark_data.plan_analysis = plan_analysis;

    saveData(benchmark_data);
  }

  ROS_INFO_NAMED(LOGNAME, "Finished!");

  return 0;
}
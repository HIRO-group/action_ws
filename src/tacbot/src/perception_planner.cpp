#include "perception_planner.h"

using namespace std::chrono;
constexpr char LOGNAME[] = "contact_planner";

namespace tacbot {

PerceptionPlanner::PerceptionPlanner() : BasePlanner() {
  ROS_INFO_NAMED(LOGNAME, "contact_perception_->init()");
  contact_perception_->init();
}

}  // namespace tacbot
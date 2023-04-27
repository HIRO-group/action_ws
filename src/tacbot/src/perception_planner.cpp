#include "perception_planner.h"

#include <ompl/multilevel/planners/qrrt/QRRTStar.h>

using namespace std::chrono;
constexpr char LOGNAME[] = "perception_planner";

namespace tacbot {

PerceptionPlanner::PerceptionPlanner() : BasePlanner() {
  ROS_INFO_NAMED(LOGNAME, "contact_perception_->init()");
  contact_perception_ = std::make_shared<ContactPerception>();
  contact_perception_->init();
}

void PerceptionPlanner::changePlanner() {
  ROS_INFO_NAMED(LOGNAME, "context_->getOMPLSimpleSetup()");
  ompl::geometric::SimpleSetupPtr simple_setup = context_->getOMPLSimpleSetup();
  ROS_INFO_NAMED(LOGNAME, "simple_setup->getSpaceInformation()");
  ompl::base::SpaceInformationPtr si = simple_setup->getSpaceInformation();
  ROS_INFO_NAMED(LOGNAME, "std::make_shared<ompl::multilevel::QRRTStar>(si)");
  std::shared_ptr<ompl::multilevel::QRRTStar> planner =
      std::make_shared<ompl::multilevel::QRRTStar>(si);
}

}  // namespace tacbot
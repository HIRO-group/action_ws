#include "perception_planner.h"

#include <ompl/multilevel/planners/qrrt/QRRTStar.h>

using namespace std::chrono;
constexpr char LOGNAME[] = "contact_planner";

namespace tacbot {

PerceptionPlanner::PerceptionPlanner() : BasePlanner() {
  ROS_INFO_NAMED(LOGNAME, "contact_perception_->init()");
  contact_perception_->init();
}

void PerceptionPlanner::changePlanner() {
  ompl::geometric::SimpleSetupPtr simple_setup = context_->getOMPLSimpleSetup();
  ompl::base::SpaceInformationPtr si = simple_setup->getSpaceInformation();

  std::shared_ptr<ompl::multilevel::QRRTStar> planner =
      std::make_shared<ompl::multilevel::QRRTStar>(si);
}

}  // namespace tacbot
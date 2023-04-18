#ifndef TACBOT_PERCEPTION_PLANNER_H
#define TACBOT_PERCEPTION_PLANNER_H

#include "base_planner.h"
#include "contact_perception.h"

namespace tacbot {

class PerceptionPlanner : public BasePlanner {
 public:
  PerceptionPlanner();

 protected:
  /** \brief The class that handles point cloud processing of the surrounding
   * environment and transfers this information to the planner.*/
  std::shared_ptr<ContactPerception> contact_perception_;
};
}  // namespace tacbot
#endif
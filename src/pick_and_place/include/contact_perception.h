#ifndef CONTACT_PERCEPTION_H
#define CONTACT_PERCEPTION_H

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit_msgs/CollisionObject.h>

namespace pick_and_place {
class ContactPerception {
 public:
  void init();

 private:
  ros::NodeHandle nh_;
  ros::Publisher collision_object_pub_;
};
}  // namespace pick_and_place

#endif
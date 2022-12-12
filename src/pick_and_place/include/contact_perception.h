#ifndef CONTACT_PERCEPTION_H
#define CONTACT_PERCEPTION_H

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ObjectColor.h>

namespace pick_and_place {
class ContactPerception {
 public:
  void init();

  void addCollisionObjects(
      const std::vector<moveit_msgs::CollisionObject>& collision_objects,
      const std::vector<moveit_msgs::ObjectColor>& object_colors);

 private:
  ros::NodeHandle nh_;
  ros::Publisher planning_scene_diff_publisher_;
};
}  // namespace pick_and_place

#endif
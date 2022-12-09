#include "contact_perception.h"

namespace pick_and_place {

void ContactPerception::init() {
  collision_object_pub_ = nh_.advertise<moveit_msgs::CollisionObject>(
      "collision_object", 1024, true);

  moveit_msgs::CollisionObject safety_perimeter;
  safety_perimeter.header.frame_id = "panda_link0";
  safety_perimeter.id = "safety_perimeter";
  safety_perimeter.operation = safety_perimeter.ADD;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 0.5;

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;

  pose.position.x = 0.5;
  pose.position.y = 0.0;
  pose.position.z = 0.25;

  safety_perimeter.primitives.push_back(primitive);
  safety_perimeter.primitive_poses.push_back(pose);

  collision_object_pub_.publish(safety_perimeter);
}

}  // namespace pick_and_place
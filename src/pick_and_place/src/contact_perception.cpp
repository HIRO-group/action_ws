#include "contact_perception.h"

namespace pick_and_place {

void ContactPerception::init() {
  planning_scene_diff_publisher_ =
      nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  moveit_msgs::CollisionObject safety_perimeter;
  safety_perimeter.header.frame_id = "panda_link0";
  safety_perimeter.id = "safety_perimeter";
  safety_perimeter.operation = safety_perimeter.ADD;

  // back wall
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.01;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 0.5;

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;

  pose.position.x = -0.5;
  pose.position.y = 0.0;
  pose.position.z = 0.25;

  safety_perimeter.primitives.push_back(primitive);
  safety_perimeter.primitive_poses.push_back(pose);

  // bottom table
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 1.5;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 0.01;

  // pose.orientation.w = 1.0;
  // pose.orientation.x = 0.0;
  // pose.orientation.y = 0.0;
  // pose.orientation.z = 0.0;

  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;

  safety_perimeter.primitives.push_back(primitive);
  safety_perimeter.primitive_poses.push_back(pose);

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.emplace_back(safety_perimeter);

  moveit_msgs::ObjectColor obj_color;
  // obj_color.id = safety_perimeter.id;
  std_msgs::ColorRGBA color;
  color.a = 0.5;
  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;
  obj_color.color = color;

  std::vector<moveit_msgs::ObjectColor> object_colors;
  object_colors.emplace_back(obj_color);

  addCollisionObjects(collision_objects, object_colors);
}

void ContactPerception::addCollisionObjects(
    const std::vector<moveit_msgs::CollisionObject>& collision_objects,
    const std::vector<moveit_msgs::ObjectColor>& object_colors) {
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects = collision_objects;
  planning_scene.object_colors = object_colors;

  for (size_t i = 0; i < planning_scene.object_colors.size(); ++i) {
    if (planning_scene.object_colors[i].id.empty() &&
        i < collision_objects.size())
      planning_scene.object_colors[i].id = collision_objects[i].id;
    else
      break;
  }

  planning_scene.is_diff = true;
  planning_scene_diff_publisher_.publish(planning_scene);
}

}  // namespace pick_and_place
#include "contact_perception.h"

#include <moveit_msgs/PlanningScene.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <chrono>
using namespace std::chrono;
constexpr char LOGNAME[] = "contact_perception";
namespace tacbot {

ContactPerception::ContactPerception()
    : point_cloud_(new pcl::PointCloud<pcl::PointXYZ>) {}

void ContactPerception::init() {
  planning_scene_diff_publisher_ =
      nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  // cloud_subscriber_ = nh_.subscribe(
  //     "/oak/points", 1, &ContactPerception::pointCloudCallback, this);

  // addSafetyPerimeter();

  // addCylinder();

  // addFrontWall();

  // this keeps callback through the duration of the class not just once, not
  // sure why
  // ros::spinOnce();
}

void ContactPerception::addSafetyPerimeter() {
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
  pose.position.z = -0.05;

  safety_perimeter.primitives.push_back(primitive);
  safety_perimeter.primitive_poses.push_back(pose);

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.emplace_back(safety_perimeter);

  moveit_msgs::ObjectColor obj_color;
  // obj_color.id = safety_perimeter.id;
  std_msgs::ColorRGBA color;
  color.a = 0.2;
  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;
  obj_color.color = color;

  std::vector<moveit_msgs::ObjectColor> object_colors;
  object_colors.emplace_back(obj_color);

  addCollisionObjects(collision_objects, object_colors);
}

void ContactPerception::addCylinder() {
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "panda_link0";
  collision_object.id = "cylinder_" + std::to_string(obst_num_);
  obst_num_++;
  collision_object.operation = collision_object.ADD;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.15;
  primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.045;

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;

  pose.position.x = 0.545;
  pose.position.y = -0.045;
  pose.position.z = 0.15;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.emplace_back(collision_object);

  moveit_msgs::ObjectColor obj_color;
  // obj_color.id = collision_object.id;
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;
  obj_color.color = color;

  std::vector<moveit_msgs::ObjectColor> object_colors;
  object_colors.emplace_back(obj_color);

  addCollisionObjects(collision_objects, object_colors);
}

void ContactPerception::addSphere(const Eigen::Vector3d& center,
                                  double radius) {
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "panda_link0";
  collision_object.id = "sphere_" + std::to_string(obst_num_);
  obst_num_++;
  collision_object.operation = collision_object.ADD;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.SPHERE;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.SPHERE_RADIUS] = radius;

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;

  pose.position.x = center[0];
  pose.position.y = center[1];
  pose.position.z = center[2];

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.emplace_back(collision_object);

  moveit_msgs::ObjectColor obj_color;
  // obj_color.id = collision_object.id;
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;
  obj_color.color = color;

  std::vector<moveit_msgs::ObjectColor> object_colors;
  object_colors.emplace_back(obj_color);

  addCollisionObjects(collision_objects, object_colors);
}

void ContactPerception::addFrontWall() {
  moveit_msgs::CollisionObject safety_perimeter;
  safety_perimeter.header.frame_id = "panda_link0";
  safety_perimeter.id = "plc_wall";
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

  pose.position.x = 1.2;
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

  pose.position.x = 1.0;
  pose.position.y = 0.0;
  pose.position.z = 0.1;

  safety_perimeter.primitives.push_back(primitive);
  safety_perimeter.primitive_poses.push_back(pose);

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.emplace_back(safety_perimeter);

  moveit_msgs::ObjectColor obj_color;
  // obj_color.id = safety_perimeter.id;
  std_msgs::ColorRGBA color;
  color.a = 0.2;
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

void ContactPerception::extractNormals(
    const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
    const pcl::PointIndices::Ptr& inliers_plane) {
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  extract_normals.setNegative(true);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*cloud_normals);
}

void ContactPerception::passThroughFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  // min and max values in z axis to keep
  pass.setFilterLimits(0.3, 1.1);
  pass.filter(*cloud);
}

void ContactPerception::computeNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  // Set the number of k nearest neighbors to use for the feature estimation.
  ne.setKSearch(50);
  ne.compute(*cloud_normals);
}

bool ContactPerception::extractNearPts(const Eigen::Vector3d& search_origin,
                                       std::vector<Eigen::Vector3d>& pts_out) {
  auto start = high_resolution_clock::now();

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(point_cloud_);
  pcl::PointXYZ search_point;
  search_point.x = search_origin[0];
  search_point.y = search_origin[1];
  search_point.z = search_origin[2];

  // Neighbors within radius search
  std::vector<int> pt_idx_search_rad;
  std::vector<float> pt_rad_sq_dist;
  pts_out.clear();

  if (kdtree.radiusSearch(search_point, PROXIMITY_RADIUS, pt_idx_search_rad,
                          pt_rad_sq_dist) <= 0) {
    // std::cout << "No points found." << std::endl;
    return false;
  }

  std::size_t num_pts = pt_idx_search_rad.size();
  // std::cout << "Num pts found: " << num_pts << std::endl;
  // should resize the pts_out here

  for (std::size_t i = 0; i < num_pts; ++i) {
    double x = (*point_cloud_)[pt_idx_search_rad[i]].x;
    double y = (*point_cloud_)[pt_idx_search_rad[i]].y;
    double z = (*point_cloud_)[pt_idx_search_rad[i]].z;
    // std::cout << "pt: " << x << ", " << y << ", " << z
    //           << " (squared distance: " << pt_rad_sq_dist[i] << ")"
    //           << std::endl;
    Eigen::Vector3d vec(x, y, z);
    pts_out.emplace_back(vec);
  }

  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<microseconds>(stop - start);
  // std::cout << "extractNearPts us: " << duration.count() << std::endl;
  return true;
}

void ContactPerception::pointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& input) {
  // First, we convert from sensor_msgs to pcl::PointXYZ which is needed for
  // most of the processing.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud);
  // std::cout << "raw cloud.size(): " << cloud->size() << std::endl;

  tf::StampedTransform transform;
  try {
    tf_listener_.lookupTransform("world", "oak_rgb_camera_optical_frame",
                                 ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl_ros::transformPointCloud(*cloud, *cloud_transformed, transform);

  // Create the filtering object
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_transformed);
  sor.setLeafSize(0.05f, 0.05f, 0.05f);
  sor.filter(*cloud_filtered);
  // std::cout << "filtered cloud.size(): " << cloud_filtered->size() <<
  // std::endl;
  point_cloud_ = std::move(cloud_filtered);
}

}  // namespace tacbot
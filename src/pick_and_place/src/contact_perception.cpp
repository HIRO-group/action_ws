#include "contact_perception.h"

namespace pick_and_place {

void ContactPerception::init() {
  planning_scene_diff_publisher_ =
      nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  cloud_subscriber_ =
      nh_.subscribe("/camera/depth_registered/points", 1,
                    &ContactPerception::pointCloudCallback, this);

  addSafetyPerimeter();
  ros::spinOnce();
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

void ContactPerception::extractNormals(
    const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
    const pcl::PointIndices::Ptr& inliers_plane) {
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  extract_normals.setNegative(true);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*cloud_normals);
}

/** \brief Given a pointcloud extract the ROI defined by the user.
    @param cloud - Pointcloud whose ROI needs to be extracted. */
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
  // First, we convert from sensor_msgs to pcl::PointXYZ which is needed for
  // most of the processing.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*point_cloud_, *cloud);
  // std::cout << "raw cloud.size(): " << cloud->size() << std::endl;

  // add outlier removal, filtering techniques

  // Use a passthrough filter to get the region of interest.
  // The filter removes points outside the specified range.
  // passThroughFilter(cloud);
  // std::cout << "filtered cloud.size(): " << cloud->size() << std::endl;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);
  pcl::PointXYZ search_point;
  search_point.x = search_origin[0];
  search_point.y = search_origin[1];
  search_point.z = search_origin[2];

  // Neighbors within radius search
  std::vector<int> pt_idx_search_rad;
  std::vector<float> pt_rad_sq_dist;

  // std::cout << "Neighbors within radius search at (" << search_point.x << ",
  // "
  //           << search_point.y << ", " << search_point.z
  //           << ") with radius= " << PROXIMITY_RADIUS << std::endl;

  pts_out.clear();

  if (kdtree.radiusSearch(search_point, PROXIMITY_RADIUS, pt_idx_search_rad,
                          pt_rad_sq_dist) <= 0) {
    // std::cout << "No points found." << std::endl;
    return false;
  }

  std::size_t num_pts = pt_idx_search_rad.size();
  // std::cout << "Num pts found: " << num_pts << std::endl;
  // should resize the points vector here

  for (std::size_t i = 0; i < num_pts; ++i) {
    double x = (*cloud)[pt_idx_search_rad[i]].x;
    double y = (*cloud)[pt_idx_search_rad[i]].y;
    double z = (*cloud)[pt_idx_search_rad[i]].z;
    // std::cout << "pt: " << x << ", " << y << ", " << z
    //           << " (squared distance: " << pt_rad_sq_dist[i] << ")"
    //           << std::endl;
    Eigen::Vector3d vec(x, y, z);
    pts_out.emplace_back(vec);
  }

  return true;
}

void ContactPerception::pointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& input) {
  std::cout << "received a single plc message" << std::endl;
  point_cloud_ = input;
}

}  // namespace pick_and_place
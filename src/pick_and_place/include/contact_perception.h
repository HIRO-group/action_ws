#ifndef CONTACT_PERCEPTION_H
#define CONTACT_PERCEPTION_H

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ObjectColor.h>

// PLC
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pick_and_place {
class ContactPerception {
 public:
  void init();

  void addCollisionObjects(
      const std::vector<moveit_msgs::CollisionObject>& collision_objects,
      const std::vector<moveit_msgs::ObjectColor>& object_colors);

  bool extractNearPts(const Eigen::Vector3d& search_origin,
                      std::vector<Eigen::Vector3d>& pts_out);

  const static inline double PROXIMITY_RADIUS = 0.3;

 private:
  ros::NodeHandle nh_;
  ros::Publisher planning_scene_diff_publisher_;
  ros::Subscriber cloud_subscriber_;

  sensor_msgs::PointCloud2ConstPtr point_cloud_;

  /** \brief Add a set of static obstacle around the robot. These obstacles
   * include any walls, tables, or beams that the robot should not approach
   * under any circumstances.
   */
  void addSafetyPerimeter();

  /** \brief Given the point normals and point indices, extract the normals for
     the indices.
      @param cloud_normals - Point normals.
      @param inliers_plane - Indices whose normals need to be extracted. */
  void extractNormals(const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                      const pcl::PointIndices::Ptr& inliers_plane);

  /** \brief Given a pointcloud extract the ROI defined by the user.
      @param cloud - Pointcloud whose ROI needs to be extracted. */
  void passThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  /** \brief Given the pointcloud and pointer cloud_normals compute the point
     normals and store in cloud_normals.
      @param cloud - Pointcloud.
      @param cloud_normals - The point normals once computer will be stored in
     this. */
  void computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals);

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
};
}  // namespace pick_and_place

#endif
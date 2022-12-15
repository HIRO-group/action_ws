#ifndef CONTACT_PERCEPTION_H
#define CONTACT_PERCEPTION_H

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ObjectColor.h>
#include <sensor_msgs/PointCloud2.h>
// PLC
#include <pcl/features/normal_3d.h>
#include <tf/transform_listener.h>

namespace pick_and_place {
class ContactPerception {
 public:
  ContactPerception();

  /** \brief Initialize the member variables. */
  void init();

  /** \brief Add collision objects to the planning scene.
      @param collision_objects - Vector of collision objects to be added.
      @param object_colors - The colors of each object .*/
  void addCollisionObjects(
      const std::vector<moveit_msgs::CollisionObject>& collision_objects,
      const std::vector<moveit_msgs::ObjectColor>& object_colors);

  /** \brief Uses kd search to find points (from a point cloud the class
     subscribes to) near the input origin.
      @param search_origin - Point from which the search originates.
      @param pts_out - The points that have been found within the radius.
      @return bool - Whether or not any points have been found within radius.*/
  bool extractNearPts(const Eigen::Vector3d& search_origin,
                      std::vector<Eigen::Vector3d>& pts_out);

  /** \brief Kd search will consider a point as an obstacle when it is within
   * this radius from a given point on the robot. Value in meters. */
  const double PROXIMITY_RADIUS = 1.0;

 private:
  ros::NodeHandle nh_;

  /** \brief Used to add static obstacles to the scene. These obstacles must be
   * respected by the collison checker.*/
  ros::Publisher planning_scene_diff_publisher_;

  /** \brief Subscries to camera point cloud. */
  ros::Subscriber cloud_subscriber_;

  /** \brief Used to transform the point cloud to robot frame. */
  tf::TransformListener tf_listener_;

  /** \brief Processes point cloud from the callback. */
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;

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

  /** \brief Processes the incoming point cloud. All transforms, conversion, and
     filtering needs to be done at this level. This way, less processing has to
     be done when the planner needs to use the perception class.
      @param input - Pointcloud as a ros sensor message. */
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
};
}  // namespace pick_and_place

#endif
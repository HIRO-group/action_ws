#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "publish_pc_bag");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher point_cloud_publisher =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud", 100, true);

  // Variable holding the rosbag containing point cloud data.
  rosbag::Bag bagfile;
  std::string path = ros::package::getPath("tacbot");
  path += "/bags/reprocessed_best.bag";
  bagfile.open(path, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back("/cloud");

  // Iterator for topics in bag.
  rosbag::View bag(bagfile, rosbag::TopicQuery(topics));

  sensor_msgs::PointCloud2::Ptr point_cloud_ptr =
      bag.begin()->instantiate<sensor_msgs::PointCloud2>();
  if (!point_cloud_ptr) {
    ROS_FATAL("invalid message in rosbag");
    return 1;
  }

  // Give a bit of time to move_group to connect & cache transforms
  // works around sporadic tf extrapolation errors
  ros::Duration(1.0).sleep();

  ros::Rate loop_rate(0.2);
  while (ros::ok()) {
    point_cloud_ptr->header.stamp = ros::Time::now();
    point_cloud_publisher.publish(*point_cloud_ptr);
    loop_rate.sleep();
  }

  bagfile.close();
  return 0;
}

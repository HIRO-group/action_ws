#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/Pose.h>

geometry_msgs::TransformStamped createCamTf() {
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "panda_link0";
  transformStamped.child_frame_id =
      "camera_rgb_optical_frame";  // oak_undist_rgb_cam_frame
  transformStamped.transform.translation.x = -0.3;
  transformStamped.transform.translation.y = -0.20;
  transformStamped.transform.translation.z = 0.6;
  tf2::Quaternion q;
  transformStamped.transform.rotation.x = 0.583;
  transformStamped.transform.rotation.y = -0.571;
  transformStamped.transform.rotation.z = 0.373;
  transformStamped.transform.rotation.w = -0.442;
  return transformStamped;

  // transformStamped.transform.translation.x = 0.318894;
  // transformStamped.transform.translation.y = -0.390716;
  // transformStamped.transform.translation.z = 0.602591;
  // tf2::Quaternion q;
  // transformStamped.transform.rotation.x = 0.803583;
  // transformStamped.transform.rotation.y = -0.438683;
  // transformStamped.transform.rotation.z = 0.133163;
  // transformStamped.transform.rotation.w = -0.379578;
  return transformStamped;
}

geometry_msgs::TransformStamped createWorldTf() {
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "panda_link0";
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  // q.setEuler(0.0, 0.0, 0.0);
  transformStamped.transform.rotation.x = 0.0;
  transformStamped.transform.rotation.y = 0.0;
  transformStamped.transform.rotation.z = 0.0;
  transformStamped.transform.rotation.w = 1.0;
  return transformStamped;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "broadcast_tf");
  ros::NodeHandle nh();
  tf2_ros::TransformBroadcaster br;

  ros::Rate r(10);  // 10 hz

  while (ros::ok()) {
    geometry_msgs::TransformStamped camera_tf = createCamTf();
    geometry_msgs::TransformStamped world_tf = createWorldTf();

    br.sendTransform(camera_tf);
    br.sendTransform(world_tf);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
};
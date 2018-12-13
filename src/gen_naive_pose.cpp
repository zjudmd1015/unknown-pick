/*
Author: Miaoding Dai (m.dai AT u.northwestern.edu DOT com)
*/

// #include <iostream>
// #include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// #include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <math.h> 

#define PI 3.14159265

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef sensor_msgs::PointCloud2 RosPointCloud2;

class NaivePoseGoalGenerator {
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    tf2_ros::TransformBroadcaster br_;
    geometry_msgs::TransformStamped tf_;

  public:
    NaivePoseGoalGenerator(): nh_("~") {
      sub_ = nh_.subscribe("/cloud_object", 1, &NaivePoseGoalGenerator::callback, this);
      pub_ = nh_.advertise<geometry_msgs::Pose>("pose_goal", 1);

      // initialize tf_
      tf_.header.stamp = ros::Time::now();
      tf_.header.frame_id = "camera_frame";
      tf_.child_frame_id  = "grasp_frame";
      tf_.transform.rotation.w = 1.0;

      // ROS loop
      ros::Rate br_rate(100);
      while (ros::ok()) {
        tf_.header.stamp = ros::Time::now();
        br_.sendTransform(tf_);
        ros::spinOnce();
        br_rate.sleep();
      }
    }

    ~NaivePoseGoalGenerator() {
    }
    
    void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_goal_generator");
  NaivePoseGoalGenerator npgg;
  return 0;
}

void NaivePoseGoalGenerator::callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  PointCloud::Ptr cloud_object(new PointCloud);
  pcl::fromROSMsg(*cloud_msg, *cloud_object);

  double center_x, center_y, center_z;
  int num_pts = cloud_object->points.size();
  for (auto p: cloud_object->points) {
      center_x += p.x;
      center_y += p.y;
      center_z += p.z;
  }
  center_x /= num_pts;
  center_y /= num_pts;
  center_z /= num_pts;

  geometry_msgs::Pose pose_goal;
  pose_goal.position.x = center_x;
  pose_goal.position.y = center_y;
  pose_goal.position.z = center_z;
  pose_goal.orientation.z = sin(PI/2);
  pose_goal.orientation.w = cos(PI/2);

  tf_.transform.translation.x = center_x;
  tf_.transform.translation.y = center_y;
  tf_.transform.translation.z = center_z;
  tf_.transform.rotation.z = sin(PI/2);
  tf_.transform.rotation.w = cos(PI/2);

  pub_.publish(pose_goal);
}
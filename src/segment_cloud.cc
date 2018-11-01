/*
Author: Miaoding Dai (m.dai AT u.northwestern.edu DOT com)
*/
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <unknown_pick/mask_req.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef sensor_msgs::PointCloud2 RosPointCloud2;

const double camera_factor = 1000;
const double camera_cx = 474.82;
const double camera_cy = 249.06;
const double camera_fx = 523.44;
const double camera_fy = 524.40;

int main(int argc, char** argv) {
  cv::Mat bgr, depth;
  // later on, these two (bgr and depth should be obtained from RGB-D sensor)
  bgr = cv::imread("/home/dylan2/unknown_pick_ws/src/unknown_pick/testdata/0001_color.jpg");
  depth = cv::imread("/home/dylan2/unknown_pick_ws/src/unknown_pick/testdata/0001_depth.png", -1);

  // cv::Mat mask;
  // mask = cv::imread("/home/dylan2/unknown_pick_ws/src/unknown_pick/testdata/mask_255.jpg", -1);

  // ROS setup
  ros::init(argc, argv, "segment_cloud");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<unknown_pick::mask_req>("generate_mask");

  ros::Publisher pub_object, pub_obstacle;
  pub_object = nh.advertise<sensor_msgs::PointCloud2>("cloud_object", 1);
  pub_obstacle = nh.advertise<sensor_msgs::PointCloud2>("cloud_obstacle", 1);



  // sensor_msgs::CvBridge bridge_;
  sensor_msgs::ImagePtr ros_rgb = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr).toImageMsg();

  unknown_pick::mask_req srv;
  srv.request.interested_object = "orange";
  srv.request.rgb_img = *ros_rgb;

  client.waitForExistence(ros::Duration(5.0));
  if (client.call(srv)) {
    // cv_bridge::CvImageConstPtr mask_ptr;
    cv::Mat mask;
    boost::shared_ptr<void const> tracked_object;

    if (srv.response.is_found == false) {
      ROS_INFO("Did not find the interested object!");
    } else {
      ROS_INFO("Found it, starting to generate point cloud...");
      // mask_ptr = cv_bridge::toCvShare(srv.response.obj_mask, tracked_object, "bgr8");
      // const cv::Mat &mask = mask_ptr->image;
      try {
        mask = cv_bridge::toCvShare(srv.response.obj_mask, tracked_object, "mono8")->image;
      } catch (cv::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to '%s'.", srv.response.obj_mask.encoding.c_str(), srv.response.obj_mask.encoding.c_str());
      }

      PointCloud::Ptr cloud(new PointCloud);
      for (int m = 0; m < depth.rows; m++) {
        for (int n = 0; n < depth.cols; n++) {
          uchar mask_pt = mask.ptr<uchar>(m)[n];
          if (mask_pt == 0)
            continue;
          ushort d = depth.ptr<ushort>(m)[n];
          if (d == 0)
            continue;

          PointT p;
          // calculate world coordinates
          p.z = double(d) / camera_factor;
          p.x = (n - camera_cx) * p.z / camera_fx;
          p.y = (m - camera_cy) * p.z / camera_fy;
          // add color
          p.b = bgr.ptr<uchar>(m)[n*3];
          p.g = bgr.ptr<uchar>(m)[n*3+1];
          p.r = bgr.ptr<uchar>(m)[n*3+2];
          // add this point to cloud
          cloud->points.push_back(p);
        }
      }

      // save the cloud
      cloud->height = 1;
      cloud->width = cloud->points.size();
      cout << "point cloud size = "<< cloud->points.size() << endl;
      cloud->is_dense = true;
      pcl::io::savePCDFile("./object.pcd", *cloud);

      // send to the ROS topic
      RosPointCloud2::Ptr cloud_object(new RosPointCloud2());
      pcl::toROSMsg(*cloud, *cloud_object);
      cloud_object->header.frame_id = "mdworld";
      pub_object.publish(*cloud_object);

      // clear the data and exit
      cloud->points.clear();
      cout << "Point cloud saved." << endl;

    }
  } else {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  // ROS Spin
  // ros::spin();
  return 0;
}

/*
Author: Miaoding Dai (m.dai AT u.northwestern.edu DOT com)
*/

// similar function with v1, but written as a class

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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef sensor_msgs::PointCloud2 RosPointCloud2;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;

const double camera_factor = 1000;
const double camera_cx = 474.82;
const double camera_cy = 249.06;
const double camera_fx = 523.44;
const double camera_fy = 524.40;

class CloudSegmentor {
  private:
    ros::NodeHandle nh_;
    // service client
    ros::ServiceClient client_;
    // subscriber
    // ros::Subscriber bgr_img_sub_;
    // ros::Subscriber depth_img_sub_;

    // publisher
    ros::Publisher object_pub_;
    ros::Publisher obstacle_pub_;

    // params
    // std::string interested_object_ = "orange";
    std::string interested_object_;
    
  public:
    CloudSegmentor() {
      // ros param
      if (nh_.getParam("/cloud_segmentor/interested_object", interested_object_)) {
        ROS_INFO("Got param: %s", interested_object_.c_str());
      } else {
        ROS_ERROR("Failed to get param: %s", interested_object_.c_str());
        ros::shutdown();
      }
      // service client
      client_ = nh_.serviceClient<unknown_pick::mask_req>("generate_mask");
      // publisher
      object_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_object", 1);
      obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_obstacle", 1);
      // subscriber - normal

/* FIXME: how to handle this subscriber? */
      // bgr_img_sub_ = nh_.subscribe("/kinect2/qhd/image_color_rect", 1, boost::bind(&CloudSegmentor::callback, this, _1));
    
      // subscriber synchronize
      message_filters::Subscriber<sensor_msgs::Image> bgr_img_sub_(nh_, "/kinect2/qhd/image_color_rect", 5);
      message_filters::Subscriber<sensor_msgs::Image> depth_img_sub_(nh_, "/kinect2/qhd/image_depth_rect", 5);
      /* method1 */
      // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(bgr_img_sub_, depth_img_sub_, 10);
      // sync.registerCallback(boost::bind(&CloudSegmentor::callback, this, _1, _2));
      /* method2 */
      // message_filters::Synchronizer<SyncPolicy>* sync_;
      // sync_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), bgr_img_sub_, depth_img_sub_);
      // sync_->registerCallback(boost::bind(&CloudSegmentor::callback, this, _1, _2));
      /* method3 */
      message_filters::Synchronizer<SyncPolicy> sync_(SyncPolicy(10), bgr_img_sub_, depth_img_sub_);
      sync_.registerCallback(boost::bind(&CloudSegmentor::callback, this, _1, _2));

/* FIXME: why ros::spin() must be here to make the code work? */
      ros::spin();
    }
    ~CloudSegmentor() {
      // delete sync_;
    }

    void callback(const sensor_msgs::ImageConstPtr& ros_bgr_img, const sensor_msgs::ImageConstPtr& ros_depth_img);
    
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_segmentor");
  CloudSegmentor cs;
  // ros::spin();
  return 0;
}


void CloudSegmentor::callback(const sensor_msgs::ImageConstPtr& ros_bgr_img, const sensor_msgs::ImageConstPtr& ros_depth_img) {
  
  cv::Mat bgr, depth;
  boost::shared_ptr<void const> tracked_object;
  ROS_INFO("inside the callback func");
  try {
    bgr = cv_bridge::toCvShare(*ros_bgr_img, tracked_object, "bgr8")->image;
    depth = cv_bridge::toCvShare(*ros_depth_img, tracked_object, "mono16")->image;
  } catch (cv::Exception &e) {
    ROS_ERROR("Could not convert from ROS image to CV image.");
  }

  /* for testing */
  // bgr = cv::imread("/home/dylan2/unknown_pick_ws/src/unknown_pick/testdata/0001_color.jpg");
  // depth = cv::imread("/home/dylan2/unknown_pick_ws/src/unknown_pick/testdata/0001_depth.png", -1);
  // cv::Mat mask;
  // mask = cv::imread("/home/dylan2/unknown_pick_ws/src/unknown_pick/testdata/mask_255.jpg", -1);

  // sensor_msgs::CvBridge bridge_;
  // sensor_msgs::ImagePtr ros_rgb = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr).toImageMsg();

  unknown_pick::mask_req srv;
  srv.request.interested_object = interested_object_;
  srv.request.rgb_img = *ros_bgr_img;

  client_.waitForExistence(ros::Duration(5.0));
  if (client_.call(srv)) {
    // cv_bridge::CvImageConstPtr mask_ptr;
    cv::Mat mask;

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

      // cloud:           point cloud of object
      // cloud_obstacle:  point cloud of obstacle
      PointCloud::Ptr cloud(new PointCloud);
      PointCloud::Ptr cloud_obstacle(new PointCloud);
      uchar cloud_interval = 20; 
      uchar cloud_interval_cnt = 0;

      for (int m = 0; m < depth.rows; m++) {
        for (int n = 0; n < depth.cols; n++) {
          ushort d = depth.ptr<ushort>(m)[n];
          if (d == 0)
            continue;

          uchar mask_pt = mask.ptr<uchar>(m)[n];
          if (mask_pt == 0) {
            cloud_interval_cnt ++;
            if (cloud_interval_cnt == cloud_interval) {
              // point cloud of obstacle
              PointT p;
              // calculate world coordinates
              p.z = double(d) / camera_factor;
              p.x = (n - camera_cx) * p.z / camera_fx;
              p.y = (m - camera_cy) * p.z / camera_fy;
              // add gray color
              p.b = p.g = p.r = 102;
              // add this point to cloud
              cloud_obstacle->points.push_back(p);

              cloud_interval_cnt = 0;
            }

          } else {
            // point cloud of interested object
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
      }

      // set attributes of point cloud
      cloud->height = 1;
      cloud->width = cloud->points.size();
      cout << "cloud_object size = "<< cloud->points.size() << endl;
      cloud->is_dense = true;

      cloud_obstacle->height = 1;
      cloud_obstacle->width = cloud_obstacle->points.size();
      cout << "cloud_obstacle size = "<< cloud_obstacle->points.size() << endl;
      cloud_obstacle->is_dense = true;

      // send to the ROS topic
      RosPointCloud2::Ptr ros_cloud_object(new RosPointCloud2());
      RosPointCloud2::Ptr ros_cloud_obstacle(new RosPointCloud2());
      pcl::toROSMsg(*cloud, *ros_cloud_object);
      pcl::toROSMsg(*cloud_obstacle, *ros_cloud_obstacle);
      ros_cloud_object->header.frame_id = "camera_frame";
      ros_cloud_obstacle->header.frame_id = "camera_frame";

      ros::Rate r(5);
      while (ros::ok()) {
        object_pub_.publish(*ros_cloud_object);
        obstacle_pub_.publish(*ros_cloud_obstacle);
        r.sleep();
      }  

      // save the cloud, clear the data and exit
      pcl::io::savePCDFile("/home/dylan2/unknown_pick_ws/src/unknown_pick/debug/object.pcd", *cloud);
      cloud->points.clear();
      cloud_obstacle->points.clear();
      cout << "Point cloud saved." << endl;

    }
  } else {
    ROS_ERROR("Failed to call service");
    return;
  }

}
#ifndef STITCHING_TRAVERSABILITY_GRAPH_H
#define STITCHING_TRAVERSABILITY_GRAPH_H

#include <ros/ros.h>

#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>
#include <voxblox_ros/transformer.h>
#include <minkindr_conversions/kindr_tf.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <math.h>

constexpr bool kDefaultVerbose = false;
constexpr bool kDefaultAutomaticPub = false;
static double  kDefaultResolution = 20;
static double  kDefaultGamma = 1.0;
static double  kDefaultRobotSize = 0.5;
void gammaCorrection(cv::Mat& src, cv::Mat& dst, float fGamma);

class TraversabilityGraphStitcher {

public:
  TraversabilityGraphStitcher(ros::NodeHandle nh, ros::NodeHandle nh_private);
  // Stolen from voxblox_node that stole it from octomap_manager
  bool lookupTransformTf(const std::string& from_frame,
                         const std::string& to_frame,
                         const ros::Time& timestamp,
                         voxblox::Transformation* transform);

private:
  // Initial interactions with ROS
  void subscribeToTopics();
  void advertiseTopics();
  void advertiseServices();
  void getParametersFromRos();

  // service callBack
  bool publishCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  // Datacallback
  void pointCloudCallback(const sensor_msgs::PointCloud2::Ptr& pointCloud);

  // Node Handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Tf listener
  tf::TransformListener tf_listener_;
  // Data subscribers.
  ros::Subscriber pcd_sub_;
  ros::Subscriber points_sub_;
  // Data publisher
  image_transport::Publisher  img_pub_;
  ros::Publisher source_pub_;
  ros::Publisher dest_pub_;
  ros::Publisher transform_pub_;
  // services
  ros::ServiceServer publish_srv_;

  // Params
  bool verbose_;
  bool automatic_pub_;
  double resolution_;
  double gamma_;
  double robot_size_;
  std::string world_frame_;

  // Global variables
  bool first;
  bool pub;
  // limits(in meters) of the full area (for trsansform)
  double minX;
  double maxX;
  double minY;
  double maxY;
  double zscale;
  // images of the full area
  cv::Mat traversability_full_image;
  cv::Mat elevation_full_image;
  cv::Mat pcd_in_full_image;
};

#endif //

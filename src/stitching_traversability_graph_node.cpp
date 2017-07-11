#include <ros/ros.h>
#include <stitching_traversability_graph.hpp>

// Standard C++ entry point
int main(int argc, char **argv) {

  // Announce this program to the ROS master
  ros::init(argc, argv, "stitching_traversability_graph_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Creating the object to do the work.
  TraversabilityGraphStitcher stitching_traversability_graph(nh, private_nh);

  // automatic start after 10 sec
  ros::Rate rate(0.07);
  cv::waitKey(10);
  rate.sleep();
  rate.sleep();

  ros::spin();
  return 0;
}


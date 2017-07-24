#include <stitching_traversability_graph.hpp>

int main(int argc, char **argv) {

  // Announce this program to the ROS master
  ros::init(argc, argv, "stitching_traversability_graph_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // Creating the object to do the work.
  TraversabilityGraphStitcher stitching_traversability_graph(nh, private_nh);

  // automatic start after 14 sec (drone in the air)
  ros::Rate rate(0.07);
  cv::waitKey(10);
  //rate.sleep();
  //rate.sleep();

  ros::spin();
  return 0;
}


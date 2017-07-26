# stitching_traversability_graph #
ROS package containing 3 nodes :

*  **stitching_traversability_graph_node :** subscribe to /stereo_point2 from "/dense_reconstruction" and make a stitching of the elevation map then compute traversability map as a binary image

*  **a_star_planner_node :**  subscribe to a binary occupation map and to source et goal point then compute the trajectory between them using A* algorithm

*  **pixel_to_metric_path_node :**  subscribe to the pixel trajectory of the A* and transform it into the global metric frame using the informations from the stitching node

## Dependences ##

* Opencv, Boost, pcl, voxblox, minkindr

## Usage ##

* **roslaunch rosbag_stitching_traversability_astar.launch :** 

Launch rosbag (you have to use yours) 

dense_stereo (you have to use your calibration file .yaml)

stiching_traversability_graph_node (compute the globale elevation map by stitching locale ones) : ! to clear outliers, you have to know the limits of the area (hardcoded TODO:param)!

A_star_planer_node

pixel_to_metric_path_node

## Parameters ##

* **scale :** param from dense reconstruction to reduce the nmber of point in the pointCloud

* **resolution :** param for the image resolution in pixels/meter

* **robot_size :** the size of the robot in pixels for the filtering process and trajectory sampling

* **gamma :** value used in gamme filter of the elevation map image to reduce noize

* **automatic_pub :** if false, you need to use service call for the publishment

## Pipeline ##

![alt text](https://raw.githubusercontent.com/Renaudeau82/stitching_traversability_graph/master/pipeline.png "Pipeline")

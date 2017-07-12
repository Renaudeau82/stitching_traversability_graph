# stitching_traversability_graph #
ROS package containing 2 nodes :

*  **stitching_traversability_graph_node :** subscribe to /stereo_point2 from "/dense_reconstruction" and make a stitching of the elevation map then compute traversability map as a binary image

*  **a_star_planner_node :**  subscribe to a binary occupation map and to source et goal point then compute the trajectory between them using A* algorithm

# Dependences #

* Opencv, Boost, pcl, voxblox, minkindr

# Usage #

* **roslaunch rosbag_stitching_traversability_astar.launch :** Launch rosbag, dense reconstruction, stiching_traversability and astar_planer

# Parameters #

* **scale :** param from dense reconstruction to reduce the nmber of point in the pointCloud

* **resolution :** param for the image size in pixels/meter

* **robot_size :** param the size of the robot in pixels for the filtering process

* **gamma :** value used in gamme filter of the elevation map image to reduce noize

# Pipeline #

![alt text](https://raw.githubusercontent.com/Renaudeau82/stitching_traversability_graph/master/pipeline.png "Pipeline")

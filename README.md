# stitching_traversability_graph #
ROS package containing 2 nodes :

*  **stitching_traversability_graph_node :** subscribe to /stereo_point2 from "/dense_reconstruction" and make a stitching of the elevation map then compute traversability map as a binary image

*  **a_star_planner_node :**  subscribe to a binary occupation map and to source et goal point then compute the trajectory between them using A* algorithm


# Dependences #

* Opencv, Boost, pcl, voxblox, minkindr

# Usage #

* **roslaunch rosbag_stitching_traversability_astar.launch :** Launch rosbag, dense reconstr

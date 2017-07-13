cmake_minimum_required(VERSION 2.8.3)
project(stitching_traversability_graph)
set(CMAKE_BUILD_TYPE RELEASE)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_package(INCLUDE_DIRS include)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(${Eigen_INCLUDE_DIRS})


cs_add_library(stitching_traversability_graph
  src/library/stitching_traversability_graph.cpp
)

# create the node for the stitching and compute traversability (no graph)
cs_add_executable(stitching_traversability_graph_node
  src/stitching_traversability_graph_node.cpp
  include/stitching_traversability_graph.hpp
)
target_link_libraries(stitching_traversability_graph_node stitching_traversability_graph ${catkin_LIBRARIES})

# create the node to solve path finding (A*) in image
cs_add_executable(a_star_planner_node
  src/a_star_planner.cpp
)
target_link_libraries(a_star_planner_node stitching_traversability_graph ${catkin_LIBRARIES})

# create the node to transform path in pixels to path in meters
cs_add_executable(pixel_to_metric_path_node
  src/pixel_to_metric_path_node.cpp
)
target_link_libraries(pixel_to_metric_path_node stitching_traversability_graph ${catkin_LIBRARIES})

cs_install()
cs_export()



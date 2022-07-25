cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_to_grid)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17 -Ofast)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  sensor_msgs
  nav_msgs
  pcl_ros
  roscpp
)


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/MyParams.cfg
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES pointcloud_to_grid
  #CATKIN_DEPENDS dynamic_reconfigure nav_msgs pcl_ros roscpp
  #DEPENDS roscpp std_msgs
)
  
  
## Specify additional locations of header files
## Your package locations should be listed before other locations
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(
  include
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pointcloud_to_grid_node src/pointcloud_to_grid_node.cpp)
#add_dependencies(pointcloud_to_map_node ${PROJECT_NAME}_gencfg)
target_link_libraries(pointcloud_to_grid_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
  
  
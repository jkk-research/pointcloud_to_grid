cmake_minimum_required(VERSION 3.8)
project(pointcloud_to_grid)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Compile as C++17 and with optimization level O2
add_compile_options(-std=c++17 -O2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED) 

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(INCLUDE_DIRS 
  include 
  ${ament_cmake_INCLUDE_DIRS} 
  ${rclcpp_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS} 
  ${pcl_conversions_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
)
include_directories(${INCLUDE_DIRS})

set(ament_dependencies
  pcl_conversions
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs  
  PCL
)

add_executable(pointcloud_to_grid_node src/pointcloud_to_grid_node.cpp)
target_link_libraries(pointcloud_to_grid_node ${PCL_LIBRARIES})
ament_target_dependencies(pointcloud_to_grid_node  ${ament_dependencies} )
target_compile_features(pointcloud_to_grid_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


add_executable(grid_trajectory src/grid_trajectory.cpp)
target_link_libraries(grid_trajectory ${PCL_LIBRARIES})
ament_target_dependencies(grid_trajectory  ${ament_dependencies} )
target_compile_features(grid_trajectory PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(DIRECTORY 
  launch
  doc
  DESTINATION share/${PROJECT_NAME})

install(TARGETS 
  pointcloud_to_grid_node
  grid_trajectory
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

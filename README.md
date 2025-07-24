# `pointcloud_to_grid` ROS 2 package
This package converts `sensor_msgs/PointCloud2` LIDAR data to both `nav_msgs/OccupancyGrid` and `grid_map_msgs/GridMap` 2D map data based on intensity and / or height.
![](doc/grid_map01.gif)

[![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)

If you would like to use ROS 1 version (melodic, noetic), please go to [`ROS1` branch](https://github.com/jkk-research/pointcloud_to_grid/tree/ros1).

## Build
```
cd ~/ros2_ws/src 
git clone https://github.com/jkk-research/pointcloud_to_grid -b ros2
cd ~/ros2_ws/ 
colcon build --packages-select pointcloud_to_grid --symlink-install
```
Don't foget to `source ~/ros2_ws/install/setup.bash`. 


## Features
- Few dependencies (ROS 2, PCL, and grid_map_msgs mainly) [ROS installation](http://wiki.ros.org/ROS/Installation)
- **Dual output format support**: Publishes both `nav_msgs/OccupancyGrid` and `grid_map_msgs/GridMap` messages simultaneously
- Simple as possible
- Fast

## Conventions & definitions
![](doc/grid_map_conventions.svg)

# Getting started

In a **new terminal** go to your bag folder (e.g. `~/Downloads`):

```
cd ~/Downloads
```

Download a sample rosbag (~3,3 GB) (TODO: update to ROS 2 `mcap``):

```r
wget https://laesze-my.sharepoint.com/:u:/g/personal/herno_o365_sze_hu/EYl_ahy5pgBBhNHt5ZkiBikBoy_j_x95E96rDtTsxueB_A?download=1 -O leaf-2021-04-23-campus.bag
```

Play rosbag:

```r
ros2 bag play -l ~/Downloads/leaf-2021-04-23-campus.bag
```

Start the algorithm in a **new terminal** :
```r
ros2 launch pointcloud_to_grid demo.launch.py
```
Alternatively, start with subscribing to `/my_custom_cloud_topic`:
```r
ros2 launch pointcloud_to_grid demo.launch.py topic:=my_custom_cloud_topic
```

Start the visualization in a **new terminal** :
```r
ros2 launch pointcloud_to_grid rviz.launch.py
```

## Dual Output Format Support

The package now supports publishing both `nav_msgs/OccupancyGrid` and `grid_map_msgs/GridMap` message types simultaneously. This allows for better integration with different ROS 2 packages that may prefer one format over the other.

### New Parameters

In addition to the existing parameters, the following new parameters control the GridMap output topics:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `mapi_gridmap_topic_name` | string | `intensity_gridmap` | Topic name for intensity GridMap |
| `maph_gridmap_topic_name` | string | `height_gridmap` | Topic name for height GridMap |

### Output Topics

The node now publishes to four topics simultaneously:

**OccupancyGrid format:**
- `intensity_grid` (`nav_msgs/OccupancyGrid`)
- `height_grid` (`nav_msgs/OccupancyGrid`)

**GridMap format:**
- `intensity_gridmap` (`grid_map_msgs/GridMap`)
- `height_gridmap` (`grid_map_msgs/GridMap`)

### Usage Example

```bash
# Launch with custom GridMap topic names
ros2 launch pointcloud_to_grid demo.launch.py topic:=my_pointcloud mapi_gridmap_topic_name:=my_intensity_map maph_gridmap_topic_name:=my_height_map
```

## QoS Configuration

The package is configured to use `BEST_EFFORT` reliability QoS policy for the input point cloud subscription. This ensures compatibility with typical LiDAR sensor publishers that often use this policy for performance reasons. This prevents QoS compatibility warnings that might appear with the default `RELIABLE` policy.


## Related solutions
- [github.com/ANYbotics/grid_map](https://github.com/ANYbotics/grid_map) - This is a C++ library with ROS interface to manage two-dimensional grid maps with multiple data layers. 
- [github.com/306327680/PointCloud-to-grid-map](https://github.com/306327680/PointCloud-to-grid-map) - A similar solution but instead PointCloud2 it uses PointCloud

## Remarks

In VS code it is advised to add the following to include path:

``` r
${workspaceFolder}/**
/opt/ros/humble/include/**
/usr/include/pcl-1.12/**
/usr/include/eigen3/**
```

If you are not sure where your header files are use e.g.:
``` r
find /usr/include -name point_cloud.h
find /usr/include -name crop_box.h
```


## Cite & paper

If you use any of this code please consider citing TODO:

```bibtex
```
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include <memory>
class PointXY
{
public:
  int x;
  int y;
};

class PointXYZI
{
public:
  double x;
  double y;
  double z;
  double intensity;
};

class GridMap
{
public:
  float position_x;
  float position_y;
  float cell_size;
  float length_x;
  float length_y;
  std::string cloud_in_topic;
  std::string frame_out;
  std::string mapi_topic_name;
  std::string maph_topic_name;
  std::string mapi_gridmap_topic_name;
  std::string maph_gridmap_topic_name;
  float topleft_x;
  float topleft_y;
  float bottomright_x;
  float bottomright_y;
  int cell_num_x;
  int cell_num_y;
  float intensity_factor;
  float height_factor;

  void initGrid(std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid)
  {
    // grid->header.seq = 1;
    grid->header.frame_id = GridMap::frame_out; // TODO
    grid->info.origin.position.z = 0;
    grid->info.origin.orientation.w = 0;
    grid->info.origin.orientation.x = 0;
    grid->info.origin.orientation.y = 0;
    grid->info.origin.orientation.z = 1;
    grid->info.origin.position.x = position_x + length_x / 2;
    grid->info.origin.position.y = position_y + length_y / 2;
    grid->info.width = length_x / cell_size;
    grid->info.height = length_y / cell_size;
    grid->info.resolution = cell_size;
    // resolution/grid size [m/cell]
  }

  void initGridMap(std::shared_ptr<grid_map_msgs::msg::GridMap> grid_map_msg, const std::string& layer_name)
  {
    grid_map_msg->header.frame_id = GridMap::frame_out;
    grid_map_msg->info.resolution = cell_size;
    grid_map_msg->info.length_x = length_x;
    grid_map_msg->info.length_y = length_y;
    grid_map_msg->info.pose.position.x = position_x;
    grid_map_msg->info.pose.position.y = position_y;
    grid_map_msg->info.pose.position.z = 0.0;
    grid_map_msg->info.pose.orientation.w = 1.0;
    grid_map_msg->info.pose.orientation.x = 0.0;
    grid_map_msg->info.pose.orientation.y = 0.0;
    grid_map_msg->info.pose.orientation.z = 0.0;
    
    // Clear existing layers and add the new one
    grid_map_msg->layers.clear();
    grid_map_msg->layers.push_back(layer_name);
    
    // Initialize data array
    grid_map_msg->data.clear();
    grid_map_msg->data.resize(1); // One layer
    grid_map_msg->data[0].layout.dim.resize(2);
    grid_map_msg->data[0].layout.dim[0].label = "column_index";
    grid_map_msg->data[0].layout.dim[0].size = cell_num_y;
    grid_map_msg->data[0].layout.dim[0].stride = cell_num_x * cell_num_y;
    grid_map_msg->data[0].layout.dim[1].label = "row_index";
    grid_map_msg->data[0].layout.dim[1].size = cell_num_x;
    grid_map_msg->data[0].layout.dim[1].stride = cell_num_x;
    grid_map_msg->data[0].layout.data_offset = 0;
    grid_map_msg->data[0].data.resize(cell_num_x * cell_num_y);
  }

  void paramRefresh()
  {
    topleft_x = position_x + length_x / 2;
    bottomright_x = position_x - length_x / 2;
    topleft_y = position_y + length_y / 2;
    bottomright_y = position_y - length_y / 2;
    cell_num_x = int(length_x / cell_size);
    cell_num_y = int(length_y / cell_size);
    if (cell_num_x > 0)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Cells: " << cell_num_x << "*" << cell_num_y << "px [" << topleft_x << ", " << topleft_y << "]"
                                                                 << " [" << bottomright_x << ", " << bottomright_y << "]");
    }
  }

  // number of cells
  int getSize()
  {
    return cell_num_x * cell_num_y;
  }

  // number of cells
  int getSizeX()
  {
    return cell_num_x;
  }

  // number of cells
  int getSizeY()
  {
    return cell_num_y;
  }

  // length [m] meters
  double getLengthX()
  {
    return length_x;
  }

  // length [m] meters
  double getLengthY()
  {
    return length_y;
  }

  // resolution [m/cell] size of a single cell
  double getResolution()
  {
    return cell_size;
  }

  // x and y are in meters, it returs the cell index
  PointXY getIndex(double x, double y)
  {
    PointXY ret;
    ret.x = int(fabs(x - topleft_x) / cell_size);
    ret.y = int(fabs(y - topleft_y) / cell_size);
    return ret;
  }
};
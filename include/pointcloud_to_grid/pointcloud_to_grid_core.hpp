#pragma once
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
class PointXY{
public: 
  int x;
  int y;
};

class PointXYZI{
public: 
  double x;
  double y;
  double z;
  double intensity;
};

class GridMap{
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
    float topleft_x;
    float topleft_y;
    float bottomright_x;
    float bottomright_y;
    int cell_num_x;
    int cell_num_y;
    float intensity_factor;
    float height_factor;



    void initGrid(nav_msgs::OccupancyGridPtr grid) {
      grid->header.seq = 1;
      grid->header.frame_id = GridMap::frame_out; // TODO
      grid->info.origin.position.z = 0;
      grid->info.origin.orientation.w = 0;
      grid->info.origin.orientation.x = 0;
      grid->info.origin.orientation.y = 0;
      grid->info.origin.orientation.z = 1;
      grid->info.origin.position.x = position_x + length_x / 2;
      grid->info.origin.position.y = position_y + length_y / 2;
      grid->info.width = length_x / cell_size;
      grid->info.height = length_y /cell_size;
      grid->info.resolution = cell_size;
      // resolution/grid size [m/cell]
    }
    
    void paramRefresh(){
      topleft_x = position_x + length_x / 2;
      bottomright_x = position_x - length_x / 2;
      topleft_y = position_y + length_y / 2;
      bottomright_y = position_y - length_y / 2;
      cell_num_x = int(length_x / cell_size);
      cell_num_y = int(length_y / cell_size);
      if(cell_num_x > 0){
        ROS_INFO_STREAM("Cells: " << cell_num_x << "*" << cell_num_y << "px, subscribed to " << GridMap::cloud_in_topic << " [" << topleft_x << ", " << topleft_y << "]" << " [" << bottomright_x << ", " << bottomright_y << "]");
      }
    }

    // number of cells
    int getSize(){
      return cell_num_x * cell_num_y;
    }
    
    // number of cells
    int getSizeX(){
      return cell_num_x;
    }

    // number of cells
    int getSizeY(){
      return cell_num_y;
    }

    // length [m] meters
    double getLengthX(){
      return length_x;
    }

    // length [m] meters
    double getLengthY(){
      return length_y;
    }

    // resolution [m/cell] size of a single cell
    double getResolution(){
      return cell_size;
    }
};
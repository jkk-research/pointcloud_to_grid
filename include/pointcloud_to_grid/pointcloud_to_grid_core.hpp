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

    std::vector<float> ranges {0.0, 0.0480580431416584, 0.04952912820025457, 0.05107271476990993, 0.05269363345060185, 0.05439712423301346, 0.05618887884892665, 0.05807508832254582, 0.06006249646573103, 0.06215846018173332, 0.06437101758608277, 0.06670896512456004, 0.06918194507246911, 0.0718005450435939, 0.07457641143041283, 0.07752237905008785, 0.08065261969712312, 0.08398281282093834, 0.08753034217607691, 0.09131452306247567, 0.09535686571760005, 0.09968138158636286, 0.10431494063602464, 0.10928768967611369, 0.11463354388430158, 0.12039076655489023, 0.1266026556430777, 0.1333183601959842, 0.14059385553413595, 0.14849311346929106, 0.15708951344942435, 0.1664675530375357, 0.17672493255767296, 0.1879751104675096, 0.20035045498449566, 0.21400615644938892, 0.2291251177806517, 0.24592411282444182, 0.2646616027219162, 0.2856477408593898, 0.30925729592893525, 0.33594650809441173, 0.3662753084830479, 0.4009369451742195, 0.440797978284289, 0.4869530100827344, 0.5408006990423111, 0.6041510737922202, 0.6793797969258417, 0.769654418585306, 0.8792737568385096, 1.014190021335894, 1.182835509100519, 1.3974753794571182, 1.6765073714116525, 2.0485501901036933, 2.560109231404816, 3.290908031115226, 4.387106404889913, 6.141023984061373, 9.21037993915209, 15.34909205824954, 30.695872599693494, 92.08299511357528};


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

    float get_Offset(int ring);
};
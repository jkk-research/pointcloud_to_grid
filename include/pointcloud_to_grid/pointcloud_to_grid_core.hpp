#pragma once
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pointcloud_to_grid/pointcloud_to_grid_core.hpp>
#include <pointcloud_to_grid/point.h>
#include <pointcloud_to_grid/MyParamsConfig.h>
#include <dynamic_reconfigure/server.h>


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

    std::vector<float> ranges {2.728061325481597, 2.7761193686232555, 2.82564849682351, 2.87672121159342, 2.929414845044022, 2.9838119692770353, 3.040000848125962, 3.0980759364485078, 3.158138432914239, 3.220296893095972, 3.284667910682055, 3.351376875806615, 3.420558820879084, 3.492359365922678, 3.5669357773530908, 3.6444581564031786, 3.7251107761003017, 3.80909358892124, 3.896623931097317, 3.9879384541597926, 4.083295319877393, 4.1829767014637556, 4.28729164209978, 4.396579331775894, 4.5112128756601955, 4.631603642215086, 4.758206297858163, 4.891524658054148, 5.0321185135882835, 5.180611627057575, 5.337701140506999, 5.504168693544535, 5.680893626102208, 5.868868736569717, 6.069219191554213, 6.283225348003602, 6.5123504657842535, 6.758274578608695, 7.0229361813306115, 7.308583922190001, 7.6178412181189366, 7.953787726213348, 8.320063034696396, 8.720999979870616, 9.161797958154905, 9.64875096823764, 10.18955166727995, 10.79370274107217, 11.473082537998012, 12.242736956583318, 13.122010713421828, 14.136200734757722, 15.31903624385824, 16.71651162331536, 18.39301899472701, 20.441569184830705, 23.00167841623552, 26.292586447350747, 30.67969285224066, 36.82071683630203, 46.031096775454124, 61.38018883370366, 92.07606143339716, 184.15905654697244};


    void initGrid(nav_msgs::OccupancyGrid& grid) {
      grid.header.seq = 1;
      grid.header.frame_id = GridMap::frame_out; // TODO
      grid.info.origin.position.z = 0;
      grid.info.origin.orientation.w = 0;
      grid.info.origin.orientation.x = 0;
      grid.info.origin.orientation.y = 0;
      grid.info.origin.orientation.z = 1;
      grid.info.origin.position.x = position_x + length_x / 2;
      grid.info.origin.position.y = position_y + length_y / 2;
      grid.info.width = length_x / cell_size;
      grid.info.height = length_y /cell_size;
      grid.info.resolution = cell_size;
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

class ROSHandler
{
  public:

    ros::Subscriber pc_road_sub,pc_obstacle_sub;
    ros::Publisher occupancy_grid_pub;

    pcl::PointCloud<ouster::Point>::ConstPtr obstacle_points;

    dynamic_reconfigure::Server<my_dyn_rec::MyParamsConfig> dr_srv_;

    GridMap grid_map;

    my_dyn_rec::MyParamsConfig config;

    void pointcloudCallback(const pcl::PointCloud<ouster::Point>::ConstPtr input_cloud);
    void obstaclePointsCallback(const pcl::PointCloud<ouster::Point>::ConstPtr input_cloud);

    int getIndex(double x, double y);

    ROSHandler(ros::NodeHandlePtr);
    void paramsCallback(my_dyn_rec::MyParamsConfig &config, uint32_t level);
    std::vector<int> getOffset_indexes(int offset_to_make,auto out_point);
    void getBeams(auto &beam_list,double beam_num_,const pcl::PointCloud<ouster::Point>::ConstPtr input_cloud);
    void set_map_cells_in_grid(const auto &beam_list, const std::vector<bool>& obstacle_indices, std::vector<signed char> &map,double beam_num_,double resolution_);

};
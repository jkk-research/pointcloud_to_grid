#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pointcloud_to_grid/pointcloud_to_grid_core.hpp>
#include <pointcloud_to_grid/MyParamsConfig.h>
#include <dynamic_reconfigure/server.h>

nav_msgs::OccupancyGridPtr intensity_grid(new nav_msgs::OccupancyGrid);
nav_msgs::OccupancyGridPtr height_grid(new nav_msgs::OccupancyGrid);
GridMap grid_map;
ros::Publisher pub_igrid, pub_hgrid;
ros::Subscriber sub_pc2;

PointXY getIndex(double x, double y){
  PointXY ret;
  ret.x = int(fabs(x - grid_map.topleft_x) / grid_map.cell_size);
  ret.y = int(fabs(y - grid_map.topleft_y) / grid_map.cell_size);
  return ret;
}

void paramsCallback(my_dyn_rec::MyParamsConfig &config, uint32_t level)
{
  grid_map.cell_size = config.cell_size;
  grid_map.position_x = config.position_x;
  grid_map.position_y = config.position_y;
  grid_map.cell_size = config.cell_size;
  grid_map.length_x = config.length_x;
  grid_map.length_y = config.length_y;
  grid_map.cloud_in_topic = config.cloud_in_topic;
  grid_map.intensity_factor = config.intensity_factor;
  grid_map.height_factor = config.height_factor;
  //grid_map.frame_out = config.frame_out;
  grid_map.mapi_topic_name = config.mapi_topic_name;
  grid_map.maph_topic_name = config.maph_topic_name;
  grid_map.cloud_in_topic = config.cloud_in_topic;
  grid_map.initGrid(intensity_grid);
  grid_map.initGrid(height_grid);
  grid_map.paramRefresh();
}


void pointcloudCallback(const pcl::PCLPointCloud2 &msg)
{
  pcl::PointCloud<pcl::PointXYZI> out_cloud;
  pcl::fromPCLPointCloud2(msg, out_cloud);
  // Initialize grid
  grid_map.initGrid(intensity_grid);
  grid_map.initGrid(height_grid);
  // width*height/cell_size^2 eg width = 20m, height = 30m, cell_size data size = 6000
  // or cell_num_x * cell_num_ys
  // -128 127 int8[] data
  std::vector<signed char> hpoints(grid_map.cell_num_x * grid_map.cell_num_y);
  std::vector<signed char> ipoints(grid_map.cell_num_x * grid_map.cell_num_y);
  // initialize grid vectors: -128
  for (auto& p : hpoints){p = -128;}
  for (auto& p : ipoints){p = -128;}
  //for (int i = 0; i < out_cloud.points.size(); ++i) // out_cloud.points[i].x instead of out_point.x
  for (auto out_point : out_cloud)
  {
    if (out_point.x > 0.01 || out_point.x < -0.01){
      if (out_point.x > grid_map.bottomright_x && out_point.x < grid_map.topleft_x)
      {
        if (out_point.y > grid_map.bottomright_y && out_point.y < grid_map.topleft_y)
        {
          PointXY cell = getIndex(out_point.x, out_point.y);
          if (cell.x < grid_map.cell_num_x && cell.y < grid_map.cell_num_y){
            ipoints[cell.y * grid_map.cell_num_x + cell.x] = out_point.intensity * grid_map.intensity_factor;
            hpoints[cell.y * grid_map.cell_num_x + cell.x] = out_point.z * grid_map.height_factor;
          }
          else{
            ROS_WARN_STREAM("Cell out of range: " << cell.x << " - " << grid_map.cell_num_x << " ||| " << cell.y << " - " << grid_map.cell_num_y);
          }
        }
      }
    }
  }
  intensity_grid->header.stamp = ros::Time::now();
  intensity_grid->header.frame_id = msg.header.frame_id; // TODO
  intensity_grid->info.map_load_time = ros::Time::now();
  intensity_grid->data = ipoints;
  height_grid->header.stamp = ros::Time::now();
  height_grid->header.frame_id = msg.header.frame_id; // TODO
  height_grid->info.map_load_time = ros::Time::now();  
  height_grid->data = hpoints;
  pub_igrid.publish(intensity_grid);
  pub_hgrid.publish(height_grid);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_to_grid_node");
  dynamic_reconfigure::Server<my_dyn_rec::MyParamsConfig> server;
  dynamic_reconfigure::Server<my_dyn_rec::MyParamsConfig>::CallbackType f;
  f = boost::bind(&paramsCallback, _1, _2);
  server.setCallback(f);
  ros::NodeHandle nh;
  pub_igrid = nh.advertise<nav_msgs::OccupancyGrid>(grid_map.mapi_topic_name, 1);
  pub_hgrid = nh.advertise<nav_msgs::OccupancyGrid>(grid_map.maph_topic_name, 1);
  sub_pc2 = nh.subscribe(grid_map.cloud_in_topic, 1, pointcloudCallback);
  ros::spin();
  return 0;
}
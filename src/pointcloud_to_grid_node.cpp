#include <pointcloud_to_grid/pointcloud_to_grid_core.hpp>


PointXY ROSHandler::getIndex(double x, double y){
  PointXY ret;
  ret.x = int(fabs(x - grid_map.topleft_x) / grid_map.cell_size);
  ret.y = int(fabs(y - grid_map.topleft_y) / grid_map.cell_size);
  return ret;
}
void ROSHandler::paramsCallback(my_dyn_rec::MyParamsConfig &config, uint32_t level __attribute__((unused)))
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

  grid_map.paramRefresh();
}

float GridMap::get_Offset(int ring)
{
  int actual_ring = 128 - ring;

  float offset_on_ring = ranges[actual_ring];

  return offset_on_ring / cell_size;
}

ROSHandler::ROSHandler(ros::NodeHandlePtr nh)
{
  dynamic_reconfigure::Server<my_dyn_rec::MyParamsConfig>::CallbackType f;

  f = boost::bind(&ROSHandler::paramsCallback,this,_1,_2);

  dr_srv_.setCallback(f);

  pc_road_sub = nh->subscribe(grid_map.cloud_in_topic,1,&ROSHandler::pointcloudCallback,this);
  pc_obstacle_sub = nh->subscribe("/obstacle_point",1,&ROSHandler::obstaclePointsCallback,this);

  occupancy_grid_pub = nh->advertise<nav_msgs::OccupancyGrid>(grid_map.maph_topic_name, 1);

}

void ROSHandler::obstaclePointsCallback(const pcl::PointCloud<ouster::Point>::ConstPtr input_cloud)
{
  obstacle_points = input_cloud;
}

void ROSHandler::pointcloudCallback(const pcl::PointCloud<ouster::Point>::ConstPtr input_cloud)
{
  nav_msgs::OccupancyGrid occupancyGrid_msg;
  // Initialize grid
  grid_map.initGrid(occupancyGrid_msg);
  // width*height/cell_size^2 eg width = 20m, height = 30m, cell_size data size = 6000
  // or cell_num_x * cell_num_ys
  // -128 127 int8[] data

  std::vector<signed char> occ_points(grid_map.cell_num_x * grid_map.cell_num_y);
  // initialize grid vectors: -1 as unknown
  for (auto& p : occ_points){p = -1;}

  for (const auto& out_point : input_cloud->points)
  {
    if (out_point.x > 0.01 || out_point.x < -0.01){
      if (out_point.x > grid_map.bottomright_x && out_point.x < grid_map.topleft_x)
      {
        if (out_point.y > grid_map.bottomright_y && out_point.y < grid_map.topleft_y)
        {
          PointXY cell = getIndex(out_point.x, out_point.y);
          int longitudinal_offset = grid_map.get_Offset(out_point.ring);
          longitudinal_offset++;

          for(int i = longitudinal_offset*-1;i <= longitudinal_offset;i++)
          {
            float x_off = out_point.x + (i * grid_map.cell_size);
            float y_off = (out_point.y / out_point.x) * x_off;
            PointXY off_setted_cell = getIndex(x_off,y_off);
            if (off_setted_cell.x < grid_map.cell_num_x && off_setted_cell.y < grid_map.cell_num_y){
              occ_points[off_setted_cell.y * grid_map.cell_num_x + off_setted_cell.x ] = out_point.intensity * grid_map.intensity_factor;
            }
          }
          if(longitudinal_offset < 1)
          {
            if (cell.x < grid_map.cell_num_x && cell.y < grid_map.cell_num_y){
              occ_points[cell.y * grid_map.cell_num_x + cell.x] = out_point.intensity * grid_map.intensity_factor;
            }
          }
        }
      }
    }
  }

  for (const auto& out_point : obstacle_points->points)
  {
    if (out_point.x > 0.01 || out_point.x < -0.01)
    {
      if (out_point.x > grid_map.bottomright_x && out_point.x < grid_map.topleft_x)
      {
        if (out_point.y > grid_map.bottomright_y && out_point.y < grid_map.topleft_y)
        {
          PointXY cell = getIndex(out_point.x, out_point.y);
          
          if (cell.x < grid_map.cell_num_x && cell.y < grid_map.cell_num_y){
            occ_points[cell.y * grid_map.cell_num_x + cell.x ] = 100;
          }
        }
      }
    }
  }
  

  for(float i = 0;i > -4;i -= grid_map.cell_size)
  {
    for(float j = -1.5;j < 1.5;j += grid_map.cell_size)
    {
      PointXY cell = getIndex(i,j);
      occ_points[cell.y * grid_map.cell_num_x + cell.x ] = 0;
    }
  }

  occupancyGrid_msg.header.stamp = ros::Time::now();
  occupancyGrid_msg.header.frame_id = input_cloud->header.frame_id; // TODO
  occupancyGrid_msg.info.map_load_time = ros::Time::now();
  occupancyGrid_msg.data = occ_points;
  occupancy_grid_pub.publish(occupancyGrid_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_to_grid_node");

  ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();

  ROSHandler rosHandler(nh);

  ros::spin();
  return 0;
}
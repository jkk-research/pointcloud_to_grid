#include <pointcloud_to_grid/pointcloud_to_grid_core.hpp>

float clip(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}


int ROSHandler::getIndex(double x, double y){
  PointXY ret;
  ret.x = int(fabs(x - grid_map.topleft_x) / grid_map.cell_size);
  ret.y = int(fabs(y - grid_map.topleft_y) / grid_map.cell_size);

  if(ret.x < grid_map.cell_num_x && ret.y < grid_map.cell_num_y)
    return ret.y * grid_map.cell_num_x + ret.x ;
  else
  {
    ret.x = int(fabs(0 - grid_map.topleft_x) / grid_map.cell_size);
    ret.y = int(fabs(0 - grid_map.topleft_y) / grid_map.cell_size);
    return ret.y * grid_map.cell_num_x + ret.x;
  }

}
geometry_msgs::Point ROSHandler::index_to_point(int index)
{
  geometry_msgs::Point pp;

  pp.x = ((index % grid_map.grid_width) - (grid_map.grid_width/2))*grid_map.cell_size;
  pp.y = ((index / grid_map.grid_width) - (grid_map.grid_width/2))*grid_map.cell_size;

  if(pp.x < grid_map.length_x/2)
    pp.x *= -1;

  if(pp.y < grid_map.length_x/2)
    pp.y *= -1;

  return pp;

}

void ROSHandler::occ_to_global(geometry_msgs::TransformStamped &transform,std::vector<signed char> &occ_map)
{
  geometry_msgs::Point current_point,point_global;
  GlobalPoint g_point;

  for(int i = 0; i < occ_map.size();i++)
  {
    current_point = index_to_point(i);
    tf2::doTransform(current_point,point_global,transform);
    g_point.point = point_global;
    g_point.occupancy = occ_map[i];
    g_point.distance = distance_covered;
    global_occ.push_back(g_point);
  }

}

void ROSHandler::global_to_occ(geometry_msgs::TransformStamped &transform,std::vector<signed char> &occ_map)
{
  geometry_msgs::Point point_local;

  for(const auto &point : global_occ)
  {
    tf2::doTransform(point.point,point_local,transform);
    int index = getIndex(point_local.x,point_local.y);

    if(index < occ_map.size())
      occ_map[index] = point.occupancy;
  }
  global_occ.clear();

  for(int i = 0; i < global_occ.size();i++)
  {
    if(distance_covered - global_occ[i].distance > 20.0)
      global_occ.erase(global_occ.begin() + i); 
  }
}

std::vector<int> ROSHandler::getOffset_indexes(int offset_to_make,auto out_point)
{
  std::vector<int> indexes_to_fill;
  for(int i = offset_to_make*-1;i <= offset_to_make;i++)
  {
    double x_off = out_point.x + (i * grid_map.cell_size);
    double y_off = (out_point.y / out_point.x) * x_off;
    indexes_to_fill.push_back(getIndex(x_off,y_off));
  }

  return indexes_to_fill;
}

float dist_from_point(int x,int y)
{
  return sqrt(pow(x,2) + pow(y,2));
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

  this->config = config;

  grid_map.paramRefresh();
}

float GridMap::get_Offset(int ring)
{
  int actual_ring = 128 - ring;

  float offset_on_ring = ranges[actual_ring];

  float offset_r = offset_on_ring / cell_size;

  offset_r = clip(offset_r,1,100);

  return offset_r;
}

ROSHandler::ROSHandler(ros::NodeHandlePtr nh) : listener_(tf_buffer_)
{
  dynamic_reconfigure::Server<my_dyn_rec::MyParamsConfig>::CallbackType f;

  f = boost::bind(&ROSHandler::paramsCallback,this,_1,_2);

  dr_srv_.setCallback(f);

  pc_road_sub = nh->subscribe(grid_map.cloud_in_topic,1,&ROSHandler::pointcloudCallback,this);
  pc_obstacle_sub = nh->subscribe("/obstacle_point",1,&ROSHandler::obstaclePointsCallback,this);
  distance_sub = nh->subscribe("/distance",1,&ROSHandler::distance_callback,this);

  occupancy_grid_pub = nh->advertise<nav_msgs::OccupancyGrid>(grid_map.maph_topic_name, 1);

}
void ROSHandler::getBeams(auto &beam_list,double beam_num_,const auto input_cloud)
{
  const double beam_angle_resolution = 2.0 * M_PI / (double)beam_num_;

  for(const auto &p : input_cloud->points)
  {
    const double distance = sqrt(p.x * p.x + p.y * p.y);
    const double direction = atan2(p.y, p.x);
    const int beam_index = (direction + M_PI) / beam_angle_resolution;
    auto range_index = 2;
    for(auto i = 1;i < grid_map.ranges.size();i++)
    {
      if(grid_map.ranges[i-1] < distance && distance < grid_map.ranges[i])
      {
        if(0 <= beam_index && beam_index < beam_num_)
          beam_list[beam_index][i] = true;
      }
    }
  }
}
void ROSHandler::set_map_cells_in_grid(const auto &beam_list, const std::vector<bool>& obstacle_indices, std::vector<signed char> &map,double beam_num_,double resolution_)
{
  const double beam_angle_resolution = 2.0 * M_PI / (double)beam_num_;
  for(int i = 0;i < beam_num_; i++)
  {
    double direction = i * beam_angle_resolution - M_PI;
    direction = atan2(sin(direction), cos(direction));
    const double c = cos(direction);
    const double s = sin(direction);

    for(int r = 0; r < beam_list[i].size()-1 && r < grid_map.ranges.size()-1;r++)
    {
      if(beam_list[i][r] == true)
      {
        double start_range = (grid_map.ranges[r-1] + grid_map.ranges[r])/2;
        double end_range = (grid_map.ranges[r] + grid_map.ranges[r+1])/2;
        for(double range = start_range; range < end_range; range += resolution_)
        {
          double x = range * c;
          double y = range * s;
          const int index = getIndex(x, y);

          if(index < map.size())
            map[index] = 0;
        }
      }
    }
  }
}

void ROSHandler::obstaclePointsCallback(const pcl::PointCloud<ouster::Point>::ConstPtr input_cloud)
{
  obstacle_points = input_cloud;
}

void ROSHandler::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr input_cloud)
{
  nav_msgs::OccupancyGrid occupancyGrid_msg;
  // Initialize grid
  grid_map.initGrid(occupancyGrid_msg);
  // width*height/cell_size^2 eg width = 20m, height = 30m, cell_size data size = 6000
  // or cell_num_x * cell_num_ys
  // -128 127 int8[] data

  CloudXYZINPtr cloud_ptr(new CloudXYZIN);

  pcl::fromROSMsg(*input_cloud, *cloud_ptr);

  try{
      geometry_msgs::TransformStamped transform;
      transform = tf_buffer_.lookupTransform("base_link", cloud_ptr->header.frame_id, ros::Time(0));
      const Eigen::Matrix4d mat = tf2::transformToEigen(transform.transform).matrix().cast<double>();
      pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, mat);
    cloud_ptr->header.frame_id = "base_link";
    }catch(tf2::TransformException& ex){
      std::cout << ex.what() << std::endl;
      return;
  }

  std::vector<signed char> occ_points(grid_map.cell_num_x * grid_map.cell_num_y);
  // initialize grid vectors: -1 as unknown
  for (auto& p : occ_points){p = -1;}

  double beam_num_ = config.beam_number;
  std::vector<std::vector<bool>> beam_list(beam_num_, std::vector<bool>(grid_map.ranges.size(), false));
  std::vector<bool> obstacle_indices;

  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform("base_link", "odom", ros::Time(0));
  }catch(tf2::TransformException& ex){
    std::cout << ex.what() << std::endl;
    return;
  }

  global_to_occ(transform,occ_points);

  getBeams(beam_list,beam_num_,cloud_ptr);
  set_map_cells_in_grid(beam_list,obstacle_indices,occ_points,beam_num_,grid_map.cell_size/2);

  try
  {
    transform = tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0));
  }catch(tf2::TransformException& ex){
    std::cout << ex.what() << std::endl;
    return;
  }

  occ_to_global(transform,occ_points);

  occupancyGrid_msg.header.stamp = ros::Time::now();
  occupancyGrid_msg.header.frame_id = "base_link";
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
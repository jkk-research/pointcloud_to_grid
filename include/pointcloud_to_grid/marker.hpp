#ifndef MARKER_HPP_
#define MARKER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"



void init_debug_marker(visualization_msgs::msg::Marker& debug_marker, double pose_x, double pose_y, int id){
    debug_marker.ns = "debug" + std::to_string(id);
    debug_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    debug_marker.action = visualization_msgs::msg::Marker::MODIFY;
    debug_marker.scale.x = 0.4;
    debug_marker.scale.y = 0.4;
    debug_marker.scale.z = 0.4;
    debug_marker.color.r = 0.1;
    debug_marker.color.g = 0.2;
    debug_marker.color.b = 0.8;
    debug_marker.color.a = 1.0;
    debug_marker.id = 3;
    debug_marker.pose.position.x = pose_x;
    debug_marker.pose.position.y = pose_y;
    debug_marker.pose.position.z = 0.0;
    // debug_marker.points.clear();
}

void init_text_debug_marker(visualization_msgs::msg::Marker& debug_marker){
    debug_marker.ns = "debug_text";
    debug_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    debug_marker.action = visualization_msgs::msg::Marker::MODIFY;
    debug_marker.scale.z = 0.4;
    debug_marker.color.r = 0.2;
    debug_marker.color.g = 0.8;
    debug_marker.color.b = 0.2;
    debug_marker.color.a = 1.0;
    debug_marker.id = 4;
    debug_marker.pose.position.x = 0.0;
    debug_marker.pose.position.y = 0.0;
    debug_marker.pose.position.z = 0.0;
    // debug_marker.points.clear();


}


#endif // MARKER_HPP_
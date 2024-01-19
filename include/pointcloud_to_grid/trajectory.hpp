#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

#include <cmath>
#include <vector>
#include <algorithm>
#include <functional>
#include <iostream>

// Extending the GridMap class
class GridMapTrajectory : public GridMap
{
public:
    // Bresenham's Line Generation Algorithm
    bool drawline(std::vector<signed char> &grid_points, double x_start, double y_start, double theta, double length)
    {
        double x_end = x_start + length * cos(theta);
        double y_end = y_start + length * sin(theta);
        PointXY c_start = getIndex(x_start, y_start);
        PointXY c_end = getIndex(x_end, y_end);
        int x0 = c_start.x;
        int y0 = c_start.y;
        int x1 = c_end.x;
        int y1 = c_end.y;
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "theta:  " << theta << " length: " << length);
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "meters: " << x_start << ", " << y_start << " -> " << x_end << ", " << y_end);
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "cells:  " << x0 << ", " << y0 << " -> " << x1 << ", " << y1);

        // draw end point for debugging
        // if (x1 < cell_num_x && y1 < cell_num_y)
        // {
        //     grid_points[y1 * cell_num_x + x1] = 127;
        // }

        // Bresenham's line algorithm, which works on negative slopes
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            if (x0 >= 0 && x0 < cell_num_x && y0 >= 0 && y0 < cell_num_y)
            {   
                if (grid_points[y0 * cell_num_x + x0] > 50)
                {
                    // not free
                    return false;
                    break;
                }
                // draw line points for debugging
                grid_points[y0 * cell_num_x + x0]= 1;
            }

            if (x0 == x1 && y0 == y1)
                break;

            int e2 = 2 * err;

            if (e2 > -dy)
            {
                err -= dy;
                x0 += sx;
            }

            if (e2 < dx)
            {
                err += dx;
                y0 += sy;
            }
        }
        // free from points
        return true;
    }
};

#endif // TRAJECTORY_HPP_
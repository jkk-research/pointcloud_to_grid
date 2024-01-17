#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

#include <cmath>
#include <vector>
#include <algorithm>
#include <functional>
#include <iostream>


// Bresenham's Line Generation Algorithm
void drawline(std::vector<signed char>& grid_points, int cell_num_x, int x0, int y0, int x1, int y1)
{
    int dx, dy, p, x, y;

    dx = x1 - x0;
    dy = y1 - y0;

    x = x0;
    y = y0;

    p = 2 * dy - dx;

    while (x < x1)
    {
        if (p >= 0)
        {
            grid_points[y * cell_num_x + x] += 50; 
            if(grid_points[y * cell_num_x + x]  > 127)
            {
                grid_points[y * cell_num_x + x] = 127;
            }
            y = y + 1;
            p = p + 2 * dy - 2 * dx;
        }
        else
        {
            grid_points[y * cell_num_x + x] += 50; 
            if(grid_points[y * cell_num_x + x]  > 127)
            {
                grid_points[y * cell_num_x + x] = 127;
            }
            p = p + 2 * dy;
        }
        x = x + 1;
    }
}

// drawline(x0, y0, x1, y1);

#endif // TRAJECTORY_HPP_
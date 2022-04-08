#include "trajectory.hpp"
#include <cmath>
#include <ros/ros.h>
#include "cubic_spline.hpp"
#include <array>

std::vector<Position> generate_full_trajectory(double x_begin, double y_begin, double goal_1_x, double goal_1_y, double goal_2_x, double goal_2_y)
{
    std::vector<float> rx, ry;
    rx.emplace_back(x_begin);
    ry.emplace_back(y_begin);

    rx.emplace_back(goal_1_x);
    ry.emplace_back(goal_1_y);
    rx.emplace_back(goal_2_x);
    ry.emplace_back(goal_2_y);

    ROS_WARN("Generate cubic spline start");
    ROS_WARN("rx: %f, ry: %f", rx[0], ry[0]);
    ROS_WARN("rx: %f, ry: %f", rx[1], ry[1]);
    ROS_WARN("rx: %f, ry: %f", rx[2], ry[2]);

    cpprobotics::Spline2D csp_obj(rx, ry);
    cpprobotics::Vec_f r_x, r_y;
    for (float i = 0; i < csp_obj.s.back(); i += 0.1)
    {
        std::array<float, 2> point_ = csp_obj.calc_postion(i);
        r_x.push_back(point_[0]);
        r_y.push_back(point_[1]);
    }

    ROS_WARN("Generate cubic spline done");

    std::vector<Position> trajectory;
    for (int i = 0; i < r_x.size(); i++)
    {
        trajectory.emplace_back(r_x[i], r_y[i]);
        //     ROS_WARN("rx: %f, ry: %f", spath.rx[i], spath.ry[i]);
    }

    return trajectory;
}

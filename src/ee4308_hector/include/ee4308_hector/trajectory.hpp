#include "common.hpp"
#include "vector"

#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP
std::vector<Position> generate_trajectory(double x_begin, double y_begin, double x_end, double y_end, double average_speed, double target_dt, double look_ahead_duration);
std::vector<Position> generate_full_trajectory(double x_begin, double y_begin, double turtle_x, double turtle_y, double goal_x, double goal_y, double look_ahead_duration);

#endif
#include "common.hpp"
#include "vector"

#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP
std::vector<Position> generate_full_trajectory(double x_begin, double y_begin, double goal_1_x, double goal_1_y, double goal_2_x, double goal_2_y);
std::vector<Position> generate_trajectory(double x_begin, double y_begin, double goal_1_x, double goal_1_y);
std::vector<Position> generate_straight_line_trajectory(double x_begin, double y_begin, double goal_1_x, double goal_1_y, double goal_2_x, double goal_2_y, double look_ahead_duration);

#endif
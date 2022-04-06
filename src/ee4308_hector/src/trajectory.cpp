#include "trajectory.hpp"
#include <cmath>

std::vector<Position> generate_trajectory(double x_begin, double y_begin, double x_end, double y_end, double average_speed, double target_dt, double look_ahead_duration)
{
    // (1) estimate total duration
    double Dx = x_end - x_begin;
    double Dy = y_end - y_begin;
    double duration = sqrt(Dx * Dx + Dy * Dy) / average_speed;

    // (2) generate cubic / quintic trajectory
    // done by students

    // Add turtle_target, add_turtle_estimated,
    // In between, add current + look_ahead distance
    Position pos_begin(x_begin, y_begin);
    std::vector<Position> trajectory = {pos_begin};
    double count = 1;
    for (double time = target_dt; time < duration; time += target_dt)
    {
        if (time + look_ahead_duration < duration)
        {
            trajectory.emplace_back(
                pos_begin.x + (Dx * count * look_ahead_duration) / duration,
                pos_begin.y + (Dy * count * look_ahead_duration) / duration);
        }
        count += 1;
    }
    trajectory.emplace_back(Position(x_end, y_end));

    // OR (2) generate targets for each target_dt
    // Position pos_begin(x_begin, y_begin);
    // std::vector<Position> trajectory = {pos_begin};
    // for (double time = target_dt; time < duration; time += target_dt)
    // {
    //     trajectory.emplace_back(
    //         pos_begin.x + Dx*time / duration,
    //         pos_begin.y + Dy*time / duration
    //     );
    // }

    return trajectory;
}


std::vector<Position> generate_full_trajectory(double x_begin, double y_begin, double goal_1_x, double goal_1_y, double goal_2_x, double goal_2_y, double look_ahead_duration) {
    Position pos_begin(x_begin, y_begin);
    std::vector<Position> trajectory = { pos_begin };
    double Dx = goal_1_x - x_begin;
    double Dy = goal_1_y - y_begin;

    double target_pos_x = pos_begin.x + Dx*look_ahead_duration;
    double target_pos_y = pos_begin.y + Dy*look_ahead_duration;
    if (!(dist_euc(target_pos_x,target_pos_y, goal_1_x, goal_1_y) < 1)) {
        //Add intermediate target
        trajectory.emplace_back(
        pos_begin.x + Dx*look_ahead_duration,
        pos_begin.y + Dy*look_ahead_duration
    );
    }



    //Add traj to subsequent target (can potentially predict)
    trajectory.emplace_back(goal_1_x, goal_1_y);
    if (goal_2_x && goal_2_y) {
        trajectory.emplace_back(goal_2_x,goal_2_y);
    }

    return trajectory;
}



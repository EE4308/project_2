#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <errno.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include "common.hpp"
#include <trajectory.hpp>
#define NaN std::numeric_limits<double>::quiet_NaN()

enum HectorState
{
    TAKEOFF,
    LAND,
    TURTLE,
    START,
    GOAL
};
std::string to_string(HectorState state)
{
    switch (state)
    {
    case TAKEOFF:
        return "TAKEOFF";
    case LAND:
        return "LAND";
    case TURTLE:
        return "TURTLE";
    case START:
        return "START";
    case GOAL:
        return "GOAL";
    default:
        return "??";
    }
}

bool verbose, use_cubic_spline;
double initial_x, initial_y, initial_z;
double x = NaN, y = NaN, z = NaN, a = NaN;
void cbHPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    auto &p = msg->pose.pose.position;
    x = p.x;
    y = p.y;
    z = p.z;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    a = atan2(siny_cosp, cosy_cosp);
}
double turtle_x = NaN, turtle_y = NaN;
void cbTPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    turtle_x = p.x;
    turtle_y = p.y;
}
double vx = NaN, vy = NaN, vz = NaN, va = NaN;
void cbHVel(const geometry_msgs::Twist::ConstPtr &msg)
{
    vx = msg->linear.x;
    vy = msg->linear.y;
    vz = msg->linear.z;
    va = msg->angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_main");
    ros::NodeHandle nh;

    // Make sure motion and move can run (fail safe)
    nh.setParam("run", true); // turns off other nodes

    double main_iter_rate;
    if (!nh.param("main_iter_rate", main_iter_rate, 25.0))
        ROS_WARN(" HMAIN : Param main_iter_rate not found, set to 25");
    if (!nh.param("initial_x", initial_x, 0.0))
        ROS_WARN(" HMAIN : Param initial_x not found, set initial_x to 0.0");
    if (!nh.param("initial_y", initial_y, 0.0))
        ROS_WARN(" HMAIN : Param initial_y not found, set initial_y to 0.0");
    if (!nh.param("initial_z", initial_z, 0.178))
        ROS_WARN(" HMAIN : Param initial_z not found, set initial_z to 0.178");
    double height;
    if (!nh.param("height", height, 2.0))
        ROS_WARN(" HMAIN : Param initial_z not found, set to 5");
    double look_ahead;
    if (!nh.param("look_ahead", look_ahead, 1.0))
        ROS_WARN(" HMAIN : Param look_ahead not found, set to 1");
    double close_enough;
    if (!nh.param("close_enough", close_enough, 0.1))
        ROS_WARN(" HMAIN : Param close_enough not found, set to 0.1");
    double average_speed;
    if (!nh.param("average_speed", average_speed, 2.0))
        ROS_WARN(" HMAIN : Param average_speed not found, set to 2.0");
    if (!nh.param("verbose_main", verbose, true))
        ROS_WARN(" HMAIN : Param verbose_main not found, set to false");
    if (!nh.param("use_cubic_spline", use_cubic_spline, false))
        ROS_WARN(" HMAIN : Param use_cubic_spline not found, set use_cubic_spline to false");
    // get the final goal position of turtle
    std::string goal_str;
    double goal_x = NaN, goal_y = NaN;
    if (nh.param("/turtle/goals", goal_str, std::to_string(initial_x) + "," + std::to_string(initial_y))) // set to initial hector positions
    {
        char goal_str_tok[goal_str.length() + 1];
        strcpy(goal_str_tok, goal_str.c_str()); // to tokenise --> convert to c string (char*) first
        char *tok = strtok(goal_str_tok, " ,");
        try
        {
            while (tok != nullptr)
            {
                goal_x = strtod(tok, nullptr);
                goal_y = strtod(strtok(nullptr, " ,"), nullptr);
                tok = strtok(nullptr, " ,");
            }
            ROS_INFO(" HMAIN : Last Turtle Goal is (%lf, %lf)", goal_x, goal_y);
        }
        catch (...)
        {
            ROS_ERROR(" HMAIN : Invalid Goals: %s", goal_str.c_str());
            ros::shutdown();
            return 1;
        }
    }
    else
        ROS_WARN(" HMAIN : Param goal not found, set to %s", goal_str.c_str());

    // --------- Subscribers ----------
    ros::Subscriber sub_hpose = nh.subscribe("pose", 1, &cbHPose);
    ros::Subscriber sub_tpose = nh.subscribe("/turtle/pose", 1, &cbTPose);
    ros::Subscriber sub_hvel = nh.subscribe("velocity", 1, &cbHVel);

    // --------- Publishers ----------
    ros::Publisher pub_target = nh.advertise<geometry_msgs::PointStamped>("target", 1, true);
    geometry_msgs::PointStamped msg_target;
    msg_target.header.frame_id = "world";
    ros::Publisher pub_rotate = nh.advertise<std_msgs::Bool>("rotate", 1, true);
    std_msgs::Bool msg_rotate;
    ros::Publisher pub_traj = nh.advertise<nav_msgs::Path>("trajectory", 1, true);
    nav_msgs::Path msg_traj;
    msg_traj.header.frame_id = "world";

    // --------- Wait for Topics ----------
    while (ros::ok() && nh.param("run", true) && (std::isnan(x) || std::isnan(turtle_x) || std::isnan(vx))) // not dependent on main.cpp, but on motion.cpp
        ros::spinOnce();                                                                                    // update the topics

    // --------- Main loop ----------
    ROS_INFO(" HMAIN : ===== BEGIN =====");
    HectorState state = TAKEOFF;
    ros::Rate rate(main_iter_rate);
    msg_traj.poses.push_back(geometry_msgs::PoseStamped()); // insert a posestamped initialised to all 0
    std::vector<Position> trajectory;
    while (ros::ok() && nh.param("run", true))
    {
        // get topics
        ros::spinOnce();

        //// IMPLEMENT ////
        switch (state)
        {
        case TAKEOFF:
        {
            // Update target towards takeoff height
            if (std::abs(z - height) < close_enough)
            {
                ROS_INFO_STREAM("[TAKEOFF] Takeoff Completed");
                state = START;
                msg_rotate.data = true;
                pub_rotate.publish(msg_rotate);
            }
            else
            {
                // Disable Rotate
                msg_rotate.data = false;
                pub_rotate.publish(msg_rotate);

                // Point at height above start point
                msg_target.point.x = initial_x;
                msg_target.point.y = initial_y;
                msg_target.point.z = height;
                ROS_INFO_STREAM("[TAKEOFF] Started Takeoff");
            }
            break;
        }
        case TURTLE:
        {
            // Update trajectory towards final GOAL
            if (use_cubic_spline)
            {
                trajectory = generate_full_trajectory(x, y, goal_x, goal_y, initial_x, initial_y);
            }
            else
            {
                trajectory = generate_straight_line_trajectory(x, y, goal_x, goal_y, initial_x, initial_y, look_ahead);
            }

            if (dist_euc(x, y, goal_x, goal_y) < close_enough)
            {
                state = GOAL;
                ROS_INFO_STREAM("[GOAL] Entered GOAL state");
            }
            break;
        }
        case START:
        {
            // Update trajectory towards turtle pos
            if (use_cubic_spline)
            {
                trajectory = generate_full_trajectory(x, y, turtle_x, turtle_y, goal_x, goal_y);
            }
            else
            {
                trajectory = generate_straight_line_trajectory(x, y, turtle_x, turtle_y, goal_x, goal_y, look_ahead);
            }
            if (dist_euc(x, y, turtle_x, turtle_y) < close_enough)
            {
                state = TURTLE;
                ROS_INFO_STREAM("[TURTLE] Entered TURTLE state");
            }
            if (dist_euc(x,y, initial_x, initial_y) < close_enough && !nh.param("/turtle/run",false)){
                state = LAND;
                ROS_INFO_STREAM("[LAND] Entered LAND state");
            }
            break;
        }
        case GOAL:
        {
            // Update traj towards START
            if (!nh.param("/turtle/run", false))
            {
                if (use_cubic_spline) {
                    trajectory = generate_trajectory(x, y, initial_x, initial_y);
                } else {
                    trajectory = generate_straight_line_trajectory(x,y, initial_x, initial_y, initial_x, initial_y, look_ahead);
                }
            }
            else
            {
                // Update traj towards START, if reach start, update state to start
                if (use_cubic_spline)
                {
                    trajectory = generate_full_trajectory(x, y, initial_x, initial_y, turtle_x, turtle_y);
                }
                else
                {
                    trajectory = generate_straight_line_trajectory(x, y, initial_x, initial_y, turtle_x, turtle_y, look_ahead);
                }
            }
            if (dist_euc(x, y, initial_x, initial_y) < close_enough)
            {
                state = START;
                ROS_INFO_STREAM("[START] Entered START state");
            }
            break;
        }
        case LAND:
        {
            if (dist_euc(x, y, initial_x, initial_y) < close_enough)
            {
                trajectory.clear();
                msg_rotate.data = false;
                pub_rotate.publish(msg_rotate);
                msg_target.point.z = 0.0;
            }
            else
            {
                msg_target.point.x = initial_x;
                msg_target.point.y = initial_y;
            }
            break;
        }
        default:
            ROS_WARN_STREAM("[ERROR] FSM state is not defined");
            break;
        }

        // Publish target
        if (trajectory.size() > 0)
        {
            if (use_cubic_spline)
            {
                double look_ahead_index = look_ahead * 10;
                // If trajectory is larger than look_ahead distance
                if (trajectory.size() > look_ahead_index + 1)
                {
                    msg_target.point.x = trajectory.at(look_ahead_index).x;
                    msg_target.point.y = trajectory.at(look_ahead_index).y;
                }
                else
                {
                    msg_target.point.x =  trajectory.at(trajectory.size() - 1).x;
                    msg_target.point.y =  trajectory.at(trajectory.size() - 1).y;
                }
            } else {
                msg_target.point.x = trajectory.at(1).x;
                msg_target.point.y = trajectory.at(1).y;
            }

            msg_target.point.z = height;
        }
        pub_target.publish(msg_target);

        // Publish Trajectory
        msg_traj.poses.clear();
        for (Position &pos : trajectory)
        {
            msg_traj.poses.push_back(geometry_msgs::PoseStamped()); // insert a posestamped initialised to all 0
            msg_traj.poses.back().pose.position.x = pos.x;
            msg_traj.poses.back().pose.position.y = pos.y;
            msg_traj.poses.back().pose.position.z = height;
        }
        pub_traj.publish(msg_traj);

        if (verbose)
            ROS_INFO_STREAM(" HMAIN : " << to_string(state));

        rate.sleep();
    }

    nh.setParam("run", false); // turns off other nodes
    ROS_INFO(" HMAIN : ===== END =====");
    return 0;
}

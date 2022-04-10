#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <limits>
#include <errno.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <fstream>
#include <signal.h>
#include "common.hpp"
#include <string>
#define NaN std::numeric_limits<double>::quiet_NaN()

double sat(double x, double x2)
{
    if (x > x2)
    {
        return x2;
    }
    else if (x < -x2)
    {
        return -x2;
    }
    else
    {
        return x;
    }
}

ros::ServiceClient en_mtrs;
void disable_motors(int sig)
{
    ROS_INFO(" HMOVE : Disabling motors...");
    hector_uav_msgs::EnableMotors en_mtrs_srv;
    en_mtrs_srv.request.enable = false;
    en_mtrs.call(en_mtrs_srv);
}

double target_x = NaN, target_y = NaN, target_z = NaN;
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target_x = msg->point.x;
    target_y = msg->point.y;
    target_z = msg->point.z;
}

double x = NaN, y = NaN, z = NaN, a = NaN;
void cbPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
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

bool rotate = false;
void cbRotate(const std_msgs::Bool::ConstPtr &msg)
{
    rotate = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;

    // --------- parse parameters ----------
    bool enable_move;
    if (!nh.param("enable_move", enable_move, true))
        ROS_WARN(" HMOVE : Param enable_move not found, set to true");
    if (!enable_move)
        return 0;
    bool verbose;
    if (!nh.param("verbose_move", verbose, false))
        ROS_WARN(" HMOVE : Param verbose_move not found, set to false");
    double Kp_lin;
    if (!nh.param("Kp_lin", Kp_lin, 1.0))
        ROS_WARN(" HMOVE : Param Kp_lin not found, set to 1.0");
    double Ki_lin;
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
        ROS_WARN(" HMOVE : Param Ki_lin not found, set to 0");
    double Kd_lin;
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
        ROS_WARN(" HMOVE : Param Kd_lin not found, set to 0");
    double Kp_z;
    if (!nh.param("Kp_z", Kp_z, 1.0))
        ROS_WARN(" HMOVE : Param Kp_z not found, set to 1.0");
    double Ki_z;
    if (!nh.param("Ki_z", Ki_z, 0.0))
        ROS_WARN(" HMOVE : Param Ki_lin not found, set to 0");
    double Kd_z;
    if (!nh.param("Kd_z", Kd_z, 0.0))
        ROS_WARN(" HMOVE : Param Kd_lin not found, set to 0");
    double yaw_rate;
    if (!nh.param("yaw_rate", yaw_rate, 0.5))
        ROS_WARN(" HMOVE : Param yaw_rate not found, set to 0.5");
    double max_lin_vel;
    if (!nh.param("max_lin_vel", max_lin_vel, 2.0))
        ROS_WARN(" HMOVE : Param max_lin_vel not found, set to 2");
    double max_z_vel;
    if (!nh.param("max_z_vel", max_z_vel, 0.5))
        ROS_WARN(" HMOVE : Param max_z_vel not found, set to 0.5");
    double move_iter_rate;
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
        ROS_WARN(" HMOVE : Param move_iter_rate not found, set to 25");

    // --------- Enable Motors ----------
    ROS_INFO(" HMOVE : Enabling motors...");
    en_mtrs = nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
    hector_uav_msgs::EnableMotors en_mtrs_srv;
    en_mtrs_srv.request.enable = true;
    if (en_mtrs.call(en_mtrs_srv))
        ROS_INFO(" HMOVE : Motors enabled!");
    else
        ROS_WARN(" HMOVE : Cannot enable motors!");
    signal(SIGINT, disable_motors);

    // --------- Subscribers ----------
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);
    ros::Subscriber sub_rotate = nh.subscribe("rotate", 1, &cbRotate);

    // --------- Publishers ----------
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.

    // --------- Wait for Topics ----------
    ROS_INFO(" HMOVE : Waiting for topics");
    while (ros::ok() && nh.param("run", true) && (std::isnan(target_x) || std::isnan(x))) // not dependent on main.cpp, but on motion.cpp
        ros::spinOnce();                                                                  // update the topics

    // --------- Begin Controller ----------
    ROS_INFO(" HMOVE : ===== BEGIN =====");
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic
    double cmd_lin_vel_x, cmd_lin_vel_y, cmd_lin_vel_z, cmd_lin_vel_a;
    double dt;
    double prev_time = ros::Time::now().toSec();

    // Setup Variables
    cv::Matx31d WORLD_FRAME_XYZ;
    cv::Matx31d SRC_NED_XYZ;
    cv::Matx31d TARGET_NED_XYZ;
    cv::Matx33d R = {
        1, 0, 0,
        0, -1, 0,
        0, 0, -1};
    
    double BODY_FRAME_YAW = 0;

    double error_lin_z;
    double error_lin_z_prev = 0;
    double error_lin_z_sum = 0;
    double P_lin_z = 0;
    double I_lin_z = 0;
    double D_lin_z = 0;
    double U_lin_z = 0;
    double lin_acc_z;
    double cmd_lin_vel_z_prev = 0;

    double error_lin_x;
    double error_lin_x_prev = 0;
    double error_lin_x_sum = 0;
    double P_lin_x = 0;
    double I_lin_x = 0;
    double D_lin_x = 0;
    double U_lin_x = 0;
    double lin_acc_x;
    double cmd_lin_vel_x_prev = 0;

    double error_lin_y;
    double error_lin_y_prev = 0;
    double error_lin_y_sum = 0;
    double P_lin_y = 0;
    double I_lin_y = 0;
    double D_lin_y = 0;
    double U_lin_y = 0;
    double lin_acc_y;
    double cmd_lin_vel_y_prev = 0;

    double error_ang_z;
    double error_ang_z_prev = 0;
    double error_ang_z_sum = 0;
    double P_ang_z = 0;
    double I_ang_z = 0;
    double D_ang_z = 0;
    double U_ang_z = 0;
    double ang_acc_z;
    double cmd_ang_vel_z_prev = 0;

    double cmd_lin_vel = 0;
    double error_lin = 0;

    // Log File
    std::vector<std::string> log_array;
    double count = 0;

    // main loop
    while (ros::ok() && nh.param("run", true))
    {
        // update all topics
        ros::spinOnce();

        dt = ros::Time::now().toSec() - prev_time;
        if (dt == 0) // ros doesn't tick the time fast enough
            continue;
        prev_time += dt;

        ////////////////// MOTION CONTROLLER HERE //////////////////
        // Convert coords for source and target from world frame to NED 
        WORLD_FRAME_XYZ = {x, y, z};
        SRC_NED_XYZ = R * WORLD_FRAME_XYZ;
        TARGET_NED_XYZ = R * cv::Matx31d({target_x, target_y, target_z});
        ROS_INFO_STREAM("[COORDS]" <<"WORLD_FRAME_XYZ" << WORLD_FRAME_XYZ << " SRC_NED_XYZ " << SRC_NED_XYZ << " TARGET_NED_XYZ" << TARGET_NED_XYZ);
        BODY_FRAME_YAW = -a;

        error_lin = dist_euc(target_x, target_y, x,y);
        // Calculate errors from
        error_lin_x = cos(-a) * (target_x - x) - sin(-a) * (target_y - y);
        error_lin_y = sin(-a) * (target_x - x) + cos(-a) * (target_y - y);
        error_lin_z = target_z - z;
        // pos_error = dist_euc(x, y, target_x, target_y);


        // LIN_Z
        error_lin_z_sum += error_lin_z * dt;
        P_lin_z = Kp_z * error_lin_z;
        I_lin_z = I_lin_z + (Ki_z * error_lin_z);
        D_lin_z = Kd_z * ((error_lin_z - error_lin_z_prev) / dt);
        U_lin_z = P_lin_z + I_lin_z + D_lin_z;
        error_lin_z_prev = error_lin_z;

        lin_acc_z = (U_lin_z - cmd_lin_vel_z_prev) / dt;
        cmd_lin_vel_z_prev = cmd_lin_vel_z;
        cmd_lin_vel_z = sat((U_lin_z + lin_acc_z * dt), max_z_vel);

        // LIN_X
        error_lin_x_sum += error_lin_x * dt;
        P_lin_x = Kp_lin * error_lin_x;
        I_lin_x = I_lin_x + (Ki_lin * error_lin_x);
        D_lin_x = Kd_lin * ((error_lin_x - error_lin_x_prev) / dt);
        U_lin_x = P_lin_x + I_lin_x + D_lin_x;
        error_lin_x_prev = error_lin_x;

        lin_acc_x = (U_lin_x - cmd_lin_vel_x_prev) / dt;
        cmd_lin_vel_x_prev = cmd_lin_vel_x;
        cmd_lin_vel_x = (U_lin_x + lin_acc_x * dt);

        ROS_INFO_STREAM("LIN_X_ERROR" << error_lin_x << " P_lin_x" << P_lin_x << " I_lin_x" << I_lin_x << " D_lin_x" << D_lin_x);

        // LIN_Y
        error_lin_y_sum += error_lin_y * dt;
        P_lin_y = Kp_lin * error_lin_y;
        I_lin_y = I_lin_y + (Ki_lin * error_lin_y);
        D_lin_y = Kd_lin * ((error_lin_y - error_lin_y_prev) / dt);
        U_lin_y = P_lin_y + I_lin_y + D_lin_y;
        error_lin_y_prev = error_lin_y;

        lin_acc_y = (U_lin_y - cmd_lin_vel_y_prev) / dt;
        cmd_lin_vel_y_prev = cmd_lin_vel_y;
        cmd_lin_vel_y = U_lin_y + lin_acc_y * dt;

        // Saturate to Max Speed on Each Axes
        cmd_lin_vel = sqrt(cmd_lin_vel_x*cmd_lin_vel_x + cmd_lin_vel_y*cmd_lin_vel_y);
        if (cmd_lin_vel > max_lin_vel) {
                cmd_lin_vel_x *= max_lin_vel/cmd_lin_vel;
                cmd_lin_vel_y *= max_lin_vel/cmd_lin_vel;
        }

        ROS_INFO_STREAM("LIN_Y_ERROR" << error_lin_y);
        
        // ANG_Z
        cmd_lin_vel_a = rotate ? yaw_rate : 0;

        // Log Error
        std::ostringstream oss;
        oss << "\t" << count << "\t" << error_lin << "\t" << error_lin_x << "\t" << error_lin_y << "\t" << error_lin_z;
        log_array.push_back(oss.str());
        count++;

        // publish speeds
        msg_cmd.linear.x = cmd_lin_vel_x;
        msg_cmd.linear.y = cmd_lin_vel_y;
        msg_cmd.linear.z = cmd_lin_vel_z;
        msg_cmd.angular.z = cmd_lin_vel_a;
        pub_cmd.publish(msg_cmd);

        //// IMPLEMENT /////

        // verbose
        if (verbose)
        {
            // ROS_INFO(" HMOVE : Target(%6.3f, %6.3f, %6.3f) FV(%6.3f) VX(%6.3f) VY(%6.3f) VZ(%7.3f)", target_x, target_y, target_z, cmd_lin_vel, cmd_lin_vel_x, cmd_lin_vel_y, cmd_lin_vel_z);
        }

        // wait for rate
        rate.sleep();
    }

    // Log to Terminal
    for (std::string &str : log_array){
        ROS_INFO_STREAM(str);
    }
    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.linear.y = 0;
    msg_cmd.linear.z = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);

    // disable motors
    ROS_INFO(" HMOVE : Motors Disabled");
    en_mtrs_srv.request.enable = false;
    en_mtrs.call(en_mtrs_srv);

    ROS_INFO(" HMOVE : ===== END =====");

    return 0;
}
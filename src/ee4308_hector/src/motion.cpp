#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <limits>
#include <errno.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> // publish to pose topic
#include <geometry_msgs/Vector3Stamped.h>            // subscribe to magnetic topic
#include <sensor_msgs/Imu.h>                         // subscribe to imu topic
#include <sensor_msgs/NavSatFix.h>                   // subscribe to GPS
#include <hector_uav_msgs/Altimeter.h>               // subscribe to barometer
#include <sensor_msgs/Range.h>                       // subscribe to sonar
#include <nav_msgs/Odometry.h>                       // subscribe to ground truth topic
#include <std_srvs/Empty.h>                          // Service to calrbrate motors
#include <opencv2/core/core.hpp>
#include "common.hpp"
#include <fstream>

#define NaN std::numeric_limits<double>::quiet_NaN()

// global parameters to be read from ROS PARAMs
bool verbose, use_ground_truth, enable_baro, enable_magnet, enable_sonar, enable_gps, tune_covariance;

// others
bool ready = false; // signal to topics to begin

// --------- PREDICTION WITH IMU ----------
const double G = 9.8;
double prev_imu_t = 0;
cv::Matx21d X = {0, 0}, Y = {0, 0}; // see intellisense. This is equivalent to cv::Matx<double, 2, 1>
cv::Matx21d A = {0, 0};
cv::Matx31d Z = {0, 0, 0};
cv::Matx22d P_x = cv::Matx22d::ones(), P_y = cv::Matx22d::zeros();
cv::Matx22d P_a = cv::Matx22d::ones();
cv::Matx33d P_z = cv::Matx33d::ones();
double ua = NaN, ux = NaN, uy = NaN, uz = NaN;
double qa, qx, qy, qz;
// see https://docs.opencv.org/3.4/de/de1/classcv_1_1Matx.html
void cbImu(const sensor_msgs::Imu::ConstPtr &msg)
{
    if (!ready)
    {
        prev_imu_t = msg->header.stamp.toSec();
        return;
    }

    // calculate time
    double imu_t = msg->header.stamp.toSec();
    double imu_dt = imu_t - prev_imu_t;
    prev_imu_t = imu_t;

    // read inputs
    ua = msg->angular_velocity.z;
    ux = msg->linear_acceleration.x;
    uy = msg->linear_acceleration.y;
    uz = msg->linear_acceleration.z;
    
    //// IMPLEMENT IMU ////
    // Jacobian Mat Definition
    // x 
    cv::Matx22d Fx_mat = {1, imu_dt,
                          0, 1};
    cv::Matx22d Wx_mat = {(-0.5)*pow(imu_dt,2)*cos(A(0)), (0.5)*pow(imu_dt,2)*sin(A(0)),
                          (-imu_dt)*cos(A(0)), imu_dt*sin(A(0))};
    cv::Matx22d Qx_mat = {qx, 0,
                          0, qy};
    
    // y
    cv::Matx22d Fy_mat = {1, imu_dt,
                          0, 1};
    cv::Matx22d Wy_mat = {(-0.5)*pow(imu_dt,2)*cos(A(0)), (-0.5)*pow(imu_dt,2)*sin(A(0)),
                          -imu_dt*cos(A(0)), -imu_dt*sin(A(0))};
    cv::Matx22d Qy_mat = {qy, 0,
                          0, qx};
    
    // z
    cv::Matx33d Fz_mat = {1, imu_dt, 0,
                          0, 1, 0,
                          0, 0, 1};
    cv::Matx31d Wz_mat = {(0.5)*pow(imu_dt,2), imu_dt, 0};

    // a
    cv::Matx22d Fa_mat = {1,0,0,0};
    cv::Matx21d Wa_mat = {imu_dt, 1};
    
    cv::Matx21d Ux_mat = {ux, uy};
    cv::Matx21d Uy_mat = {uy, ux};
    double Uz = uz - G;

    // estimate
    X = Fx_mat * X + Wx_mat * Ux_mat;
    P_x = Fx_mat * P_x * Fx_mat.t() + Wx_mat * Qx_mat * Wx_mat.t();
    Y = Fy_mat * Y + Wy_mat * Uy_mat;
    P_y = Fy_mat * P_y * Fy_mat.t() + Wy_mat * Qy_mat * Wy_mat.t();
    Z = Fz_mat * Z + Wz_mat * Uz;
    P_z = Fz_mat * P_z * Fz_mat.t() + Wz_mat * qz * Wz_mat.t();
    A = Fa_mat * A + Wa_mat * ua;
    P_a = Fa_mat* P_a * Fa_mat.t() + Wa_mat * qa * Wa_mat.t();
}

// --------- GPS ----------
// https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html
cv::Matx31d GPS = {NaN, NaN, NaN};
cv::Matx31d initial_pos = {NaN, NaN, NaN}; // written below in main. no further action needed.
cv::Matx31d initial_ECEF = {NaN, NaN, NaN};
const double DEG2RAD = M_PI / 180;
const double RAD_POLAR = 6356752.3;
const double RAD_EQUATOR = 6378137;
double r_gps_x, r_gps_y, r_gps_z;
double e_square, n, x_e, y_e, z_e;
cv::Matx33d R_en;
cv::Matx33d R_mn = {1,0,0,0,-1,0,0,0,-1};
cv::Matx31d NED = {NaN, NaN, NaN};
cv::Matx31d ECEF = {NaN, NaN, NaN};
double Y_gpsx, Y_gpsy, Y_gpsz, h_X_gpsx, h_X_gpsy, h_X_gpsz, V_gps;
cv::Matx12d H_gps;
cv::Matx13d H_gpsz;
cv::Matx21d K_gpsx, K_gpsy;
cv::Matx31d K_gpsz;
void cbGps(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    if (!ready)
        return;

    //// IMPLEMENT GPS /////
    ROS_INFO("initial_ECEF : %7.3f,%7.3f,%7.3f",initial_ECEF(0),initial_ECEF(1),initial_ECEF(2));
    double lat = DEG2RAD*(msg->latitude);
    double lon = DEG2RAD*(msg->longitude);
    double alt = msg->altitude;

    //read covariance 
    //covariance was read through rostopic

    e_square = 1 - ((RAD_POLAR*RAD_POLAR)/(RAD_EQUATOR*RAD_EQUATOR));
    n = RAD_EQUATOR/(sqrt(1-(e_square*sin(lat)*sin(lat))));
    
    x_e = (n+alt)*cos(lat)*cos(lon);
    y_e = (n+alt)*cos(lat)*sin(lon);
    z_e = ((n*((RAD_POLAR*RAD_POLAR)/(RAD_EQUATOR*RAD_EQUATOR)))+alt)*sin(lat);
    
    R_en = {-sin(lat)*cos(lon), -sin(lon), -cos(lat)*cos(lon),
            -sin(lat)*sin(lon),  cos(lon), -cos(lat)*sin(lon),
             cos(lat),            0,       -sin(lon)};
    ECEF = {x_e, y_e, z_e};

    // for initial message -- you may need this:
    if (std::isnan(initial_ECEF(0)))
    {   // calculates initial ECEF and returns
        initial_ECEF = ECEF;
        return;
    }

    NED = (R_en.t())*(ECEF-initial_ECEF);
    GPS = R_mn*NED + initial_pos;

    ROS_INFO("ned_coordinate : %7.3f,%7.3f,%7.3f",NED(0),NED(1),NED(2));
    ROS_INFO("ECEF : %7.3f,%7.3f,%7.3f",ECEF(0),ECEF(1),ECEF(2));
    

    //define 
    Y_gpsx = GPS(0);
    Y_gpsy = GPS(1);
    Y_gpsz = GPS(2);
    h_X_gpsx = X(0);
    h_X_gpsy = Y(0);
    h_X_gpsz = Z(0);
    H_gps = {1,0};
    H_gpsz = {1, 0, 0};     
    V_gps = 1;

    // correction step
    K_gpsx = P_x * H_gps.t() * (1/((H_gps * P_x * H_gps.t())(0) + V_gps * r_gps_x * V_gps));
    K_gpsy = P_y * H_gps.t() * (1/((H_gps * P_y * H_gps.t())(0) + V_gps * r_gps_y * V_gps));
    K_gpsz = P_z * H_gpsz.t() * (1/((H_gpsz * P_z * H_gpsz.t())(0) + V_gps * r_gps_z * V_gps));
    
    X = X + K_gpsx * (Y_gpsx - h_X_gpsx);
    Y = Y + K_gpsy * (Y_gpsy - h_X_gpsy);
    Z = Z + K_gpsz * (Y_gpsz - h_X_gpsz);

    P_x = P_x - (K_gpsx * H_gps * P_x);
    P_y = P_y - (K_gpsy * H_gps * P_y);
    P_z = P_z - (K_gpsz * H_gpsz * P_z);
    
}

// --------- Magnetic ----------
double a_mgn = NaN;
double r_mgn_a;
std::vector<double> yawlist;
void cbMagnet(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
{
    if (!ready)
        return;
    
    // IMPLEMENT GPS ////
    double mx = msg->vector.x;
    double my = msg->vector.y;
    
    
    a_mgn = atan2(-my,mx);
    
    //Variance calculation
    yawlist.push_back(a_mgn);
    //calculate variance every 100
    if(yawlist.size() >= 100 && tune_covariance){
        r_mgn_a = variance(yawlist);
        yawlist.clear();
    }

    ROS_INFO("magnetometer : %7.3f,%7.3f,%7.3f,%7.3f",msg->vector.x,msg->vector.y,r_mgn_a,a_mgn);

    //define
    double Y_mag = a_mgn;
    double h_X_mag = A(0);
    cv::Matx12d H_mag = {1,0};
    double V_mag = 1;
    double R_mag = r_mgn_a;

    //correction step 
    cv::Matx21d K_mag;
    K_mag = P_a * H_mag.t() * (1/(((H_mag * P_a * H_mag.t())(0) + V_mag * R_mag * V_mag)));
    A = A + K_mag * (Y_mag - h_X_mag);
    P_a = P_a - K_mag * H_mag * P_a;

}

// --------- Baro ----------
double z_bar = NaN;
double r_bar_z;
std::vector<double> z_bar_list;
void cbBaro(const hector_uav_msgs::Altimeter::ConstPtr &msg)
{
    if (!ready)
        return;

    //// IMPLEMENT BARO ////
    z_bar = msg->altitude;
    z_bar_list.push_back(z_bar);
    //calculate varience every 100
    if (z_bar_list.size() >= 100 && tune_covariance) {
        r_bar_z = variance(z_bar_list); 
        z_bar_list.clear();
    }
    // define
    double Y_bar = z_bar;
    double h_X_bar = Z(0);
    cv::Matx13d H_bar = {1, 0, 1};
    double V_bar = 1;
    double R_bar = r_bar_z;

    // correction
    cv::Matx31d K_bar;
    K_bar = P_z * H_bar.t() * (1/((H_bar * P_z * H_bar.t())(0) + V_bar * R_bar * V_bar));
    ROS_WARN("bias: %f, k_BAR: %f %f %f", Z(2), K_bar(0), K_bar(1), K_bar(2));
    Z = Z + K_bar * (Y_bar - h_X_bar - Z(2));
    P_z = P_z - K_bar * H_bar * P_z;
}

// --------- Sonar ----------
double z_snr = NaN;
double r_snr_z;
double z_snr_old;
std::vector<double> sonar_list;
void cbSonar(const sensor_msgs::Range::ConstPtr &msg)
{
    if (!ready)
        return;

    //// IMPLEMENT SONAR ////
    z_snr = msg->range;
    ROS_INFO_STREAM("[HM] SONAR READING " << z_snr );
    if(sonar_list.size() != 0 && mean(sonar_list)){
        z_snr = 0.3*mean(sonar_list) + 0.7*z_snr;
        ROS_INFO_STREAM("[HM] SONAR WEIGHTED AVERAGE " << z_snr);
    }
    if (sonar_list.size() < 100) {
        sonar_list.push_back(z_snr);
    } else {
        sonar_list.erase(sonar_list.begin());
        sonar_list.push_back(z_snr);
    }
     

    //check if the value is changing more than min height
    //height is 2m lowest for obstacle
  

    //calculate varience every 100
    if (sonar_list.size() >= 100 && tune_covariance) {
        r_snr_z = variance(sonar_list); 
        sonar_list.clear();
    }

    z_snr_old = z_snr;
    ROS_WARN(" varience_sonar: %f ",  r_snr_z);
    ROS_WARN(" mean_sonar: %f ",  mean(sonar_list));
    
    //define
    double Y_snr = z_snr;
    double h_X_snr = Z(0);
    cv::Matx13d H_snr = {1, 0 , 0};
    double V_snr = 1;
    double R_snr = r_snr_z;

    //correction step
    cv::Matx31d K_snr;
    K_snr = P_z * H_snr.t() * (1/(((H_snr * P_z * H_snr.t())(0) + V_snr * R_snr * V_snr)));
    Z = Z + K_snr * (Y_snr - h_X_snr);
    P_z = P_z - K_snr * H_snr * P_z;
}

// --------- GROUND TRUTH ----------
nav_msgs::Odometry msg_true;
void cbTrue(const nav_msgs::Odometry::ConstPtr &msg)
{
    msg_true = *msg;
}

// --------- MEASUREMENT UPDATE WITH GROUND TRUTH ----------
int main(int argc, char **argv)
{
    std::ofstream data_file;
    data_file.open("/home/zccccclin/project_2/data.txt");
    data_file << "r_gps_x" << "\t" << "r_gps_y" << "\t" << "r_gps_z" << "\t" << "r_mgn_a" << "\t" << "r_bar_z" << "\t" << "r_snr_z" << std::endl;


    ros::init(argc, argv, "hector_motion");
    ros::NodeHandle nh;

    // --------- parse parameters ----------
    double motion_iter_rate;
    if (!nh.param("motion_iter_rate", motion_iter_rate, 50.0))
        ROS_WARN("HMOTION: Param motion_iter_rate not found, set to 50.0");
    if (!nh.param("verbose_motion", verbose, false))
        ROS_WARN("HMOTION: Param verbose_motion not found, set to false");
    if (!nh.param("initial_x", X(0), 0.0))
        ROS_WARN("HMOTION: Param initial_x not found, set initial_x to 0.0");
    if (!nh.param("initial_y", Y(0), 0.0))
        ROS_WARN("HMOTION: Param initial_y not found, set initial_y to 0.0");
    if (!nh.param("initial_z", Z(0), 0.178))
        ROS_WARN("HMOTION: Param initial_z not found, set initial_z to 0.178");
    initial_pos = {X(0), Y(0), Z(0)};
    if (!nh.param("use_ground_truth", use_ground_truth, true))
        ROS_WARN("HMOTION: Param use_ground_truth not found, set use_ground_truth to true");
    if (!nh.param("r_gps_x", r_gps_x, 1.0))
        ROS_WARN("HMOTION: Param r_gps_x not found, set to 1.0");
    if (!nh.param("r_gps_y", r_gps_y, 1.0))
        ROS_WARN("HMOTION: Param r_gps_y not found, set to 1.0");
    if (!nh.param("r_gps_z", r_gps_z, 1.0))
        ROS_WARN("HMOTION: Param r_gps_z not found, set to 1.0");
    if (!nh.param("r_mgn_a", r_mgn_a, 1.0))
        ROS_WARN("HMOTION: Param r_mgn_a not found, set to 1.0");
    if (!nh.param("r_bar_z", r_bar_z, 1.0))
        ROS_WARN("HMOTION: Param r_bar_z not found, set to 1.0");
    if (!nh.param("r_snr_z", r_snr_z, 1.0))
        ROS_WARN("HMOTION: Param r_snr_z not found, set to 1.0");
    if (!nh.param("qa", qa, 1.0))
        ROS_WARN("HMOTION: Param qa not found, set to 1.0");
    if (!nh.param("qx", qx, 1.0))
        ROS_WARN("HMOTION: Param qx not found, set to 1.0");
    if (!nh.param("qy", qy, 1.0))
        ROS_WARN("HMOTION: Param qy not found, set to 1.0");
    if (!nh.param("qz", qz, 1.0))
        ROS_WARN("HMOTION: Param qz not found, set to 1.0");
    if (!nh.param("enable_baro", enable_baro, true))
        ROS_WARN("HMOTION: Param enable_baro not found, set to true");
    if (!nh.param("enable_magnet", enable_magnet, true))
        ROS_WARN("HMOTION: Param enable_magnet not found, set to true");
    if (!nh.param("enable_sonar", enable_sonar, true))
        ROS_WARN("HMOTION: Param enable_sonar not found, set to true");
    if (!nh.param("enable_gps", enable_gps, true))
        ROS_WARN("HMOTION: Param enable_gps not found, set to true");
    if (!nh.param("tune_covariance", tune_covariance, false))
        ROS_WARN("HMOTION: Param tune_covariance not found, set to false");

    // --------- Subscribers ----------
    ros::Subscriber sub_true = nh.subscribe<nav_msgs::Odometry>("ground_truth/state", 1, &cbTrue);
    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("raw_imu", 1, &cbImu);
    ros::Subscriber sub_gps = nh.subscribe<sensor_msgs::NavSatFix>("fix", 1, &cbGps);
    if (!enable_gps)
        sub_gps.shutdown();
    ros::Subscriber sub_magnet = nh.subscribe<geometry_msgs::Vector3Stamped>("magnetic", 1, &cbMagnet);
    if (!enable_magnet)
        sub_magnet.shutdown();
    ros::Subscriber sub_baro = nh.subscribe<hector_uav_msgs::Altimeter>("altimeter", 1, &cbBaro);
    if (!enable_baro)
        sub_baro.shutdown();
    ros::Subscriber sub_sonar = nh.subscribe<sensor_msgs::Range>("sonar_height", 1, &cbSonar);
    if (!enable_sonar)
        sub_sonar.shutdown();

    // --------- Publishers ----------
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 1, true);
    geometry_msgs::PoseWithCovarianceStamped msg_pose;
    msg_pose.header.frame_id = "world";   // for rviz
    msg_pose.pose.pose.orientation.x = 0; // no roll
    msg_pose.pose.pose.orientation.y = 0; // no pitch
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("velocity", 1, true); // publish velocity
    geometry_msgs::Twist msg_vel;

    // --------- Wait for Topics ----------
    ROS_INFO("HMOTION: Waiting for topics");
    while (ros::ok() && nh.param("run", true) && ((std::isnan(ux) && msg_true.header.seq == 0))) // wait for imu and truth only
        ros::spinOnce(); // update subscribers

    if (!ros::ok())
    { // ROS shutdown
        ROS_INFO("HMOTION: ===== END =====");
        return 0;
    }

    // --------- Calibrate Gyro service ----------
    ROS_INFO("HMOTION: Calibrating Gyro...");
    ros::ServiceClient calibrate_gyro = nh.serviceClient<std_srvs::Empty>("raw_imu/calibrate");
    std_srvs::Empty calibrate_gyro_srv;
    if (calibrate_gyro.call(calibrate_gyro_srv))
        ROS_INFO("HMOTION: Calibrated Gyro");
    else
        ROS_WARN("HMOTION: Gyro cannot be calibrated!");

    // --------- Main loop ----------

    ros::Rate rate(motion_iter_rate);
    ROS_INFO("HMOTION: ===== BEGIN =====");
    ready = true;
    while (ros::ok() && nh.param("run", true))
    {
        ros::spinOnce(); // update topics

        // Verbose
        if (verbose)
        {   
            double sum_error_x ;
            double sum_error_y ; 
            double sum_error_z ; 
            double sum_error_a ; 
            double error_x ;
            double error_y ; 
            double error_z ; 
            double error_a ; 
            double iter; 
            
            auto & tp = msg_true.pose.pose.position;
            auto &q = msg_true.pose.pose.orientation;
            double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            error_x = tp.x-X(0);
            error_y = tp.y-Y(0);
            error_z= tp.z-Z(0);
            error_a = atan2(siny_cosp, cosy_cosp)-A(0);
            sum_error_x += error_x;
            sum_error_y += error_y;
            sum_error_z += error_z;
            sum_error_a += error_a;

            ROS_INFO("[HM] ---------X-------Y-------Z-------A------");
            ROS_INFO("[HM]  TRUE(%7.3lf,%7.3lf,%7.3lf,%6.3lf)", tp.x, tp.y, tp.z, atan2(siny_cosp, cosy_cosp));
            
            ROS_INFO("[HM] ERROR(%7.3lf,%7.3lf,%7.3lf,%6.3lf)", error_x, error_y, error_z,error_a);
            ROS_INFO("[HM] SUM_ERROR(%7.3lf,%7.3lf,%7.3lf,%6.3lf)", sum_error_x, sum_error_y, sum_error_z, sum_error_a);
            ROS_INFO("[HM] MEAN_ERROR(%7.3lf,%7.3lf,%7.3lf,%6.3lf)", sum_error_x/iter, sum_error_y/iter, sum_error_z/iter, sum_error_a/iter);
            ROS_INFO("[HM] STATE(%7.3lf,%7.3lf,%7.3lf,%6.3lf)", X(0), Y(0), Z(0), A(0));
            ROS_INFO("[HM]   GPS(%7.3lf,%7.3lf,%7.3lf, ---- )", GPS(0), GPS(1), GPS(2));
            ROS_INFO("[HM] MAGNT( ----- , ----- , ----- ,%6.3lf)", a_mgn);
            ROS_INFO("[HM]  BARO( ----- , ----- ,%7.3lf, ---- )", z_bar);
            ROS_INFO("[HM] BAROC( ----- , ----- ,%7.3lf, ---- )", z_bar - Z(2));
            ROS_INFO("[HM] BAROB( ----- , ----- ,%7.3lf, ---- )", Z(2));
            ROS_INFO("[HM] SONAR( ----- , ----- ,%7.3lf, ---- )", z_snr);
            if (tune_covariance){
                data_file << r_gps_x << "\t" << r_gps_y << "\t" << r_gps_z << "\t" << r_mgn_a << "\t" << r_bar_z << "\t" << r_snr_z << std::endl;
            }
            iter ++;
        }

        //  Publish pose and vel
        if (use_ground_truth)
        {
            msg_pose.header.stamp = ros::Time::now();
            msg_pose.pose.pose.position = msg_true.pose.pose.position;
            msg_pose.pose.pose.orientation = msg_true.pose.pose.orientation;
            msg_vel = msg_true.twist.twist;
        }
        else
        {
            msg_pose.header.stamp = ros::Time::now();
            msg_pose.pose.pose.position.x = X(0);
            msg_pose.pose.pose.position.y = Y(0);
            msg_pose.pose.pose.position.z = Z(0);
            msg_pose.pose.covariance[0] = P_x(0, 0);  // x cov
            msg_pose.pose.covariance[7] = P_y(0, 0);  // y cov
            msg_pose.pose.covariance[14] = P_z(0, 0); // z cov
            msg_pose.pose.covariance[35] = P_a(0, 0); // a cov
            msg_pose.pose.pose.orientation.w = cos(A(0) / 2);
            msg_pose.pose.pose.orientation.z = sin(A(0) / 2);
            msg_vel.linear.x = X(1);
            msg_vel.linear.y = Y(1);
            msg_vel.linear.z = Z(1);
            msg_vel.angular.z = A(1);
        }
        pub_pose.publish(msg_pose);
        pub_vel.publish(msg_vel);

        rate.sleep();
    }

    ROS_INFO("HMOTION: ===== END =====");
    data_file.close();
    return 0;
}

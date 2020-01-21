#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <Eigen/Dense>
#include <math.h>
#include <tf/tf.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Range.h>

#include <dynamic_reconfigure/server.h>

#define CAMERA_V_FOV 43.2
#define ALLOWED_MIN_Z 1.0
#define LIDAR_FILTER_WINDOW 5
#define SONAR_FILTER_WINDOW 5
#define WEIGHT_LIDAR 0.7
#define WEIGHT_SONAR 0.3 
#define MAX_ALLOWED_YAWRATE 0.1
#define DEG_BET_EACH_LIDAR 10.0
#define TIME_BEFORE_LAND_END 3.0 
#define CLIMB_RATE 0.4
#define LAND_RATE 0.4


using namespace geometry_msgs;
using namespace std;
using namespace ros;

const double deg2rad = M_PI/180;
const double rad2deg = 180/M_PI;

// Roslaunch param
bool verbose;
std::string mocap_topic, Llidar_topic, Clidar_topic, Rlidar_topic, sonar_topic;

// Publishers
ros::Publisher ref_pos_pub;
ros::Publisher ref_vel_pub;
ros::Publisher ref_yaw_pub;
ros::Publisher setpoint_pos_pub;
ros::Publisher traj_on_pub;
ros::Publisher reg_on_pub;

// Subscriber
ros::Subscriber pos_sub;
ros::Subscriber Llidar_read_sub, Clidar_read_sub, Rlidar_read_sub, sonar_read_sub;

double yaw_hover, wall_dist, x_sp_start, y_sp_start, z_sp_start, x_sp_end, y_sp_end, z_sp_end, length, height, z_interval_dist, z_interval_dist_updated, absvel;
int camera_V_FOV, req_V_overlap_percent, num_turns, sp_z_counter;
bool traj_on, adaptive_yaw_on, reg_on, land_on, pub_setpoint_pos;
bool sp_z_counter_switch, sp_left_corner_reached_flag, sp_right_corner_reached_flag;
bool land_forced = false;

geometry_msgs::PoseStamped pos_ref_start_msg;
geometry_msgs::Vector3 reftrajectory_msg, reftrajectory_vel_msg;
std_msgs::Float64 ref_yaw_msg;
geometry_msgs::PoseStamped setpoint_pos_msg;
tf::Quaternion current_att_quat, setpoint_att_quat;
tf::Matrix3x3  current_att_mat, R_BI, R_IB;
std_msgs::Bool traj_on_msg, reg_on_msg;

bool use_sonar;
double Llidar_read_data, Clidar_read_data, Rlidar_read_data, sonar_read_data, ref_yaw_adaptive = 0.0;
std::vector<double> Llidar_read_data_unfiltered(LIDAR_FILTER_WINDOW, 0.0), Clidar_read_data_unfiltered(LIDAR_FILTER_WINDOW+2, 0.0), Rlidar_read_data_unfiltered(LIDAR_FILTER_WINDOW, 0.0), sonar_read_data_unfiltered(SONAR_FILTER_WINDOW, 0.0);

double x,y,z;
double x_B,y_B;
double x_last = 0, y_last = 0, z_last = 0;
double x_atTrajStart = 0, y_atTrajStart = 0, z_atTrajStart = 0, x_B_atTrajStart = 0, y_B_atTrajStart = 0;
double x_B_sp_start,y_B_sp_start, x_B_sp_end,y_B_sp_end;
double u,v,w;

double t, t_last, traj_time, t_last_eachRun, traj_time_eachRun, traj_time_z, t_last_z, t_Clidar_lost_start, t_Clidar_return_start, t_ALLlidar_lost_start;
bool traj_started_flag = 0, climbed_flag = 0, landed_flag = 0, lidar_started_flag = 0;
int print_flag_traj_on = 0, print_flag_setpoint = 1;
int print_flag_climb = 0, print_flag_land = 0;

bool print_Llidar_nan_flag = 0, print_Clidar_nan_flag = 0, print_Rlidar_nan_flag = 0, print_sonar_nan_flag = 0, print_ALLlidar_nan_flag = 0; 

double compute_ref_yaw();
double compute_actual_wall_dist();

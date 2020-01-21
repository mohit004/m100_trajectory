/**
 * @file   trajectory_final.cpp
 * @author Mohit Mehndiratta
 * @date   July 2019
 *
 * @copyright
 * Copyright (C) 2019.
 */

#include <trajectory_final.h>
#include <m100_trajectory/set_trajectory_finalConfig.h>

double sampleTime = 0.01;

void dynamicReconfigureCallback(m100_trajectory::set_trajectory_finalConfig &config, uint32_t level)
{
    traj_on = config.traj_on;
    adaptive_yaw_on = config.adaptive_yaw_on;
    reg_on = config.reg_on;
    land_on = config.land;
    pub_setpoint_pos = config.pub_on_setpoint_position;

    yaw_hover = config.yaw_hover;
    wall_dist = config.wall_dist;
    x_sp_start = config.x_sp_start;
    y_sp_start = config.y_sp_start;
    z_sp_start = config.z_sp_start;
    length = config.length;
    height = config.height;
    req_V_overlap_percent = config.req_V_overlap_percent;
    absvel = config.des_velocity;
}

// Callback function
std::vector<double> current_pos(3,0.0), current_att(3,0.0);
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pos = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    // TO BE: try to get the Euler angles in one line of code!
    current_att_quat = {msg->pose.orientation.x, msg->pose.orientation.y,
                        msg->pose.orientation.z, msg->pose.orientation.w};
    current_att_mat.setRotation(current_att_quat);
    current_att_mat.getRPY(current_att[0], current_att[1], current_att[2]);
}

void Llidar_read_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    if (!std::isnan(msg->range) ||
        !std::isnan(Llidar_read_data_unfiltered[Llidar_read_data_unfiltered.size()-1]))
    {
        // erase from the front!
        Llidar_read_data_unfiltered.erase(Llidar_read_data_unfiltered.begin());
        if (msg->range < msg->min_range)
            Llidar_read_data_unfiltered.push_back(Llidar_read_data_unfiltered.back());
        else if (msg->range > msg->max_range)
            Llidar_read_data_unfiltered.push_back(Llidar_read_data_unfiltered.back());
        else
            Llidar_read_data_unfiltered.push_back(msg->range);

        Llidar_read_data = 0.0;
        for (int i=0; i<Llidar_read_data_unfiltered.size(); ++i)
            Llidar_read_data += Llidar_read_data_unfiltered[i];
        Llidar_read_data = Llidar_read_data/(Llidar_read_data_unfiltered.size());
    }
}
void Clidar_read_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    if (!std::isnan(msg->range) ||
        !std::isnan(Clidar_read_data_unfiltered[Clidar_read_data_unfiltered.size()-1]))
    {
        // erase from the front!
        Clidar_read_data_unfiltered.erase(Clidar_read_data_unfiltered.begin());
        if (msg->range < msg->min_range)
            Clidar_read_data_unfiltered.push_back(Clidar_read_data_unfiltered.back());
        else if (msg->range > msg->max_range)
            Clidar_read_data_unfiltered.push_back(Clidar_read_data_unfiltered.back());
        else
            Clidar_read_data_unfiltered.push_back(msg->range);

        Clidar_read_data = 0.0;
        for (int i=0; i<Clidar_read_data_unfiltered.size(); ++i)
            Clidar_read_data += Clidar_read_data_unfiltered[i];
        Clidar_read_data = Clidar_read_data/(Clidar_read_data_unfiltered.size());
    }
}
void Rlidar_read_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    if (!std::isnan(msg->range) ||
        !std::isnan(Rlidar_read_data_unfiltered[Rlidar_read_data_unfiltered.size()-1]))
    {
        // erase from the front!
        Rlidar_read_data_unfiltered.erase(Rlidar_read_data_unfiltered.begin());
        if (msg->range < msg->min_range)
            Rlidar_read_data_unfiltered.push_back(Rlidar_read_data_unfiltered.back());
        else if (msg->range > msg->max_range)
            Rlidar_read_data_unfiltered.push_back(Rlidar_read_data_unfiltered.back());
        else
            Rlidar_read_data_unfiltered.push_back(msg->range);

        Rlidar_read_data = 0.0;
        for (int i=0; i<Rlidar_read_data_unfiltered.size(); ++i)
            Rlidar_read_data += Rlidar_read_data_unfiltered[i];
        Rlidar_read_data = Rlidar_read_data/(Rlidar_read_data_unfiltered.size());
    }
}

void sonar_read_cb(const sensor_msgs::Range::ConstPtr& msg)
{
    if (!std::isnan(msg->range) ||
        !std::isnan(sonar_read_data_unfiltered[sonar_read_data_unfiltered.size()-1]))
    {
        // erase from the front!
        sonar_read_data_unfiltered.erase(sonar_read_data_unfiltered.begin());
        if (msg->range < msg->min_range)
            sonar_read_data_unfiltered.push_back(sonar_read_data_unfiltered.back());
        else if (msg->range > msg->max_range)
            sonar_read_data_unfiltered.push_back(sonar_read_data_unfiltered.back());
        else
            sonar_read_data_unfiltered.push_back(msg->range);

        sonar_read_data = 0.0;
        for (int i=0; i<sonar_read_data_unfiltered.size(); ++i)
            sonar_read_data += sonar_read_data_unfiltered[i];
        sonar_read_data = sonar_read_data/(sonar_read_data_unfiltered.size());
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<m100_trajectory::set_trajectory_finalConfig> server;
    dynamic_reconfigure::Server<m100_trajectory::set_trajectory_finalConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    // Roslaunch parameters
    ros::param::get("verbose_traj", verbose);
    ros::param::get("mocap_topic", mocap_topic);
    ros::param::get("Llidar_topic", Llidar_topic);
    ros::param::get("Clidar_topic", Clidar_topic);
    ros::param::get("Rlidar_topic", Rlidar_topic);
    ros::param::get("use_sonar", use_sonar);
    ros::param::get("sonar_topic", sonar_topic);

    // Publisher
    ref_pos_pub = nh.advertise<geometry_msgs::Vector3>("ref_trajectory/pose", 1);
    ref_vel_pub = nh.advertise<geometry_msgs::Vector3>("ref_trajectory/velocity", 1);
    ref_yaw_pub = nh.advertise<std_msgs::Float64>("ref_trajectory/yaw", 1);
    setpoint_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
    traj_on_pub = nh.advertise<std_msgs::Bool>("trajectory_on", 1);
    reg_on_pub = nh.advertise<std_msgs::Bool>("regression_on", 1);

    // Subscriber
    pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(mocap_topic, 1, pos_cb);
    Llidar_read_sub = nh.subscribe<sensor_msgs::Range>(Llidar_topic, 1, Llidar_read_cb);
    Clidar_read_sub = nh.subscribe<sensor_msgs::Range>(Clidar_topic, 1, Clidar_read_cb);
    Rlidar_read_sub = nh.subscribe<sensor_msgs::Range>(Rlidar_topic, 1, Rlidar_read_cb);
    if (use_sonar)
        sonar_read_sub = nh.subscribe<sensor_msgs::Range>(sonar_topic, 1, sonar_read_cb);

    ros::Rate rate(1/sampleTime);

    pos_ref_start_msg.pose.position.x = 0;
    pos_ref_start_msg.pose.position.y = 0;
    pos_ref_start_msg.pose.position.z = 0;

    while(ros::ok())
    {
        traj_on_msg.data = traj_on;
        reg_on_msg.data = traj_on ? reg_on : false;

        if(pos_ref_start_msg.pose.position.z != 0.0 && !traj_on)
        {
            pos_ref_start_msg.pose.position.z = 0.0;
            print_flag_traj_on = 0;
        }

        if(traj_on && !landed_flag)
        {
            if(!traj_started_flag)
                traj_started_flag = true;

            t = ros::Time::now().toSec();
            traj_time = t - t_last;

            while(ros::ok() && traj_on && !land_on && !climbed_flag && z < z_sp_start)
            {
                if (print_flag_climb == 0)
                {
                    ROS_INFO("---------------------------------");
                    ROS_INFO("Climbing initialized!");
                    print_flag_climb = 1;
                }
                x = x;
                y = y;
                z = z < z_sp_start ? z + CLIMB_RATE*sampleTime : z_sp_start;

                u = (x - x_last)/sampleTime;
                v = (y - y_last)/sampleTime;
                w = (z - z_last)/sampleTime;

                ref_yaw_msg.data = compute_ref_yaw();
                if(!pub_setpoint_pos)
                {
                    reftrajectory_msg.x = x;
                    reftrajectory_msg.y = y;
                    reftrajectory_msg.z = z;
                    ref_pos_pub.publish(reftrajectory_msg);

                    reftrajectory_vel_msg.x = std::abs(u) <= absvel ? u : (u < 0 ? -absvel : absvel);
                    reftrajectory_vel_msg.y = std::abs(v) <= absvel ? v : (v < 0 ? -absvel : absvel);
                    reftrajectory_vel_msg.z = std::abs(w) <= absvel ? w : (w < 0 ? -absvel : absvel);
                    ref_vel_pub.publish(reftrajectory_vel_msg);

                    ref_yaw_pub.publish(ref_yaw_msg);
                }
                else
                {
                    setpoint_pos_msg.header.stamp = ros::Time::now();
                    setpoint_pos_msg.pose.position.x = x;
                    setpoint_pos_msg.pose.position.y = y;
                    setpoint_pos_msg.pose.position.z = z;

                    setpoint_att_quat.setRPY(0,0,ref_yaw_msg.data);
                    setpoint_pos_msg.pose.orientation.x = setpoint_att_quat.getX();
                    setpoint_pos_msg.pose.orientation.y = setpoint_att_quat.getY();
                    setpoint_pos_msg.pose.orientation.z = setpoint_att_quat.getZ();
                    setpoint_pos_msg.pose.orientation.w = setpoint_att_quat.getW();
                    setpoint_pos_pub.publish(setpoint_pos_msg);
                }

                if(z >= z_sp_start)
                {
                    if (print_flag_climb == 1)
                    {
                        ROS_INFO("Climbing complete!");
                        print_flag_climb = 0;
                    }
                    pos_ref_start_msg.pose.position.z = z_sp_start;
                    climbed_flag = true;
                    landed_flag = false;
                    t_last = ros::Time::now().toSec();
                }

                x_last = x;
                y_last = y;
                z_last = z;

                traj_on_pub.publish(traj_on_msg);
                reg_on_pub.publish(reg_on_msg);
                ros::spinOnce();
                rate.sleep();
            }

            if(climbed_flag && !landed_flag)
            {
                pos_ref_start_msg.pose.position.z = z_sp_start;
            }
            else
                continue;       // to get out of the if command!

            if (print_flag_traj_on == 1)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Reference trajectory started!");
                print_flag_traj_on = 0;
            }

            if (print_flag_setpoint == 1)
            {
                t_last = ros::Time::now().toSec();
                t_last_eachRun = ros::Time::now().toSec();

                ROS_INFO("--------Setpoint selected!--------");
                print_flag_setpoint = 0;

                x = x_sp_start;
                y = y_sp_start;
                z = z_sp_start;

                setpoint_att_quat.setRPY(0,0,ref_yaw_msg.data);
                R_IB.setRotation(setpoint_att_quat);
                R_BI = R_IB.transpose();

                if (verbose)
                {
                    std::cout<<"R_BI = ";
                    for (int i=0; i<3; i++)
                    {
                        for (int j=0; j<3; j++)
                            std::cout<<R_BI[i][j]<<", ";
                        std::cout<<"\n";
                    }
                    std::cout<<"R_IB = ";
                    for (int i=0; i<3; i++)
                    {
                        for (int j=0; j<3; j++)
                            std::cout<<R_IB[i][j]<<", ";
                        std::cout<<"\n";
                    }
                }

                // TO BE: checked!
                x_sp_end = x_sp_start - R_IB[0][1]*length;
                y_sp_end = y_sp_start - R_IB[1][1]*length;
                z_sp_end = z_sp_start - height;
                if (z_sp_end < ALLOWED_MIN_Z)
                {
                    ROS_WARN("Travel height is set to more than allowed!");
                    ROS_WARN("Drone will go to the lowest allowed z = %f m!", ALLOWED_MIN_Z);
                    z_sp_end = ALLOWED_MIN_Z;
                }

                x_B_sp_start = R_BI[0][0]*x + R_BI[0][1]*y;
                y_B_sp_start = R_BI[1][0]*x + R_BI[1][1]*y;
                x_B = x_B_sp_start;
                y_B = y_B_sp_start;

                x_B_sp_end = R_BI[0][0]*x_sp_end + R_BI[0][1]*y_sp_end;
                y_B_sp_end = R_BI[1][0]*x_sp_end + R_BI[1][1]*y_sp_end;

                z_interval_dist = wall_dist * tan(deg2rad*(1.0 - (double)req_V_overlap_percent/100)*CAMERA_V_FOV/2);
                num_turns = (int)(std::abs(z_sp_start-z_sp_end)/z_interval_dist);
                if (num_turns % 2 != 0)
                    z_interval_dist_updated = std::abs(z_sp_start-z_sp_end)/++num_turns;
                else
                    z_interval_dist_updated = z_interval_dist;
                sp_left_corner_reached_flag = true;
                sp_right_corner_reached_flag = false;
                sp_z_counter = 0;
                sp_z_counter_switch = true;

                if (verbose)
                {
                    std::cout<<"num_turns = "<<num_turns<<"\n";
                    std::cout<<"z_interval_dist = "<<z_interval_dist<<"\n";
                    std::cout<<"z_interval_dist_updated = "<<z_interval_dist_updated<<"\n";
                }

                x_last = x;
                y_last = y;
                z_last = z;
            }

            traj_time_eachRun = ros::Time::now().toSec() - t_last_eachRun;

            x_B = x_B_sp_start + (compute_actual_wall_dist()-wall_dist)
                               + (R_BI[0][0]*current_pos[0] +
                                  R_BI[0][1]*current_pos[1]);
            if (z >= z_sp_end)
            {
                if ((y_B > y_B_sp_end) && sp_left_corner_reached_flag && !sp_right_corner_reached_flag)
                {
                    if (sp_z_counter_switch)
                    {
                        if (verbose)
                            std::cout<<"sp_z_counter = "<<sp_z_counter++<<"\n";
                        sp_z_counter_switch = false;
                        t_last_eachRun = ros::Time::now().toSec();
                    }
                    y_B -= absvel*sampleTime;
                    z = z;
                    if (std::abs(y_B-y_B_sp_end)<=absvel*sampleTime &&
                        std::abs(traj_time_eachRun-std::abs(y_B_sp_start-y_B_sp_end)/absvel<1))
                    {
                        if (verbose)
                            std::cout<<"right corner reached!\n";
                        sp_left_corner_reached_flag = false;
                        sp_right_corner_reached_flag = false;
                    }

                }
                else if ((y_B < y_B_sp_start) && !sp_left_corner_reached_flag && sp_right_corner_reached_flag)
                {
                    if (sp_z_counter_switch)
                    {
                        if (verbose)
                            std::cout<<"sp_z_counter = "<<sp_z_counter++<<"\n";
                        sp_z_counter_switch = false;
                        t_last_eachRun = ros::Time::now().toSec();
                    }
                    y_B += absvel*sampleTime;
                    z = z;
                    if (std::abs(y_B-y_B_sp_start)<=absvel*sampleTime &&
                        std::abs(traj_time_eachRun-std::abs(y_B_sp_start-y_B_sp_end)/absvel<1))
                    {
                        if (verbose)
                            std::cout<<"left corner reached!\n";
                        sp_left_corner_reached_flag = false;
                        sp_right_corner_reached_flag = false;
                    }
                }
                else if ((!sp_left_corner_reached_flag && !sp_right_corner_reached_flag) &&
                         z>(z_sp_start-z_interval_dist_updated*sp_z_counter))
                {
                    if (!sp_z_counter_switch)
                    {
                        sp_z_counter_switch = true;
                        t_last_eachRun = ros::Time::now().toSec();
                    }
                    y_B = y_B;
                    z -= absvel*sampleTime;
                    if (std::abs(z-(z_sp_start-z_interval_dist_updated*sp_z_counter))<=absvel*sampleTime)
                    {
                        if (std::abs(y_B-y_B_sp_end)<=absvel*sampleTime)
                        {
                            sp_left_corner_reached_flag = false;
                            sp_right_corner_reached_flag = true;
                        }
                        else if (std::abs(y_B-y_B_sp_start)<=absvel*sampleTime)
                        {
                            sp_left_corner_reached_flag = true;
                            sp_right_corner_reached_flag = false;
                        }
                    }
                }
            }
            else
            {
                if (sp_z_counter_switch)
                {
                    std::cout<<"Final setpoint reached!\n";
                    std::cout<<"Trajectory complete!\n";
                    std::cout<<"Landing will commence in "<<TIME_BEFORE_LAND_END<<" seconds!\n";
                    sp_z_counter_switch = false;
                    t_last_eachRun = ros::Time::now().toSec();
                }
                y_B = y_B;
                if (traj_time_eachRun < TIME_BEFORE_LAND_END)
                    z = z;
                else
                    land_forced = true;
            }

            x = R_IB[0][0]*x_B + R_IB[0][1]*y_B;
            y = R_IB[1][0]*x_B + R_IB[1][1]*y_B;

            ref_yaw_msg.data = compute_ref_yaw();
            setpoint_att_quat.setRPY(0,0,ref_yaw_msg.data);
        }
        // to turn off the land_forced
        else if(!traj_on && land_on && landed_flag)
        {
            print_flag_traj_on = 0;
            land_forced = false;
        }
        else if((traj_on && land_on) || land_forced)
        {
            if (print_flag_traj_on == 0)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Holding position at [x,y,z]: %.2f,%.2f,%.2f",x,y,z);
                if (land_forced)
                {
                    ROS_INFO("Forced landing is executed. Comment and subsequently, "
                             "uncomment the land switch to continue.");
                    ROS_WARN("WARNING: Drone will jump to [0,0,0]!");
                }
                else
                    ROS_INFO("Uncomment all switches to proceed to next run");
                print_flag_traj_on = 1;
            }
            x = x;
            y = y;
            z = z;

            if (land_on)
                ref_yaw_msg.data = ref_yaw_msg.data;
            else
                ref_yaw_msg.data = compute_ref_yaw();
            setpoint_att_quat.setRPY(0,0,ref_yaw_msg.data);

            t_last = ros::Time::now().toSec();

//            if (!lidar_on)
//            {
//                print_Llidar_nan_flag = 0;
//                print_Clidar_nan_flag = 0;
//                print_Rlidar_nan_flag = 0;
//                print_ALLlidar_nan_flag = 0;
//            }
        }
        // For safety: not to let the UAV drop by turning off the traj_on switch.
//        else if (!traj_on && (z > 0.2 || !landed_flag))
        else if (!traj_on && current_pos[2] > 0.2 && z > 0.2)
        {
            print_flag_traj_on = 0;
            land_forced = true;
        }
        else
        {
            if(traj_started_flag)
                traj_started_flag = false;

            if (print_flag_traj_on == 0)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Default reference position only!");
                ROS_INFO("Position reference x =  %.2f m!",pos_ref_start_msg.pose.position.x);
                ROS_INFO("Position reference y =  %.2f m!",pos_ref_start_msg.pose.position.y);
                ROS_INFO("Position reference z =  %.2f m!",pos_ref_start_msg.pose.position.z);
                print_flag_traj_on = 1;
            }

            x = pos_ref_start_msg.pose.position.x;
            y = pos_ref_start_msg.pose.position.y;
            z = pos_ref_start_msg.pose.position.z;

            ref_yaw_msg.data = ref_yaw_msg.data;
            setpoint_att_quat.setRPY(0,0,ref_yaw_msg.data);

            t_last = ros::Time::now().toSec();

            climbed_flag = false;
            landed_flag = false;
            land_forced = false;

            print_Llidar_nan_flag = 0;
            print_Clidar_nan_flag = 0;
            print_Rlidar_nan_flag = 0;
            print_ALLlidar_nan_flag = 0;
            print_flag_setpoint = 1;
        }

//        if(land_forced)
//            std::cout<<"land_forced is on!\n";
        while(ros::ok() && (land_on || land_forced) && !landed_flag && z > 0.0)
        {
            if (print_flag_land == 0)
            {
                ROS_INFO("---------------------------------");
                ROS_INFO("Landing initialized!");
                print_flag_land = 1;
            }
            x = x;
            y = y;
            z = z > 0 ? z - LAND_RATE*sampleTime : 0;

            u = (x - x_last)/sampleTime;
            v = (y - y_last)/sampleTime;
            w = (z - z_last)/sampleTime;

            ref_yaw_msg.data = compute_ref_yaw();

            if(!pub_setpoint_pos)
            {
                reftrajectory_msg.x = x;
                reftrajectory_msg.y = y;
                reftrajectory_msg.z = z;
                ref_pos_pub.publish(reftrajectory_msg);

                reftrajectory_vel_msg.x = std::abs(u) <= absvel ? u : (u < 0 ? -absvel : absvel);
                reftrajectory_vel_msg.y = std::abs(v) <= absvel ? v : (v < 0 ? -absvel : absvel);
                reftrajectory_vel_msg.z = std::abs(w) <= absvel ? w : (w < 0 ? -absvel : absvel);
                ref_vel_pub.publish(reftrajectory_vel_msg);

                ref_yaw_pub.publish(ref_yaw_msg);
            }
            else
            {
                setpoint_pos_msg.header.stamp = ros::Time::now();
                setpoint_pos_msg.pose.position.x = x;
                setpoint_pos_msg.pose.position.y = y;
                setpoint_pos_msg.pose.position.z = z;

                setpoint_att_quat.setRPY(0,0,ref_yaw_msg.data);
                setpoint_pos_msg.pose.orientation.x = setpoint_att_quat.getX();
                setpoint_pos_msg.pose.orientation.y = setpoint_att_quat.getY();
                setpoint_pos_msg.pose.orientation.z = setpoint_att_quat.getZ();
                setpoint_pos_msg.pose.orientation.w = setpoint_att_quat.getW();
                setpoint_pos_pub.publish(setpoint_pos_msg);
            }

            if(z <= 0.0)
            {
                if (print_flag_land == 1)
                {
                    ROS_INFO("Landing complete!");
                    print_flag_land = 0;
                }
                climbed_flag = false;
                landed_flag = true;
                t_last = ros::Time::now().toSec();
            }

            x_last = x;
            y_last = y;
            z_last = z;

            traj_on_pub.publish(traj_on_msg);
            reg_on_pub.publish(reg_on_msg);
            ros::spinOnce();
            rate.sleep();
        }

        u = (x - x_last)/sampleTime;
        v = (y - y_last)/sampleTime;
        w = (z - z_last)/sampleTime;

        x_last = x;
        y_last = y;
        z_last = z;

        if(!pub_setpoint_pos)
        {
            reftrajectory_msg.x = x;
            reftrajectory_msg.y = y;
            reftrajectory_msg.z = z;
            ref_pos_pub.publish(reftrajectory_msg);

            reftrajectory_vel_msg.x = std::abs(u) <= absvel ? u : (u < 0 ? -absvel : absvel);
            reftrajectory_vel_msg.y = std::abs(v) <= absvel ? v : (v < 0 ? -absvel : absvel);
            reftrajectory_vel_msg.z = std::abs(w) <= absvel ? w : (w < 0 ? -absvel : absvel);
            ref_vel_pub.publish(reftrajectory_vel_msg);

            ref_yaw_pub.publish(ref_yaw_msg);
        }
        else
        {
            setpoint_pos_msg.header.stamp = ros::Time::now();
            setpoint_pos_msg.pose.position.x = x;
            setpoint_pos_msg.pose.position.y = y;
            setpoint_pos_msg.pose.position.z = z;

            setpoint_att_quat.setRPY(0,0,ref_yaw_msg.data);
            setpoint_pos_msg.pose.orientation.x = setpoint_att_quat.getX();
            setpoint_pos_msg.pose.orientation.y = setpoint_att_quat.getY();
            setpoint_pos_msg.pose.orientation.z = setpoint_att_quat.getZ();
            setpoint_pos_msg.pose.orientation.w = setpoint_att_quat.getW();
            setpoint_pos_pub.publish(setpoint_pos_msg);
        }

        traj_on_pub.publish(traj_on_msg);
        reg_on_pub.publish(reg_on_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

double compute_ref_yaw()
{
    if (!adaptive_yaw_on)
    {
        ref_yaw_adaptive = deg2rad*yaw_hover;
    }
    else if (std::isnan(Llidar_read_data) || std::isnan(Rlidar_read_data))
    {
        if (print_Llidar_nan_flag == 0 && print_Rlidar_nan_flag == 0)
        {
            if (verbose)
            {
                ROS_WARN("Either Llidar or Rlidar is showing nan values!");
                ROS_WARN("Maintaining yaw = %f deg!", rad2deg*current_att[2]);
            }

            if (std::isnan(Llidar_read_data))
                print_Llidar_nan_flag = 1;
            if (std::isnan(Rlidar_read_data))
                print_Rlidar_nan_flag = 1;
        }
        ref_yaw_adaptive = current_att[2];
    }
    else
    {
        if (print_Llidar_nan_flag == 1 || print_Rlidar_nan_flag == 1)
        {
            if (verbose)
            {
                ROS_WARN("Normal function resumed for L and R lidars!");
                ROS_WARN("Maintaining adaptive yaw!");
            }
            print_Llidar_nan_flag = 0;
            print_Rlidar_nan_flag = 0;
        }
        if (std::abs(Llidar_read_data - Rlidar_read_data) > 0.1)
        {
            if (Llidar_read_data < Rlidar_read_data)
                ref_yaw_adaptive += MAX_ALLOWED_YAWRATE * sampleTime;
            else
                ref_yaw_adaptive -= MAX_ALLOWED_YAWRATE * sampleTime;
        }
    }
    return ref_yaw_adaptive;
}

double compute_actual_wall_dist()
{
    if (!std::isnan(Clidar_read_data))
    {
        if (print_Clidar_nan_flag == 1)
        {
            if (verbose)
                ROS_WARN("Normal function resumed for Clidar!");
            t_Clidar_return_start = ros::Time::now().toSec();
        }
        print_Clidar_nan_flag = 0;
        print_ALLlidar_nan_flag = 0;

        if (use_sonar)
        {
            if (ros::Time::now().toSec() - t_Clidar_return_start > 1.0)
                return Clidar_read_data;
            else
                return sonar_read_data - (sonar_read_data - Clidar_read_data)*
                                         sin((M_PI/2)*(ros::Time::now().toSec() - t_Clidar_return_start));
        }
        else
            return Clidar_read_data;

    }
    else if (use_sonar && !std::isnan(sonar_read_data))
    {
        if (print_Clidar_nan_flag == 0)
        {
            if (verbose)
            {
                ROS_WARN("Clidar is showing nan values!");
                ROS_WARN("Taking the sonar data as actual distance!");
            }
            print_Clidar_nan_flag = 1;
            t_Clidar_lost_start = ros::Time::now().toSec();
        }
        if (verbose && print_sonar_nan_flag == 1)
            ROS_WARN("Normal function resumed for sonar!");
        print_Clidar_nan_flag = 1;
        print_sonar_nan_flag = 0;
        print_ALLlidar_nan_flag = 0;

        if (ros::Time::now().toSec() - t_Clidar_lost_start > 1.0)
            return sonar_read_data;
        else
            return Clidar_read_data_unfiltered[Clidar_read_data_unfiltered.size()-2] -
                        (Clidar_read_data_unfiltered[Clidar_read_data_unfiltered.size()-2] -
                         sonar_read_data)*sin((M_PI/2)*(ros::Time::now().toSec() - t_Clidar_lost_start));
    }
    else
    {
        if (verbose && (print_Clidar_nan_flag == 0 || print_sonar_nan_flag == 0))
            ROS_WARN("Clidar and sonar are showing nan values!");

        if (std::isnan(Llidar_read_data) && std::isnan(Rlidar_read_data))
        {
            if (print_ALLlidar_nan_flag == 0)
            {
                ROS_WARN("All lidars and sonar are showing nan values!");
                ROS_WARN("Taking actual distance = %f for 10 secs before landing!", wall_dist);
                print_ALLlidar_nan_flag = 1;
                print_Llidar_nan_flag = 1;
                print_Clidar_nan_flag = 1;
                print_sonar_nan_flag = 1;
                print_Rlidar_nan_flag = 1;
                t_ALLlidar_lost_start = ros::Time::now().toSec();
            }
            if (ros::Time::now().toSec() - t_ALLlidar_lost_start < 10)
                return wall_dist;
            else
            {
                ROS_WARN("Neither any lidar nor sonar showed any reading for the past 10 secs!");
                ROS_WARN("Landing is requested!");
                land_on = true;
                return wall_dist;
            }
        }
        else if (std::isnan(Llidar_read_data) || std::isnan(Rlidar_read_data))
        {
            if (std::isnan(Llidar_read_data))
            {
                if (print_Llidar_nan_flag == 0)
                {
                    if(verbose)
                    {
                        ROS_WARN("Llidar is also showing nan values!");
                        ROS_WARN("Rlidar is taken as actual distance!");
                    }
                    print_Llidar_nan_flag = 1;
                    print_Clidar_nan_flag = 1;
                    print_sonar_nan_flag = 1;
                }
                if (print_Rlidar_nan_flag == 1 || print_ALLlidar_nan_flag == 1)
                {
                    if(verbose)
                        ROS_WARN("Normal function resumed for Rlidar!");
                    print_Rlidar_nan_flag = 0;
                    print_ALLlidar_nan_flag = 0;
                    print_Clidar_nan_flag = 1;
                    print_sonar_nan_flag = 1;
                }

                return Rlidar_read_data*cos(deg2rad*DEG_BET_EACH_LIDAR);
            }
            else if(std::isnan(Rlidar_read_data))
            {
                if (print_Rlidar_nan_flag == 0)
                {
                    if(verbose)
                    {
                        ROS_WARN("Rlidar is also showing nan values!");
                        ROS_WARN("Llidar is taken as actual distance!");
                    }
                    print_Rlidar_nan_flag = 1;
                    print_Clidar_nan_flag = 1;
                    print_sonar_nan_flag = 1;
                }
                if (print_Llidar_nan_flag == 1 || print_ALLlidar_nan_flag == 1)
                {
                    if(verbose)
                        ROS_WARN("Normal function resumed for Llidar!");
                    print_Llidar_nan_flag = 0;
                    print_ALLlidar_nan_flag = 0;
                    print_Clidar_nan_flag = 1;
                    print_sonar_nan_flag = 1;
                }

                return Llidar_read_data*cos(deg2rad*DEG_BET_EACH_LIDAR);
            }
        }
        else
        {
            if (print_Clidar_nan_flag == 0 || print_sonar_nan_flag == 0)
            {
                if(verbose)
                    ROS_WARN("Taking the average of L and R lidars as actual distance!");
                print_Clidar_nan_flag = 1;
                print_sonar_nan_flag = 1;
            }
            if (print_Llidar_nan_flag == 1 || print_Rlidar_nan_flag == 1)
            {
                if(verbose)
                    ROS_WARN("Normal function resumed for L and R lidars!");
                print_Llidar_nan_flag = 0;
                print_Rlidar_nan_flag = 0;
                print_ALLlidar_nan_flag = 0;
            }

            return (Llidar_read_data*cos(deg2rad*DEG_BET_EACH_LIDAR) +
                    Rlidar_read_data*cos(deg2rad*DEG_BET_EACH_LIDAR))/2;
        }
    }
}

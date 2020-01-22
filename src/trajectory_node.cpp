#include "trajectory.h"
// #include <Eigen/Eigen>
// #include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int16.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>

#define TRAJ_FINISH 17
// using namespace Eigen;

ros::Publisher pub_position_cmd;
ros::Publisher pub_des_path;
ros::Publisher pub_planner_state;
ros::Publisher pub_traj_state;

ros::Time time_now, time_last, time_start;
Eigen::Vector3d pos_now, pos_start;
Eigen::Vector3d vel_now;
int traj_id;
bool is_init = false;
bool is_traj = false;
double max_time = 0.0;
double last_yaw = 0.0;

Eigen::Matrix3d w_R_odom;
Eigen::Vector3d w_t_odom;

void
trajectory_draw( ros::Time now_t )
{
    // double max_time_sec = 30.0;
    double dt           = 0.01;
    int pose_num        = max_time / dt;

    nav_msgs::Path path_des;
    path_des.header.stamp    = now_t;
    path_des.header.frame_id = "world";

    for ( int seq_index = 0; seq_index < pose_num; ++seq_index )
    {

        Eigen::Vector3d desired_pos, desired_vel, desired_acc, _pos_start( 0.0, 0.0, 0.0 ), _vel_now( 0.0, 0.0, 0.0 );
        double time_end;
        trajectory_control( seq_index * dt, _pos_start, _vel_now, time_end, desired_pos, desired_vel, desired_acc );

        geometry_msgs::PoseStamped pose;
        pose.header.stamp       = now_t;
        pose.header.seq         = seq_index;
        pose.header.frame_id    = "world";
        pose.pose.position.x    = desired_pos.x( );
        pose.pose.position.y    = desired_pos.y( );
        pose.pose.position.z    = desired_pos.z( );
        pose.pose.orientation.w = 1.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        path_des.poses.push_back( pose );
    }
    pub_des_path.publish( path_des );
}

void 
transform_callback( const geometry_msgs::PoseConstPtr msg)
{
    Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    w_R_odom = q.toRotationMatrix();
    w_t_odom << msg->position.x, msg->position.y, msg->position.z;
}

void
trigger_callback( const geometry_msgs::PoseStamped::ConstPtr& trigger_msg )
{
    if ( is_init )
    {
        std::cout << "[#INFO] get traj trigger info." << std::endl;
        time_start = time_now;
        pos_start  = w_R_odom * pos_now + w_t_odom;

        // replace start_pos with pos_start
        Eigen::Vector3d start_pos = Eigen::Vector3d(0,0,0);
        if(generate_trajectory(pos_start, max_time))
            trajectory_draw( time_now );
        traj_id    = trigger_msg->header.seq + 1;
        is_traj    = true;
        
    }
}

void
odom_callback( const nav_msgs::Odometry::ConstPtr& odom_msg )
{
    time_now     = odom_msg->header.stamp;
    pos_now.x( ) = odom_msg->pose.pose.position.x;
    pos_now.y( ) = odom_msg->pose.pose.position.y;
    pos_now.z( ) = odom_msg->pose.pose.position.z;

    if ( is_init )
    {
        if ( is_traj )
        {
            ros::Duration delta_t = time_now - time_start;

            Eigen::Vector3d desired_pos, desired_vel, desired_acc;
            double time_end;
            bool traj_ok
            = trajectory_control( delta_t.toSec( ), pos_start, vel_now, time_end, desired_pos, desired_vel, desired_acc );

            quadrotor_msgs::PositionCommand position_cmd;
            position_cmd.header.stamp    = time_now;
            position_cmd.header.frame_id = "world";

            desired_pos = w_R_odom.transpose() * (desired_pos - w_t_odom);
            desired_vel = w_R_odom.transpose() * desired_vel;
            desired_acc = w_R_odom.transpose() * desired_acc;
            target_point_vio = w_R_odom.transpose() * (target_point - w_t_odom);
            if ( traj_ok )
            {
                position_cmd.position.x      = desired_pos.x( );
                position_cmd.position.y      = desired_pos.y( );
                position_cmd.position.z      = desired_pos.z( );
                position_cmd.velocity.x      = desired_vel.x( );
                position_cmd.velocity.y      = desired_vel.y( );
                position_cmd.velocity.z      = desired_vel.z( );
                position_cmd.acceleration.x  = desired_acc.x( );
                position_cmd.acceleration.y  = desired_acc.y( );
                position_cmd.acceleration.z  = desired_acc.z( );
                position_cmd.yaw             = atan2(target_point_vio.y() - pos_now.y(), target_point_vio.x() - pos_now.x());
                position_cmd.yaw_dot         = 0.5;
                position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_READY;
                position_cmd.trajectory_id   = traj_id;
                last_yaw = position_cmd.yaw;
                pub_position_cmd.publish( position_cmd );
            }
            else
            {
                position_cmd.position.x      = pos_now.x( );
                position_cmd.position.y      = pos_now.y( );
                position_cmd.position.z      = pos_now.z( );
                position_cmd.velocity.x      = 0.0;
                position_cmd.velocity.y      = 0.0;
                position_cmd.velocity.z      = 0.0;
                position_cmd.acceleration.x  = 0.0;
                position_cmd.acceleration.y  = 0.0;
                position_cmd.acceleration.z  = 0.0;
                position_cmd.yaw             = last_yaw;
                position_cmd.yaw_dot         = 0;
                position_cmd.trajectory_flag = position_cmd.TRAJECTORY_STATUS_EMPTY;
                position_cmd.trajectory_id   = traj_id;
                pub_position_cmd.publish( position_cmd );
                std_msgs::Int16 traj_state;
                traj_state.data = TRAJ_FINISH;
                pub_traj_state.publish(traj_state);
            }
            
        }
    }
    else
    {
        is_init = true;
        std_msgs::Int16 planner_state_msg;
        planner_state_msg.data = 1;
        pub_planner_state.publish(planner_state_msg);
    }
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "trajectory" );
    ros::NodeHandle n( "~" );

    n.param("time_line", t_l, 0.0);
    n.param("time_circle", t_c, 0.0);
    n.param("time_duration", t_d, 0.0);
    n.param("radius", radius, 0.0);

    n.param("target_x", target_point.x(), 0.0);
    n.param("target_y", target_point.y(), 0.0);
    n.param("target_z", target_point.z(), 0.0);


    ros::Subscriber sub_trigger = n.subscribe( "traj_start_trigger", 100, trigger_callback );
    ros::Subscriber sub_odom    = n.subscribe( "odom", 100, odom_callback );
    ros::Subscriber sub_trans   = n.subscribe( "w_T_odom", 1, transform_callback );

    pub_des_path     = n.advertise< nav_msgs::Path >( "des_path", 100 );
    pub_position_cmd = n.advertise< quadrotor_msgs::PositionCommand >( "position_cmd", 10 );
    pub_planner_state= n.advertise< std_msgs::Int16>( "/planner_state", 10);
    pub_traj_state   = n.advertise< std_msgs::Int16>( "/traj_server_state", 10);

    max_time = 2 * t_l + 6 * t_d + 4 * t_c;
    // w_R_odom = Eigen::Matrix3d::Identity();
    // w_t_odom = Eigen::Vector3d::Zero();
    w_R_odom << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;
    w_t_odom << 0, 0, 0;
    ros::spin( );
    return 0;
}

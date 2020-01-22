#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Eigen/Eigen>
#include <stdio.h>
#include <math.h>

#define PI 3.1415926

using namespace std;

double t_l, t_c, t_d;
double radius;
double dt = 0.01;
Eigen::Vector3d target_point;
Eigen::Vector3d target_point_vio;
vector<Eigen::Vector3d> des_pos;
vector<Eigen::Vector3d> des_vel;
vector<Eigen::Vector3d> des_acc;
/*
 * this function is to get desired states for specific trajectory, just generated, at time dt.
 * input:
 * dT   -> the time
 * hover_pos -> the desired position where you want quadrotor to hover
 * now_vel -> maybe useless
 *
 * output:
 * desired_pos -> desired position at dT
 * desired_vel -> desired velocity at dT
 * desired_acc -> desired acceleration at dT
 * return:
 * true  -> you have alread configured desired states
 * false -> no desired state
 */
bool
trajectory_control( const double dT,
                    const Eigen::Vector3d hover_pos,
                    const Eigen::Vector3d now_vel,
                    double& end_time,
                    Eigen::Vector3d& desired_pos,
                    Eigen::Vector3d& desired_vel,
                    Eigen::Vector3d& desired_acc )
{
    // if you don't want to use Eigen, then you can use these arrays
    // or you can delete them and use Eigen
    double hover_p[3], now_v[3];
    double desired_p[3], desired_v[3], desired_a[3];
    hover_p[0] = hover_pos.x( );
    hover_p[1] = hover_pos.y( );
    hover_p[2] = hover_pos.z( );
    now_v[0]   = now_vel.x( );
    now_v[1]   = now_vel.y( );
    now_v[2]   = now_vel.z( );
    // your code // please use coefficients from matlab to get desired states
    
    

    /*---------------------------------------------------------------------------------*/
    /*--- YOUR CODE FROM HERE --- YOUR CODE FROM HERE --- YOUR CODE FROM HERE ---------*/
    /*---------------------------------------------------------------------------------*/
    //	printf("dt= %.2f\n",dT);
    double max_run_time = 2 * t_l + 5 * t_d + 4 * t_c; // set your trajectory max run time.
    double end_position[3];     // set your trajectory end point.
    // printf("11\n");
    end_position[0] = des_pos.back().x();
    end_position[1] = des_pos.back().y();
    end_position[2] = des_pos.back().z();

    // printf("12\n");

    /*---------------------------------------------------------------------------------*/
    /*---- YOUR CODE END HERE ---- YOUR CODE END HERE ---- YOUR CODE END HERE ---------*/
    /*---------------------------------------------------------------------------------*/
    if ( dT < max_run_time )
    {
        // output trajectory
        desired_p[0] = des_pos[int(dT / dt)].x();
        desired_p[1] = des_pos[int(dT / dt)].y();
        desired_p[2] = des_pos[int(dT / dt)].z();

        desired_v[0] = des_vel[int(dT / dt)].x();
        desired_v[1] = des_vel[int(dT / dt)].y();
        desired_v[2] = des_vel[int(dT / dt)].z();

        desired_a[0] = des_acc[int(dT / dt)].x();
        desired_a[1] = des_acc[int(dT / dt)].y();
        desired_a[2] = des_acc[int(dT / dt)].z();
        printf( "x = %.2f, y = %.2f, z = %.2f\n", desired_p[0], desired_p[1], desired_p[2] );

        desired_pos.x( ) = desired_p[0];
        desired_pos.y( ) = desired_p[1];
        desired_pos.z( ) = desired_p[2];
        desired_vel.x( ) = desired_v[0];
        desired_vel.y( ) = desired_v[1];
        desired_vel.z( ) = desired_v[2];
        desired_acc.x( ) = desired_a[0];
        desired_acc.y( ) = desired_a[1];
        desired_acc.z( ) = desired_a[2];
        end_time         = max_run_time;
    // printf("13\n");

        return true; // if you have got desired states, true.
    }
    else
    {
        // output end point trajectory
        desired_pos.x( ) = end_position[0];
        desired_pos.y( ) = end_position[1];
        desired_pos.z( ) = end_position[2];
        desired_vel.x( ) = 0.0;
        desired_vel.y( ) = 0.0;
        desired_vel.z( ) = 0.0;
        desired_acc.x( ) = 0.0;
        desired_acc.y( ) = 0.0;
        desired_acc.z( ) = 0.0;
        return false; // if you have got desired states, true.
    }
}

bool generate_trajectory(Eigen::Vector3d start_pose, double total_time)
{
    Eigen::Vector3d p_s, p_1;
    p_s = start_pose;
    p_1.x() = target_point.x() - radius;
    p_1.y() = target_point.y();
    p_1.z() = target_point.z();
    int pose_num = total_time / dt;
    for(int i = 0; i < pose_num; i++)
    {
        double tmp_p_x, tmp_p_y, tmp_p_z;
        double tmp_v[3];
        double tmp_a[3];
        // phase 1
        if (i * dt <= t_l)
        {
            double t = i * dt;
            tmp_p_x = (p_1.x() - p_s.x()) * i * dt / t_l + p_s.x();
            tmp_p_y = (p_1.y() - p_s.y()) * i * dt / t_l + p_s.y();
            tmp_p_z = (p_1.z() - p_s.z()) * i * dt / t_l + p_s.z();
            des_pos.push_back(Eigen::Vector3d(tmp_p_x, tmp_p_y, tmp_p_z));

            tmp_v[0] = (p_1.x() - p_s.x()) / t_l;
            tmp_v[1] = (p_1.y() - p_s.y()) / t_l;
            tmp_v[2] = (p_1.z() - p_s.z()) / t_l;
            des_vel.push_back(Eigen::Vector3d(tmp_v[0], tmp_v[1], tmp_v[2]));

            tmp_a[0] = 0;
            tmp_a[1] = 0;
            tmp_a[2] = 0;
            des_acc.push_back(Eigen::Vector3d(tmp_a[0], tmp_a[1], tmp_a[2]));
        }
        // duration
        else if (i * dt <= t_l + t_d)
        {
            des_pos.push_back(des_pos.back());

            tmp_v[0] = 0;
            tmp_v[1] = 0;
            tmp_v[2] = 0;
            des_vel.push_back(Eigen::Vector3d(tmp_v[0], tmp_v[1], tmp_v[2]));

            tmp_a[0] = 0;
            tmp_a[1] = 0;
            tmp_a[2] = 0;
            des_acc.push_back(Eigen::Vector3d(tmp_a[0], tmp_a[1], tmp_a[2]));
            
        }
        // phase 2
        else if (i * dt <= t_l + t_d + 0.5 * t_c)
        {
            double t = i * dt - t_l - t_d;
            tmp_p_x = target_point.x() - radius * cos(PI/t_c * t);
            tmp_p_y = target_point.y() + radius * sin(PI/t_c * t);
            tmp_p_z = target_point.z();
            des_pos.push_back(Eigen::Vector3d(tmp_p_x, tmp_p_y, tmp_p_z));

            tmp_v[0] = PI/t_c * radius * sin(PI/t_c * t);
            tmp_v[1] = PI/t_c * radius * cos(PI/t_c * t);
            tmp_v[2] = 0;
            des_vel.push_back(Eigen::Vector3d(tmp_v[0], tmp_v[1], tmp_v[2]));

            tmp_a[0] = (PI/t_c) * (PI/t_c) * radius * cos(PI/t_c * t);
            tmp_a[1] = -(PI/t_c) * (PI/t_c) * radius * sin(PI/t_c * t);
            tmp_a[2] = 0;
            des_acc.push_back(Eigen::Vector3d(tmp_a[0], tmp_a[1], tmp_a[2]));
        }
        // duration
        else if (i * dt <= t_l + 2 * t_d + 0.5 * t_c)
        {
            des_pos.push_back(des_pos.back());   
            
            tmp_v[0] = 0;
            tmp_v[1] = 0;
            tmp_v[2] = 0;
            des_vel.push_back(Eigen::Vector3d(tmp_v[0], tmp_v[1], tmp_v[2]));

            tmp_a[0] = 0;
            tmp_a[1] = 0;
            tmp_a[2] = 0;
            des_acc.push_back(Eigen::Vector3d(tmp_a[0], tmp_a[1], tmp_a[2]));        
        }
        // phase 3
        else if (i * dt <= t_l + 2 * t_d + 1.5 * t_c)
        {
            double t = i * dt - t_l - 2 * t_d - 0.5 * t_c;
            tmp_p_x = target_point.x() - radius * sin(PI/t_c * t);
            tmp_p_y = target_point.y() + radius * cos(PI/t_c * t);
            tmp_p_z = target_point.z();
            des_pos.push_back(Eigen::Vector3d(tmp_p_x, tmp_p_y, tmp_p_z));

            tmp_v[0] = -PI/t_c * radius * cos(PI/t_c * t);
            tmp_v[1] = -PI/t_c * radius * sin(PI/t_c * t);
            tmp_v[2] = 0;
            des_vel.push_back(Eigen::Vector3d(tmp_v[0], tmp_v[1], tmp_v[2]));

            tmp_a[0] = (PI/t_c) * (PI/t_c) * radius * sin(PI/t_c * t);
            tmp_a[1] = -(PI/t_c) * (PI/t_c) * radius * cos(PI/t_c * t);
            tmp_a[2] = 0;
            des_acc.push_back(Eigen::Vector3d(tmp_a[0], tmp_a[1], tmp_a[2]));
        }
        // duration
        else if (i * dt <= t_l + 3 * t_d + 1.5 * t_c)
        {
            des_pos.push_back(des_pos.back());

            tmp_v[0] = 0;
            tmp_v[1] = 0;
            tmp_v[2] = 0;
            des_vel.push_back(Eigen::Vector3d(tmp_v[0], tmp_v[1], tmp_v[2]));

            tmp_a[0] = 0;
            tmp_a[1] = 0;
            tmp_a[2] = 0;
            des_acc.push_back(Eigen::Vector3d(tmp_a[0], tmp_a[1], tmp_a[2]));
        }
        // phase 4
        else if (i * dt <= t_l + 3 * t_d + 2.5 * t_c)
        {
            double t = i * dt - t_l - 3 * t_d - 1.5 * t_c;
            tmp_p_x = target_point.x() - radius * sin(PI/t_c * t);
            tmp_p_y = target_point.y() - radius * cos(PI/t_c * t);
            tmp_p_z = target_point.z();
            des_pos.push_back(Eigen::Vector3d(tmp_p_x, tmp_p_y, tmp_p_z));

            tmp_v[0] = -PI/t_c * radius * cos(PI/t_c * t);
            tmp_v[1] = PI/t_c * radius * sin(PI/t_c * t);
            tmp_v[2] = 0;
            des_vel.push_back(Eigen::Vector3d(tmp_v[0], tmp_v[1], tmp_v[2]));

            tmp_a[0] = (PI/t_c) * (PI/t_c) * radius * sin(PI/t_c * t);
            tmp_a[1] = (PI/t_c) * (PI/t_c) * radius * cos(PI/t_c * t);
            tmp_a[2] = 0;
            des_acc.push_back(Eigen::Vector3d(tmp_a[0], tmp_a[1], tmp_a[2]));
        }
        // duration
        else if (i * dt <= t_l + 4 * t_d + 2.5 * t_c)
        {
            des_pos.push_back(des_pos.back());

            tmp_v[0] = 0;
            tmp_v[1] = 0;
            tmp_v[2] = 0;
            des_vel.push_back(Eigen::Vector3d(tmp_v[0], tmp_v[1], tmp_v[2]));

            tmp_a[0] = 0;
            tmp_a[1] = 0;
            tmp_a[2] = 0;
            des_acc.push_back(Eigen::Vector3d(tmp_a[0], tmp_a[1], tmp_a[2]));
        }
        // phase 5
        else if (i * dt <= t_l + 4 * t_d + 3.5 * t_c)
        {
            double t = i * dt - t_l - 4 * t_d - 2.5 * t_c;
            tmp_p_x = target_point.x() - radius * sin(PI/t_c * t);
            tmp_p_y = target_point.y() + radius * cos(PI/t_c * t);
            tmp_p_z = target_point.z();
            des_pos.push_back(Eigen::Vector3d(tmp_p_x, tmp_p_y, tmp_p_z));

            tmp_v[0] = -PI/t_c * radius * cos(PI/t_c * t);
            tmp_v[1] = -PI/t_c * radius * sin(PI/t_c * t);
            tmp_v[2] = 0;
            des_vel.push_back(Eigen::Vector3d(tmp_v[0], tmp_v[1], tmp_v[2]));

            tmp_a[0] = (PI/t_c) * (PI/t_c) * radius * sin(PI/t_c * t);
            tmp_a[1] = -(PI/t_c) * (PI/t_c) * radius * cos(PI/t_c * t);
            tmp_a[2] = 0;
            des_acc.push_back(Eigen::Vector3d(tmp_a[0], tmp_a[1], tmp_a[2]));
        }
        // duration
        else if (i * dt <= t_l + 5 * t_d + 3.5 * t_c)
        {
            des_pos.push_back(des_pos.back());

            tmp_v[0] = 0;
            tmp_v[1] = 0;
            tmp_v[2] = 0;
            des_vel.push_back(Eigen::Vector3d(tmp_v[0], tmp_v[1], tmp_v[2]));

            tmp_a[0] = 0;
            tmp_a[1] = 0;
            tmp_a[2] = 0;
            des_acc.push_back(Eigen::Vector3d(tmp_a[0], tmp_a[1], tmp_a[2]));
        }
        // phase 6
        else if (i * dt <= t_l + 5 * t_d + 4 * t_c)
        {
            double t = i * dt - t_l - 5 * t_d - 3.5 * t_c;
            tmp_p_x = target_point.x() - radius * sin(PI/t_c * t);
            tmp_p_y = target_point.y() - radius * cos(PI/t_c * t);
            tmp_p_z = target_point.z();
            des_pos.push_back(Eigen::Vector3d(tmp_p_x, tmp_p_y, tmp_p_z));

            tmp_v[0] = -PI/t_c * radius * cos(PI/t_c * t);
            tmp_v[1] = PI/t_c * radius * sin(PI/t_c * t);
            tmp_v[2] = 0;
            des_vel.push_back(Eigen::Vector3d(tmp_v[0], tmp_v[1], tmp_v[2]));

            tmp_a[0] = (PI/t_c) * (PI/t_c) * radius * sin(PI/t_c * t);
            tmp_a[1] = (PI/t_c) * (PI/t_c) * radius * cos(PI/t_c * t);
            tmp_a[2] = 0;
            des_acc.push_back(Eigen::Vector3d(tmp_a[0], tmp_a[1], tmp_a[2]));
        }
        // duration
        else if (i * dt <= t_l + 6 * t_d + 4 * t_c)
        {
            des_pos.push_back(des_pos.back());     
            
            tmp_v[0] = 0;
            tmp_v[1] = 0;
            tmp_v[2] = 0;
            des_vel.push_back(Eigen::Vector3d(tmp_v[0], tmp_v[1], tmp_v[2]));

            tmp_a[0] = 0;
            tmp_a[1] = 0;
            tmp_a[2] = 0;
            des_acc.push_back(Eigen::Vector3d(tmp_a[0], tmp_a[1], tmp_a[2]));       
        }
        // phase 7
        else if (i * dt <= 2 * t_l + 6 * t_d + 4 * t_c)
        {
            double t = i * dt - t_l - 6 * t_d - 4 * t_c;
            tmp_p_x = p_1.x() + (p_s.x() - p_1.x())/t_l * t;
            tmp_p_y = p_1.y() + (p_s.y() - p_1.y())/t_l * t;
            tmp_p_z = p_1.z() + (p_s.z() - p_1.z())/t_l * t;
            des_pos.push_back(Eigen::Vector3d(tmp_p_x, tmp_p_y, tmp_p_z));

            tmp_v[0] = (p_s.x() - p_1.x())/t_l;
            tmp_v[1] = (p_s.y() - p_1.y())/t_l;
            tmp_v[2] = (p_s.z() - p_1.z())/t_l;
            des_vel.push_back(Eigen::Vector3d(tmp_v[0], tmp_v[1], tmp_v[2]));

            tmp_a[0] = 0;
            tmp_a[1] = 0;
            tmp_a[2] = 0;
            des_acc.push_back(Eigen::Vector3d(tmp_a[0], tmp_a[1], tmp_a[2]));
        }
                
    }
    printf("Trajectory generated !\n");
    return true;
}

#endif // TRAJECTORY_H

#!/bin/bash

source ~/app/bashrc.sh;
echo " " | sudo -S chmod 777 /dev/ttyUSB0
#taskset -c 1 roslaunch mocap_optitrack mocap.launch & sleep 1;
#rosrun trajectory_generator toOdom
#rosrun pos_vel_mocap pos_vel_mocap & sleep 1;
roslaunch demo_main circle.launch & sleep 2;
roslaunch djiros djiros.launch & sleep 4;
roslaunch realsense2_camera rs_camera.launch & sleep 4;
rosrun dynamic_reconfigure dynparam set /camera/Stereo_Module 'Emitter Enabled' false & sleep 2; 
rosrun vins vins_node ~/small_drone_ws/src/VINS-Fusion/config/newcalibration/realsense_stereo_imu_config.yaml & sleep 5;
rosrun loop_fusion loop_fusion_node ~/small_drone_ws/src/VINS-Fusion/config/newcalibration/realsense_stereo_imu_config.yaml & sleep 2;
roslaunch machine_defined ctrl_md.launch & sleep 4;
roslaunch trajectory_node inspection.launch & sleep 2;
rosrun dji_sdk_demo demo_flight_control
#taskset -c 1 rosrun trajectory_node trajectory_node #& sleep 5;
#taskset -c 6 roslaunch esdf_tools demo.launch

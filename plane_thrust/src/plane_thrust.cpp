/*
 * position_control.cpp
 *
 * Author:mz
 *
 * Time: 2018.11.27
 *
 * 说明: mavros位置控制示例程序
 *      输入：mavros发布的位置/速度信息
 *      输出：无人机的推力和姿态信息
 *      采用位置环/速度环串级PID控制，位置环P控制，速度环PID控制
 */

#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry> 
#include <Eigen/Core> 

#include <ros/ros.h>
#include "Parameter.h"
#include <PID.h>


//topic
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/AttitudeTarget.h>
using namespace Eigen;

// //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define x_dis 3
mavros_msgs::State current_state;           //无人机当前状态
nav_msgs::Odometry pose_drone_odom;       //读入的无人机当前位置
nav_msgs::Odometry pose_car_odom;       //读入car当前位置
geometry_msgs::Twist vel_car_odom;      // read car vel
geometry_msgs::TwistStamped vel_drone;      //读入的无人机当前速度
//geometry_msgs::PoseStamped att_drone;       //读入的无人机姿态
geometry_msgs::Vector3 angle_receive;       //读入的无人机姿态（欧拉角）
//geometry_msgs::PoseStamped target_position;  //publish plane position
//mavros_msgs::PositionTarget pose_plane_target;
geometry_msgs::Quaternion orientation_target;   //发给无人机的姿态指令
geometry_msgs::Vector3 angle_target;
geometry_msgs::Vector3 vel_target;
geometry_msgs::Point plane_expected_position;
geometry_msgs::PoseStamped target_attitude;
mavros_msgs::Thrust target_thrust_msg;
std_msgs::Float64 plane_real_alt;
mavros_msgs::AttitudeTarget target_atti_thrust_msg;
geometry_msgs::Vector3 plane_angle_msgs;
geometry_msgs::Vector3 xyz_dis;
//variable for plotting plane pose
geometry_msgs::PoseStamped plane_pose_plot;


float thrust_target;        //期望推力
float Yaw_Init;
float Yaw_Locked = 0;           //锁定的偏航角(一般锁定为0)
bool got_initial_point = false;
PID PIDVX, PIDVY, PIDVZ;    //声明PID类
PID PID_err_vx,PID_err_vy;  //the velocity feedback between car and plane
PID PID_THRUST_Z; //pix_controller_thrust z
Parameter param;
std::ofstream logfile;

float plane_roll,plane_pitch,plane_yaw;
float PX_P;
float VX_P,VX_I,VX_D;
float VX_ERR_P,VX_ERR_I,VX_ERR_D;
float PZ_P;
float VZ_P,VZ_I,VZ_D;
float VZ_ERR_P,VZ_ERR_I,VZ_ERR_D;
float delta_d;

std_msgs::Bool thrust_mode;
std_msgs::Bool enable_getParam;
std_msgs::Bool leave_flag;

float cur_time_01;
float cur_time_02;
float cur_time_03;
float thrust_base;
float thrust_delta;
float cal_target_z;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//欧拉角转四元数
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw);
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w);

float get_ros_time(ros::Time time_begin);                                            //获取ros当前时间
int pix_controller(float cur_time);//控制程序
float pix_controller_thrust(float cur_time);
float calculate_cur_err();
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;

}

void plane_pos_cb(const nav_msgs::Odometry::ConstPtr &msg){
    pose_drone_odom = *msg;
}

void plane_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    vel_drone = *msg;
}

void car_pos_cb(const nav_msgs::Odometry::ConstPtr &msg){
    pose_car_odom = *msg;
    plane_expected_position.x = pose_car_odom.pose.pose.position.x - 3; //-1 means axis difference
    plane_expected_position.y = pose_car_odom.pose.pose.position.y -1; //-1 means axis difference
    plane_expected_position.z = pose_car_odom.pose.pose.position.z + 6;//6 respect to 4.5m in laser_recirver model
//    plane_expected_position.x = 0;
//    plane_expected_position.y = 0;
//    plane_expected_position.z = 5;

}
void car_vel_cb(const geometry_msgs::Twist::ConstPtr &msg){
    vel_car_odom = *msg;
}

void plane_alt_cb(const std_msgs::Float64::ConstPtr &msg){
    plane_real_alt = *msg;
}

void param_load_cb(const std_msgs::Bool::ConstPtr &msg){
    enable_getParam = *msg;
}
void thrust_mode_cb(const std_msgs::Bool::ConstPtr &msg){
    thrust_mode = *msg;
}
void leave_flag_cb(const std_msgs::Bool::ConstPtr &msg){
    leave_flag= *msg;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_control");
    ros::NodeHandle nh;
    std::ofstream logfile;

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient setmode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 【订阅】无人机当前状态/位置/速度信息
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber plane_position_pose_sub = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 10, plane_pos_cb);
    ros::Subscriber plane_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, plane_vel_cb);
    ros::Subscriber car_position_sub = nh.subscribe<nav_msgs::Odometry>("odom",10,car_pos_cb);
    ros::Subscriber car_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel",10,car_vel_cb);
    ros::Subscriber plane_alt_sub = nh.subscribe<std_msgs::Float64>("mavros/global_position/rel_alt",10,plane_alt_cb);
    ros::Subscriber enable_getParam_sub = nh.subscribe<std_msgs::Bool>("enable_getParam",2,param_load_cb);
    ros::Subscriber thrust_mode_sub = nh.subscribe<std_msgs::Bool>("thrust_mode",2,thrust_mode_cb);
    ros::Subscriber leave_flag_sub = nh.subscribe<std_msgs::Bool>("leave_flag",2,leave_flag_cb);
    // 【发布】飞机姿态/拉力信息 坐标系:NED系
//    ros::Publisher target_attitude_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude", 10);
//    ros::Publisher target_thrust_pub = nh.advertise<mavros_msgs::Thrust>("mavros/setpoint_attitude/thrust",10);
//    ros::Publisher target_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher target_atti_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude",10);
    ros::Publisher enable_getParam_pub = nh.advertise<std_msgs::Bool>("enable_getParam",2);
    ros::Publisher plane_pitch_pub = nh.advertise<geometry_msgs::Vector3>("plane_angle_angle",10);
    ros::Publisher xyz_distance_pub = nh.advertise<geometry_msgs::Vector3>("xyz_dis",2);
    // 频率 [30Hz]
    ros::Rate rate(30.0);

    // 读取PID参数
    std::string paraadr("/home/wzy/catkin_ws/src/plane_thrust/src/param");
    if (param.readParam(paraadr.c_str()) == 0){
        std::cout<<"read config file error!"<<std::endl;
        return 0;
    }


    // 设置速度环PID参数 比例参数 积分参数 微分参数
    PIDVX.setPID(param.vx_p, param.vx_i, param.vx_d);
    PIDVY.setPID(param.vy_p, param.vy_i, param.vy_d);
    PIDVZ.setPID(param.vz_p, param.vz_i, param.vz_d);
    // 设置速度环积分上限 控制量最大值 误差死区
    PIDVX.set_sat(2, 3, 0);
    PIDVY.set_sat(2, 3, 0);
    PIDVZ.set_sat(2, 5, 0);
    //set velocity feedback parameter
    PID_err_vx.setPID(param.vx_error_p, param.vx_error_i, param.vx_error_d);
    PID_err_vy.setPID(param.vy_error_p, param.vy_error_i, param.vy_error_d);

    // 设置velocity feedback积分上限 控制量最大值 误差死区
    PID_err_vx.set_sat(2, 3, 0);
    PID_err_vy.set_sat(2, 3, 0);
    //set thrust_mode parameter
    PID_THRUST_Z.setPID(param.error_z_pos_P,param.error_z_pos_I,param.error_z_pos_D);
    PID_THRUST_Z.setPID(param.error_z_vel_P,param.error_z_vel_I,param.error_z_vel_D);
    PID_THRUST_Z.set_sat(2,3,0);

    // 等待和飞控的连接
    while(ros::ok() && current_state.connected == 0)
    {
        ros::spinOnce();
        ros::Duration(1).sleep();
        ROS_INFO("Not Connected");
    }
    ROS_INFO("Connected!!");

//    target_atti_thrust_msg.orientation.x = 0;
//    target_atti_thrust_msg.orientation.y = 0;
//    target_atti_thrust_msg.orientation.z = 0;
//    target_atti_thrust_msg.orientation.w = -1;
    target_atti_thrust_msg.thrust = 0.6;

    // get car current pose to set plane pose
    float x = pose_car_odom.pose.pose.orientation.x;
    float y = pose_car_odom.pose.pose.orientation.y;
    float z = pose_car_odom.pose.pose.orientation.z;
    float w = pose_car_odom.pose.pose.orientation.w;
    Yaw_Init = quaternion2euler(x,y,z,w).z;
    //set initial hover position and pose

//    target_attitude.pose.orientation.w = w;
    target_atti_thrust_msg.orientation.x = x;
    target_atti_thrust_msg.orientation.y = y;
    target_atti_thrust_msg.orientation.z = z;
    target_atti_thrust_msg.orientation.w = w;
    ROS_INFO("got initial point ");
//    target_attitude.pose.position.x = -4;
//    target_attitude.pose.position.y = -1;
//    target_attitude.pose.position.z = 1;
//    target_attitude.pose.orientation.x = 0;
//    target_attitude.pose.orientation.y = 0;
//    target_attitude.pose.orientation.z = 0;
//    target_attitude.pose.orientation.w = -1;
//    target_thrust_msg.thrust = 0.5;
        for (int i = 100; ros::ok() && i > 0 ; --i)
        {
//            target_thrust_pub.publish(target_thrust_msg);
//            target_attitude_pub.publish(target_attitude);
            target_atti_thrust_pub.publish(target_atti_thrust_msg);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("OUT OF LOOP WAIT");



    // change mode to arm ,then offboard
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( setmode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
//        target_attitude_pub.publish(target_attitude);
//        target_thrust_pub.publish(target_thrust_msg);
        target_atti_thrust_pub.publish(target_atti_thrust_msg);
        if(plane_real_alt.data > 0.7 )
        {   ROS_INFO("plane takeoff !");
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }


    // reach initial hover position and pose by position control
    float error_position_sum = 0;
    float error_pose_sum = 5;
    float yaw_current;
    ros::Time begin_time_01 = ros::Time::now();
    while(error_position_sum < 0.2)
    {
        ROS_INFO("adjusting !");

        ros::spinOnce();

        cur_time_01 = get_ros_time(begin_time_01);  // 当前时间
        pix_controller(cur_time_01);                   //控制程序

        target_atti_thrust_msg.orientation = orientation_target;
        target_atti_thrust_msg.thrust = thrust_target;
//        target_attitude_pub.publish(target_attitude);
//        target_thrust_pub.publish(target_thrust_msg);
        target_atti_thrust_pub.publish(target_atti_thrust_msg);
        plane_pitch_pub.publish(plane_angle_msgs);
        rate.sleep();
//        ros::spinOnce();

        error_position_sum = abs(pose_drone_odom.pose.pose.position.z - target_attitude.pose.position.z) + abs(pose_drone_odom.pose.pose.position.x - target_attitude.pose.position.x) + abs(pose_drone_odom.pose.pose.position.y - target_attitude.pose.position.y);
        ROS_INFO("%f",error_position_sum);
//        error_pose_sum = abs(pose_drone_odom.pose.pose.orientation.x - pose_plane_target.pose.orientation.x)
//                +abs(pose_drone_odom.pose.pose.orientation.y - pose_plane_target.pose.orientation.y)
//                +abs(pose_drone_odom.pose.pose.orientation.z - pose_plane_target.pose.orientation.z)
//                +abs(pose_drone_odom.pose.pose.orientation.w - pose_plane_target.pose.orientation.w);
//        yaw_current = quaternion2euler(pose_drone_odom.pose.pose.orientation.x,pose_drone_odom.pose.pose.orientation.y,pose_drone_odom.pose.pose.orientation.z,pose_drone_odom.pose.pose.orientation.w).z;
//        error_pose_sum = abs(yaw_current - Yaw_Init);
//        std::cout << "error_pose_sum" << error_pose_sum << std::endl;
//        target_pos_pub.publish(pose_plane_target);
//        rate.sleep();
    }
    got_initial_point = true;
    ROS_INFO("reached initial point and pose ");
    // 记录启控时间
    ros::Time begin_time_02 = ros::Time::now();

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok() && !thrust_mode.data)
    {
        ros::spinOnce();
        //// in terminal : cd {this_package_path/param}
        //// rosparam load PIDparam.yaml
        if(enable_getParam.data == true)
        {
            nh.getParam("PX_P", PX_P);
            nh.getParam("VX_P", VX_P);
            nh.getParam("VX_I", VX_I);
            nh.getParam("VX_D", VX_D);
            nh.getParam("VX_ERR_P", VX_ERR_P);
            nh.getParam("VX_ERR_I", VX_ERR_I);
            nh.getParam("VX_ERR_D", VX_ERR_D);

            nh.getParam("PZ_P", PZ_P);
            nh.getParam("VZ_P", VZ_P);
            nh.getParam("VZ_I", VZ_I);
            nh.getParam("VZ_D", VZ_D);

            if (PX_P < 10 && PX_P > -1 && VX_P < 10 && VX_P > -1 && VX_I < 10 && VX_I > -1 && VX_D < 10 && VX_D > -1 &&
                VX_ERR_P > -1 && VX_ERR_P < 10 && VX_ERR_I > -1 && VX_ERR_I < 10 && VX_ERR_D > -1 && VX_ERR_D < 10) {
                PIDVX.setPID(VX_P, VX_I, VX_D);
                PID_err_vx.setPID(VX_ERR_P, VX_ERR_I, VX_ERR_D);
                param.x_p = PX_P;
                ROS_INFO("LOAD NEW  X  PARAM !");

            }

            if (PZ_P < 10 && PZ_P > -1 && VZ_P < 10 && VZ_P > -1 && VZ_I < 10 && VZ_I > -1 && VZ_D < 10 && VZ_D > -1) {
                PIDVZ.setPID(VZ_P, VZ_I, VZ_D);
                param.z_p = PZ_P;
                ROS_INFO("LOAD NEW  Z  PARAM !");
            }
        }

        ROS_INFO("LOOP");


        cur_time_02 = get_ros_time(begin_time_02);  // 当前时间
        pix_controller(cur_time_02);
        std::cout << "thrust_target" << thrust_target << std::endl;
        target_atti_thrust_msg.orientation = orientation_target;
        target_atti_thrust_msg.thrust = thrust_target;
//        target_attitude_pub.publish(target_attitude);
//        target_thrust_pub.publish(target_thrust_msg);
        target_atti_thrust_pub.publish(target_atti_thrust_msg);
        plane_pitch_pub.publish(plane_angle_msgs);
        xyz_distance_pub.publish(xyz_dis);

        rate.sleep();
    }

    calculate_cur_err();
    thrust_base = 0.49;
    // 记录启控时间
    ros::Time begin_time_03 = ros::Time::now();
    while(ros::ok() && !leave_flag.data)
    {
        ros::spinOnce();

        cur_time_03 = get_ros_time(begin_time_03);  // 当前时间
        ////test max velocity with a constant pitch and thrust
        pix_controller_thrust(cur_time_03);
        orientation_target = euler2quaternion(0,0.15,0.0);

        thrust_target = thrust_base + thrust_delta;
        target_atti_thrust_msg.thrust = thrust_target;
        target_atti_thrust_msg.orientation = orientation_target;
        target_atti_thrust_pub.publish(target_atti_thrust_msg);
        plane_pitch_pub.publish(plane_angle_msgs);
//        std::cout << "plane_vel_x: " << pose_drone_odom.twist.twist.linear.x << "\tplane_pos_z: " << pose_drone_odom.pose.pose.position.z << "\tthrust: "<< thrust_target <<std::endl;
        xyz_distance_pub.publish(xyz_dis);
        rate.sleep();
    }


    return 0;
}


/**
 * 获取当前时间 单位：秒
 */
float get_ros_time(ros::Time time_begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-time_begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - time_begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>控 制 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int pix_controller(float cur_time)
{
    // need to raise height to overcome the pitch angle change corresponding to velocity
//    delta_d = 3.0 * tan(0.12 * vel_car_odom.linear.x);
//    std::cout << "delta_d : "<< delta_d << std::endl;
//位 置 环
    //计算误差
    float error_x = plane_expected_position.x - pose_drone_odom.pose.pose.position.x;
    float error_y = plane_expected_position.y - pose_drone_odom.pose.pose.position.y;
    float error_z = plane_expected_position.z - plane_real_alt.data;
    std::cout << "error: x：" << error_x << "\ty：" << error_y << "\tz：" << error_z << std::endl;
    xyz_dis.x = error_x;
    xyz_dis.y = error_y;
    xyz_dis.z = error_z + 1;
    //计算指定速度误差
    float vel_xd = param.x_p * error_x;
    float vel_yd = param.y_p * error_y;
    float vel_zd = param.z_p * error_z;

    // calculate velocity error between car and plane
//    float vel_x_error = vel_car_odom.linear.x - vel_drone.twist.linear.x;
//    float vel_y_error = vel_car_odom.linear.y - vel_drone.twist.linear.y;
//    PID_err_vx.add_error(vel_x_error,cur_time);
//    PID_err_vy.add_error(vel_y_error,cur_time);
//    PID_err_vx.pid_output();
//    PID_err_vy.pid_output();

    // postion control plus velocity control
//    vel_target.x = vel_xd +PID_err_vx.Output;
//    vel_target.y = vel_yd +PID_err_vy.Output;
//    vel_target.z = vel_zd;
//    std::cout << " VX_P：" <<  PIDVX.Kp << "\tVX_I：" << PIDVX.Ki << "\tVX_D：" << PIDVX.Kd << std::endl;
//    std::cout << "vel_xd " << vel_xd << std::endl;
//    std::cout << "PID_err_vx.Output: " << PID_err_vx.Output << std::endl;
//    std::cout << "vel_target.x : " << vel_target.x << std::endl;

////速 度 环
    ////积分标志位.未进入OFFBOARD时,不累积积分项;进入OFFBOARD时,开始积分.
    PIDVX.start_intergrate_flag = true;
    PIDVY.start_intergrate_flag = true;
    PIDVZ.start_intergrate_flag = true;
    if(got_initial_point == false){
        PIDVX.start_intergrate_flag = false;
        PIDVY.start_intergrate_flag = false;
        PIDVZ.start_intergrate_flag = false;
    }
    //计算误差
    float error_vx = vel_xd ;
    float error_vy = vel_yd ;
    float error_vz = vel_zd ;
    //传递误差
    PIDVX.add_error(error_vx, cur_time);
    PIDVY.add_error(error_vy, cur_time);
    PIDVZ.add_error(error_vz, cur_time);
    //计算输出
    PIDVX.pid_output();
    PIDVY.pid_output();
    PIDVZ.pid_output();

    Matrix2f A_yaw;
    A_yaw << sin(Yaw_Locked), cos(Yaw_Locked),
            -cos(Yaw_Locked), sin(Yaw_Locked);
    Vector2f mat_temp(PIDVX.Output,PIDVY.Output);       //赋值到期望推力和姿态
    Vector2f euler_temp= 1/9.8 * A_yaw.inverse() * mat_temp;
    angle_target.x = euler_temp[0];
    angle_target.y = euler_temp[1];
//    std::cout << "angle_target.x: " << angle_target.x << "\tangle_target.y: " << angle_target.y << std::endl;
//    angle_target.z = Yaw_Locked + Yaw_Init;
    angle_target.z = Yaw_Init;

    orientation_target = euler2quaternion(angle_target.x, angle_target.y, angle_target.z);
    thrust_target = (float)(0.05 * (9.8 + PIDVZ.Output));   //目标推力值

    plane_pitch = quaternion2euler(pose_drone_odom.pose.pose.orientation.x,pose_drone_odom.pose.pose.orientation.y,pose_drone_odom.pose.pose.orientation.z,pose_drone_odom.pose.pose.orientation.w).y;
    plane_roll = quaternion2euler(pose_drone_odom.pose.pose.orientation.x,pose_drone_odom.pose.pose.orientation.y,pose_drone_odom.pose.pose.orientation.z,pose_drone_odom.pose.pose.orientation.w).x;
    plane_yaw = quaternion2euler(pose_drone_odom.pose.pose.orientation.x,pose_drone_odom.pose.pose.orientation.y,pose_drone_odom.pose.pose.orientation.z,pose_drone_odom.pose.pose.orientation.w).z;
    plane_angle_msgs.y = plane_pitch;
    plane_angle_msgs.x = plane_roll;
    plane_angle_msgs.z = plane_yaw;

    //    std::cout << "roll: " << plane_roll << "\tpitch: " << plane_pitch << std::endl;

        std::cout << "PIDVZ.OUTPUT:  " << PIDVZ.Output << std::endl;
//    std::cout << "thrust_target:  " << thrust_target << std::endl;
    return 0;
}

float pix_controller_thrust(float cur_time)
{
    float error_x = plane_expected_position.x - pose_drone_odom.pose.pose.position.x;
    float error_y = plane_expected_position.y - pose_drone_odom.pose.pose.position.y;
    float error_z = plane_expected_position.z - plane_real_alt.data;
    std::cout << "error: x：" << error_x << "\ty：" << error_y << "\tz：" << error_z << std::endl;
    xyz_dis.x = error_x;
    xyz_dis.y = error_y;
    xyz_dis.z = error_z + 1;
//    float error_z_pos = 5 - pose_drone_odom.pose.pose.position.z;
    float error_z_pos = cal_target_z - (float)pose_drone_odom.pose.pose.position.z;
    float error_z_vel = param.error_z_pos_P *error_z_pos;

    PID_THRUST_Z.start_intergrate_flag = true;
    PID_THRUST_Z.add_error(error_z_vel,cur_time);
    PID_THRUST_Z.pid_output();
    std::cout << "PID_THRUST_Z.Output: "<<PID_THRUST_Z.Output<<std::endl;
//    thrust_target = 0.05 * (9.8 + PID_THRUST_Z.Output);
    thrust_delta = 0.05 * PID_THRUST_Z.Output;
    return 0;
}
/**
 * 将欧拉角转化为四元数
 * @param roll
 * @param pitch
 * @param yaw
 * @return 返回四元数
 */
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw)
{
    geometry_msgs::Quaternion temp;
    temp.w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.x = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.y = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    temp.z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
    return temp;
}

/**
 * 将四元数转化为欧拉角形式
 * @param x
 * @param y
 * @param z
 * @param w
 * @return 返回Vector3的欧拉角
 */
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w)
{
    geometry_msgs::Vector3 temp;
    temp.x = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    temp.y = asin(2.0 * (z * x - w * y));
    temp.z = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return temp;
}

/**
 * 将进入offboard后的位置&速度&姿态信息记录进文件
 * @param cur_time
 */
float calculate_cur_err()
{
    ros::spinOnce();
    float error_x_pos = pose_car_odom.pose.pose.position.x - pose_drone_odom.pose.pose.position.x - x_dis;
    float error_z_pos = error_x_pos * sin(0.15) * cos(0.15);
    cal_target_z = error_z_pos + pose_drone_odom.pose.pose.position.z;
    return 0;
}
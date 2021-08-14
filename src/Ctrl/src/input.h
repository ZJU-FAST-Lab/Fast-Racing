#ifndef __INPUT_H
#define __INPUT_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <uav_utils/utils.h>
#include "CtrlParam.h"

class RC_Data_t {
  public:
    double mode;
    double gear;
    double last_mode;
    double last_gear;
    bool have_init_last_mode{false};
    bool have_init_last_gear{false};

    mavros_msgs::RCIn msg;
    ros::Time rcv_stamp;

    bool is_command_mode;
    bool enter_command_mode;
    bool is_api_mode;
    bool enter_api_mode;    

    static constexpr double GEAR_SHIFT_VALUE = 0.75;
    static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;

    RC_Data_t();
    void check_validity();
    void feed(mavros_msgs::RCInConstPtr pMsg);
};

class Odom_Data_t {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d w;
    Eigen::Quaterniond q;

    nav_msgs::Odometry msg;
    ros::Time rcv_stamp;
    bool odom_init;
    Odom_Data_t();
    void feed(nav_msgs::OdometryConstPtr pMsg);
};

class Imu_Data_t {
  public:
    Eigen::Quaterniond q;
    Eigen::Vector3d w;
    Eigen::Vector3d a;

    sensor_msgs::Imu msg;
    ros::Time rcv_stamp;
    bool imu_init;
    Imu_Data_t();
    void feed(sensor_msgs::ImuConstPtr pMsg);
};

class State_Data_t {
  public:
    mavros_msgs::State current_state;

    State_Data_t();
    void feed(mavros_msgs::StateConstPtr pMsg);
};

class Command_Data_t {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d jerk;
    double yaw;
    double head_rate;
    bool cmd_init;
    quadrotor_msgs::PositionCommand msg;
    ros::Time rcv_stamp;

    Command_Data_t();
    void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
};

#endif
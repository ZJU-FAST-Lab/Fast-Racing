#include "input.h"
#include <uav_utils/converters.h>
RC_Data_t::RC_Data_t() {
    rcv_stamp = ros::Time(0);

    last_mode = -1.0;
    last_gear = -1.0;

    is_command_mode = false;
    enter_command_mode = false;
    is_api_mode = false;
    enter_api_mode = false;
}

void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg) {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    mode = ((double)msg.channels[4]-1000.0)/1000.0;
    gear = ((double)msg.channels[5]-1000.0)/1000.0;

    if ( !have_init_last_mode )
    {
        have_init_last_mode = true;
        last_mode = mode;
    } 
    if ( !have_init_last_gear )
    {
        have_init_last_gear = true;
        last_gear = gear;
    } 

    check_validity();

    if (last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE)
        enter_api_mode = true;
    else
        enter_api_mode = false;

    if (mode > API_MODE_THRESHOLD_VALUE)
        is_api_mode = true;
    else
        is_api_mode = false;

    if (is_api_mode && last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE) {
        enter_command_mode = true;
    } else if (gear < GEAR_SHIFT_VALUE) {
        enter_command_mode = false;
    }

    if (gear > GEAR_SHIFT_VALUE)
        is_command_mode = true;
    else
        is_command_mode = false;

    last_mode = mode;
    last_gear = gear;
}

void RC_Data_t::check_validity() {
    if ( mode >= -1.1 && mode <= 1.1 && gear >= -1.1 && gear <= 1.1) {
        // pass
    } else {
        ROS_ERROR("RC data validity check fail. mode=%f, gear=%f", mode, gear);
    }
}

Odom_Data_t::Odom_Data_t() {
    rcv_stamp = ros::Time(0);
    q.setIdentity();
    odom_init = false;
};

void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg) {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    uav_utils::extract_odometry(pMsg, p, v, q, w);
    odom_init = true;
}

Imu_Data_t::Imu_Data_t() {
    rcv_stamp = ros::Time(0);
    imu_init = false;
}

void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg) {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    w(0) = msg.angular_velocity.x;
    w(1) = msg.angular_velocity.y;
    w(2) = msg.angular_velocity.z;

    a(0) = msg.linear_acceleration.x;
    a(1) = msg.linear_acceleration.y;
    a(2) = msg.linear_acceleration.z;

    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;
    imu_init = true;
}

State_Data_t::State_Data_t() {
}

void State_Data_t::feed(mavros_msgs::StateConstPtr pMsg) {
    
    current_state = *pMsg; 
}

Command_Data_t::Command_Data_t() {
    rcv_stamp = ros::Time(0);
    cmd_init = false;
}

void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg) {

    static double last_time;
    static double last_yaw;
    double now_time;

    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    now_time = ros::Time::now().toSec();
    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;
    // p(0) = 1.0;
    // p(1) = 0;
    // p(2) = 2;
    

    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;

    a(0) = msg.acceleration.x;
    a(1) = msg.acceleration.y;
    a(2) = msg.acceleration.z;
    // a(0) = 1;
    // a(1) = 0;
    // a(2) = 1;
    // ROS_INFO_STREAM("A: "<<msg.yaw);
    jerk(0) = msg.jerk.x;
    jerk(1) = msg.jerk.y;
    jerk(2) = msg.jerk.z;

    yaw = uav_utils::normalize_angle(msg.yaw);
    if(!cmd_init){
        last_time = now_time;
        head_rate = 0.0;
        last_yaw = yaw;
    }
    else{
        double diff_time = now_time-last_time;
        last_time = now_time;
        double diff_yaw;
        double angle1 = yaw;
        double angle2 = last_yaw;
        last_yaw = yaw;

        double TwoPi = 2*M_PI;
        if (angle1 < 0)
            angle1 = TwoPi + angle1;
        if (angle2 < 0)
            angle2 = TwoPi + angle2;
        double dist = angle1 - angle2;
        if (dist > M_PI)
            angle1 = angle1 - TwoPi;
        //if two much on other side then invert second angle
        else if (dist < -M_PI)
            angle2 = angle2 - TwoPi;
        diff_yaw = (angle1-angle2);
        diff_time = 0.01;//hzchzc
        head_rate = diff_yaw/diff_time;
        uav_utils::limit_range(head_rate,1.0);     
        // printf("angle1: %f, angle2: %f, head_rate: %f \n, diff_time: %f",angle1,angle2,head_rate,diff_time);
    }
    cmd_init = true;
}

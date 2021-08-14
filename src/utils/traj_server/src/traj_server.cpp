#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>
using namespace std;
#define PI acos(-1)
#define HOVER_SEG_NUM 1000
#define hover_yaw PI/2
const int  _DIM_x = 0;
const int  _DIM_y = 1;
const int  _DIM_z = 2;

int hover_init_flag = 0;
int hover_stop_flag = 0;
double destination_yaw = 0;
double last_detect_time = 0;
bool emegency_flag = false;
vector<Eigen::Vector3d> history_poslist;

Eigen::Vector4d target_detected_pos;

int vis_id = 0;
using namespace std;
int emer_cur = 0;
int _poly_order_min, _poly_order_max;
int ec = 0;

double yaw_controller(double yaw, double odom_yaw){


    double k = 1;
    double error;
    if(fabs(odom_yaw-yaw)<=PI){
        error = yaw-odom_yaw;
    }
    else if(yaw-odom_yaw<-PI){
        error=yaw-odom_yaw+2*PI;
    }
    else{
        error = yaw-odom_yaw-2*PI;
    }
    double c_yaw;
    //cout<<"k*error : "<<k*error<<endl;

    //proportional control
    double ampl_threshold = 0.15;
    if(k*error > ampl_threshold)
        c_yaw = ampl_threshold;
    else if(k*error < (-1 * ampl_threshold))
        c_yaw = -1 * ampl_threshold;
    else
        c_yaw = k*error;

    //1-level control
    // double time_taw = 0.05;
    // if(error < 0.2){
    //     c_yaw = error * (1 - exp(-1 * yaw_pass_time / time_taw));      
    // }
    return c_yaw+odom_yaw;
    //return k*error+odom_yaw;
} 

class TrajectoryServer
{
private:

    // Subscribers
    ros::Subscriber _odom_sub;
    ros::Subscriber _traj_sub;
    ros::Subscriber _predict_sub;

    // publishers
    ros::Publisher _cmd_pub;
    ros::Publisher _vis_cmd_pub;
    ros::Publisher _vis_vel_pub;
    ros::Publisher _vis_acc_pub;
    ros::Publisher _vis_traj_pub;
    ros::Publisher _vis_traj_points;
    ros::Publisher _vis_desired_pos;

    // configuration for trajectory
    int _n_segment = 0;
    int _traj_id = 0;
    uint32_t _traj_flag = 0;
    Eigen::VectorXd _time;
    Eigen::Vector3d final_pos;
    vector<Eigen::MatrixXd>_normalizedcoeflist;
    vector<int> _order;

    double _vis_traj_width = 0.2;
    double mag_coeff;
    ros::Time _final_time = ros::TIME_MIN;
    ros::Time _start_time = ros::TIME_MAX;
    double _start_yaw = 0.0, _final_yaw = 0.0;
    // state of the server
    //enum ServerState{INIT, TRAJ, HOVER} state = INIT;
    enum ServerState{INIT = 0, TRAJ, HOVER} state = INIT;;
    nav_msgs::Odometry _odom;
    quadrotor_msgs::PositionCommand _cmd;
    geometry_msgs::PoseStamped _vis_cmd;

    visualization_msgs::Marker _vis_vel, _vis_acc,_vis_pos;
    visualization_msgs::Marker _vis_traj;


    sensor_msgs::PointCloud2 traj_pts;
    pcl::PointCloud<pcl::PointXYZ> traj_pts_pcd;
public:
    
    vector<Eigen::VectorXd> CList;   // Position coefficients vector, used to record all the pre-compute 'n choose k' combinatorial for the bernstein coefficients .
    vector<Eigen::VectorXd> CvList; // Velocity coefficients vector.
    vector<Eigen::VectorXd> CaList; // Acceleration coefficients vector.

    TrajectoryServer(ros::NodeHandle & handle)
    {   
        _odom_sub = 
            handle.subscribe("odometry", 50, &TrajectoryServer::rcvOdometryCallback, this, ros::TransportHints().tcpNoDelay());

        _traj_sub =
            handle.subscribe("trajectory", 2, &TrajectoryServer::rcvTrajectoryCallabck, this);

        _predict_sub  = 
            handle.subscribe( "front_pos_forpredict", 1, &TrajectoryServer::frontPosPredictCallback, this);

        _cmd_pub = 
            handle.advertise<quadrotor_msgs::PositionCommand>("position_command", 50);

        _vis_cmd_pub = 
            handle.advertise<geometry_msgs::PoseStamped>("desired_position", 50);

        _vis_vel_pub = 
            handle.advertise<visualization_msgs::Marker>("desired_velocity", 50);
        
        _vis_acc_pub = 
            handle.advertise<visualization_msgs::Marker>("desired_acceleration", 50);

        _vis_traj_pub = 
            handle.advertise<visualization_msgs::Marker>("trajectory_vis", 1);
        _vis_desired_pos = handle.advertise<visualization_msgs::Marker>("desired_pos_vis", 1);

        _vis_traj_points = 
            handle.advertise<sensor_msgs::PointCloud2>("trajectory_vis_points", 1);
        
        double pos_gain[3] = {5.7, 5.7, 6.2};
        double vel_gain[3] = {3.4, 3.4, 4.0};
        setGains(pos_gain, vel_gain);

        _vis_traj.header.stamp       = ros::Time::now();
        _vis_traj.header.frame_id    = "world";
        _vis_traj.ns = "trajectory/trajectory";
        _vis_traj.id = vis_id;
        _vis_traj.type = visualization_msgs::Marker::SPHERE_LIST;
        _vis_traj.action = visualization_msgs::Marker::ADD;
        _vis_traj.scale.x = _vis_traj_width/1.3;
        _vis_traj.scale.y = _vis_traj_width/1.3;
        _vis_traj.scale.z = _vis_traj_width/1.3;
        _vis_traj.pose.orientation.x = 0.0;
        _vis_traj.pose.orientation.y = 0.0;
        _vis_traj.pose.orientation.z = 0.0;
        _vis_traj.pose.orientation.w = 1.0;
        _vis_traj.color.r = 1.0;
        _vis_traj.color.g = 1.0;
        _vis_traj.color.b = 0.0;
        _vis_traj.color.a = 0.0;
        _vis_traj.points.clear();

        _vis_pos.header.stamp       = ros::Time::now();
        _vis_pos.header.frame_id    = "world";
        _vis_pos.ns = "/desired_pos";
        _vis_pos.id = 0;
        _vis_pos.type = visualization_msgs::Marker::SPHERE_LIST;
        _vis_pos.action = visualization_msgs::Marker::ADD;
        _vis_pos.scale.x = _vis_traj_width;
        _vis_pos.scale.y = _vis_traj_width;
        _vis_pos.scale.z = _vis_traj_width;
        _vis_pos.pose.orientation.x = 0.0;
        _vis_pos.pose.orientation.y = 0.0;
        _vis_pos.pose.orientation.z = 0.0;
        _vis_pos.pose.orientation.w = 1.0;
        _vis_pos.color.r = 1.0;
        _vis_pos.color.g = 0.0;
        _vis_pos.color.b = 1.0;
        _vis_pos.color.a = 1.0;
        _vis_pos.points.clear();

    }

    void setGains(double pos_gain[3], double vel_gain[3])
    {
        _cmd.kx[_DIM_x] = pos_gain[_DIM_x];
        _cmd.kx[_DIM_y] = pos_gain[_DIM_y];
        _cmd.kx[_DIM_z] = pos_gain[_DIM_z];

        _cmd.kv[_DIM_x] = vel_gain[_DIM_x];
        _cmd.kv[_DIM_y] = vel_gain[_DIM_y];
        _cmd.kv[_DIM_z] = vel_gain[_DIM_z];
    }

    bool cmd_flag = false;
    void frontPosPredictCallback(const nav_msgs::Path & front_pos){
        target_detected_pos << front_pos.poses[0].pose.position.x, front_pos.poses[0].pose.position.y, front_pos.poses[0].pose.position.z, front_pos.poses[0].pose.orientation.x;
    }
    void rcvOdometryCallback(const nav_msgs::Odometry & odom)
    {
        //ROS_WARN("state = %d",state);

        // if (odom.child_frame_id == "X" || odom.child_frame_id == "O") return ;
        // #1. store the odometry
        _odom = odom;
        _vis_cmd.header = _odom.header;
        _vis_cmd.header.frame_id = "/world_enu";

        if(state == INIT )
        {
            //ROS_WARN("[TRAJ SERVER] Pub initial pos command");
            //_cmd.position   = _odom.pose.pose.position;
            _cmd.position.x = _odom.pose.pose.position.x;
            _cmd.position.y = _odom.pose.pose.position.y;
            _cmd.position.z = 1.0;
            
            _cmd.header.stamp = _odom.header.stamp;
            _cmd.header.frame_id = "/world_enu";
            //_cmd.trajectory_flag = _traj_flag;
            _cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

            _cmd.velocity.x = 0.0;
            _cmd.velocity.y = 0.0;
            _cmd.velocity.z = 0.0;
            
            _cmd.acceleration.x = 0.0;
            _cmd.acceleration.y = 0.0;
            _cmd.acceleration.z = 0.0;

            _cmd.jerk.x = 0.0;
            _cmd.jerk.y = 0.0;
            _cmd.jerk.z = 0.0;
            _cmd.yaw = hover_yaw;
            _cmd_pub.publish(_cmd);

            _vis_cmd.pose.position.x = _cmd.position.x;
            _vis_cmd.pose.position.y = _cmd.position.y;
            _vis_cmd.pose.position.z = _cmd.position.z;
            _vis_cmd_pub.publish(_vis_cmd);

            return;
        }


        // change the order between #2 and #3. zxzxzxzx
        
        // #2. try to calculate the new state
        if (state == TRAJ && ( (odom.header.stamp - _start_time).toSec() / mag_coeff > (_final_time - _start_time).toSec() ) )
        {
            state = HOVER;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        }

        // #3. try to publish command
        pubPositionCommand();
    }

    void rcvTrajectoryCallabck(const quadrotor_msgs::PolynomialTrajectory & traj)
    {
        //ROS_WARN("[SERVER] Recevied The Trajectory with %.3lf.", _start_time.toSec());
        //ROS_WARN("[SERVER] Now the odom time is : ");
        // #1. try to execuse the action
        
        if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ADD)
        {   

            if(emegency_flag){
                emegency_flag = false;
                history_poslist.clear();
                emer_cur = -1;
            }
            state        = TRAJ;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
            _traj_id = traj.trajectory_id;
            _n_segment = traj.num_segment;
            _final_time = _start_time = traj.header.stamp;
            _time.resize(_n_segment);

            _order.clear();
            _normalizedcoeflist.clear();
            for (int idx = 0; idx < _n_segment; ++idx)
            {
                _final_time += ros::Duration(traj.time[idx]);
                _time(idx) = traj.time[idx];
                _order.push_back(traj.order[idx]);
            }

            _start_yaw = traj.start_yaw;
            _final_yaw = traj.final_yaw;
            mag_coeff  = traj.mag_coeff;

            
           
            
            //ROS_WARN("stack the coefficients");
            int shift = 0;
            for (int idx = 0; idx < traj.num_segment; idx++)
            {     
                int order = traj.order[idx];
                Eigen::MatrixXd coefmat;
                coefmat = Eigen::MatrixXd::Zero(3,order+1);


                for (int j = 0; j <= order; j++)
                {
                    coefmat(0, j) = traj.coef_x[shift + j];
                    coefmat(1, j) = traj.coef_y[shift + j];
                    coefmat(2, j) = traj.coef_z[shift + j];
                }
                _normalizedcoeflist.push_back(coefmat);
                shift += (order + 1);
            }
            hover_stop_flag = 0;
        }
        else if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT) 
        {
            ROS_WARN("[SERVER] Aborting the trajectory.");
            state = HOVER;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        }
        else if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE)
        {
            state = HOVER;
            hover_stop_flag = 0;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_IMPOSSIBLE;
        }
        else if(traj.action==quadrotor_msgs::PositionCommand::ACTION_STOP){
            ROS_WARN("Emergency!!!");
            state = HOVER;
            emegency_flag = true;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        }
    }

    void pubPositionCommand()
    {
        // #1. check if it is right state
        if (state == INIT) return;
        if (state == HOVER)
        {
            if (_cmd.header.frame_id != "/world_enu"){
                _cmd.position = _odom.pose.pose.position;
            }
           
           // _cmd.yaw = _odom.pose.pose.orientation.;
            // _cmd.yaw =hover_yaw;
            //hzc
                
            _cmd.header.stamp = _odom.header.stamp;
            _cmd.header.frame_id = "/world_enu";
            _cmd.trajectory_flag = _traj_flag;
            if(emegency_flag){
                if(ec==0){
                    emer_cur -= 1;
                    ec++;
                }
                else{
                    ec--;
                }
                if (emer_cur < 0) emer_cur = 0;
                _cmd.position.x = history_poslist[emer_cur][0];
                _cmd.position.y = history_poslist[emer_cur][1];
                _cmd.position.z = history_poslist[emer_cur][2];
            }
            _cmd.velocity.x = 0.0;
            _cmd.velocity.y = 0.0;
            _cmd.velocity.z = 0.0;
            
            _cmd.acceleration.x = 0.0;
            _cmd.acceleration.y = 0.0;
            _cmd.acceleration.z = 0.0;

            _cmd.jerk.x = 0.0;
            _cmd.jerk.y = 0.0;
            _cmd.jerk.z = 0.0;
        }
        // #2. locate the trajectory segment
        if (state == TRAJ)
        {
            _cmd.header.stamp = _odom.header.stamp;

            _cmd.header.frame_id = "/world_enu";
            _cmd.trajectory_flag = _traj_flag;
            _cmd.trajectory_id = _traj_id;

            double t = max(0.0, (_odom.header.stamp - _start_time).toSec());// / mag_coeff;;
        // #3. calculate the desired states
            int seg_idx;
            double dur;
            for (seg_idx = 0;
             seg_idx < _n_segment &&
             t > (dur = _time[seg_idx]);
             seg_idx++)
            {
                t -= dur;
            }
            if (seg_idx == _n_segment)
            {
                seg_idx--;
                t += _time[seg_idx];
            }
            t /= _time[seg_idx];


            const int cur_order = _order[seg_idx];
            const int cur_poly_num = cur_order + 1;
            Eigen::Vector3d pos(0.0, 0.0, 0.0);
            Eigen::Vector3d vel(0.0, 0.0, 0.0);
            Eigen::Vector3d acc(0.0, 0.0, 0.0);
            Eigen::Vector3d jerk(0.0,0.0,0.0);

            double tn = 1.0,tnvel=1.0,tnacc=1.0,tnjerk = 1.0;
            int n=1,k=1,l=2,j_1=1,j_2=2,j_3=3;
            for (int i = cur_order; i >= 0; i--)
            {
                pos += tn * _normalizedcoeflist[seg_idx].col(i);
                tn *= t;
                if(i<=cur_order-1){
                    vel+=n*tnvel*_normalizedcoeflist[seg_idx].col(i);
                    tnvel*=t;
                    n++;
                    if(i<=cur_order-2){
                        acc+=l*k*tnacc*_normalizedcoeflist[seg_idx].col(i);
                        tnacc*=t;
                        l++;
                        k++;
                        if(i<=cur_order-3){
                            jerk+=j_1*j_2*j_3*tnjerk*_normalizedcoeflist[seg_idx].col(i);
                            tnjerk*=t;
                            j_1++;
                            j_2++;
                            j_3++;
                        }
                    }
                }
            }
            vel/=_time[seg_idx];
            acc/=(_time[seg_idx]*_time[seg_idx]);
            jerk/=(_time[seg_idx]*_time[seg_idx]*_time[seg_idx]);

            _cmd.position.x = pos[0];
            _cmd.position.y = pos[1];
            _cmd.position.z = pos[2];
            _cmd.velocity.x = vel[0];
            _cmd.velocity.y = vel[1];
            _cmd.velocity.z = vel[2];
            _cmd.acceleration.x = acc[0];
            _cmd.acceleration.y = acc[1];
            _cmd.acceleration.z = acc[2];   

            _cmd.jerk.x = jerk[0];
            _cmd.jerk.y = jerk[1];
            _cmd.jerk.z = jerk[2]; 
            // _cmd.yaw = atan2(_cmd.velocity.y, _cmd.velocity.x);
             _cmd.yaw = hover_yaw;
            _cmd.yaw_dot = 0.01;
        }
        // #4. just publish

        _cmd_pub.publish(_cmd);




        _vis_cmd.header = _cmd.header;
        _vis_cmd.pose.position.x = _cmd.position.x;
        _vis_cmd.pose.position.y = _cmd.position.y;
        _vis_cmd.pose.position.z = _cmd.position.z;
        
        tf::Quaternion q_ = tf::createQuaternionFromYaw(_cmd.yaw);
        geometry_msgs::Quaternion odom_quat;
        tf::quaternionTFToMsg(q_, odom_quat);
        _vis_cmd.pose.orientation = odom_quat;
        _vis_cmd_pub.publish(_vis_cmd);
        
        _vis_vel.ns = "vel";
        _vis_vel.id = 0;
        _vis_vel.header.frame_id = "/world_enu";
        _vis_vel.type = visualization_msgs::Marker::ARROW;
        _vis_vel.action = visualization_msgs::Marker::ADD;
        _vis_vel.color.a = 1.0;
        _vis_vel.color.r = 0.0;
        _vis_vel.color.g = 1.0;
        _vis_vel.color.b = 0.0;

        _vis_vel.header.stamp = _odom.header.stamp;
        _vis_vel.points.clear();

        geometry_msgs::Point pt;
        pt.x = _cmd.position.x;
        pt.y = _cmd.position.y;
        pt.z = _cmd.position.z;
        _vis_traj.points.clear();
        _vis_traj.points.push_back(pt);
        //vis_id++;
        //_vis_traj.id = vis_id;
        _vis_traj_pub.publish(_vis_traj);

        _vis_pos.points.clear();
        _vis_pos.points.push_back(pt);
        _vis_desired_pos.publish(_vis_pos);

        pcl::PointXYZ point(pt.x, pt.y, pt.z);
        traj_pts_pcd.points.clear();
        traj_pts_pcd.points.push_back(point);
        traj_pts_pcd.width = traj_pts_pcd.points.size();
        traj_pts_pcd.height = 1;
        traj_pts_pcd.is_dense = true;
         
        pcl::toROSMsg(traj_pts_pcd, traj_pts);
        traj_pts.header.frame_id = "world";
        _vis_traj_points.publish(traj_pts);

        _vis_vel.points.push_back(pt);
        
        pt.x = _cmd.position.x + _cmd.velocity.x;
        pt.y = _cmd.position.y + _cmd.velocity.y;
        pt.z = _cmd.position.z + _cmd.velocity.z;
        
        _vis_vel.points.push_back(pt);

        _vis_vel.scale.x = 0.2;
        _vis_vel.scale.y = 0.4;
        _vis_vel.scale.z = 0.4;

        _vis_vel_pub.publish(_vis_vel);

        _vis_acc.ns = "acc";
        _vis_acc.id = 0;
        _vis_acc.header.frame_id = "/world_enu";
        _vis_acc.type = visualization_msgs::Marker::ARROW;
        _vis_acc.action = visualization_msgs::Marker::ADD;
        _vis_acc.color.a = 1.0;
        _vis_acc.color.r = 1.0;
        _vis_acc.color.g = 1.0;
        _vis_acc.color.b = 0.0;

        _vis_acc.header.stamp = _odom.header.stamp;

        _vis_acc.points.clear();
        pt.x = _cmd.position.x;
        pt.y = _cmd.position.y;
        pt.z = _cmd.position.z;

        _vis_acc.points.push_back(pt);
        
        pt.x = _cmd.position.x + _cmd.acceleration.x;
        pt.y = _cmd.position.y + _cmd.acceleration.y;
        pt.z = _cmd.position.z + _cmd.acceleration.z;

        _vis_acc.points.push_back(pt);

        _vis_acc.scale.x = 0.2;
        _vis_acc.scale.y = 0.4;
        _vis_acc.scale.z = 0.4;

        _vis_acc_pub.publish(_vis_acc);
    }
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "gradient_trajectory_server_node");
    ros::NodeHandle handle("~");

    TrajectoryServer server(handle);

    ros::spin();

    return 0;
}

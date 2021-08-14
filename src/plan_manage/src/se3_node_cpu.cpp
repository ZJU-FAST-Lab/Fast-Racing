#include <plan_manage/se3_planner.h>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
ros::Subscriber waypoints_sub,_odom_sub;
Eigen::Vector3d start_pos,start_vel,start_acc;
Eigen::Vector3d endpos,endvel;
Eigen::Vector3d odom_position;
Eigen::Vector3d odom_velocity;
Eigen::Vector3d Zero3d(0,0,0);
Eigen::MatrixXd initState,finState;
std::vector<Eigen::Vector3d> gate_list;
Eigen::Vector3d target_pt;
void rcvWaypointsCallback(const nav_msgs::Path & wp);
void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
MavGlobalPlanner* glbPlanner;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_se3_node");
    start_pos<<0,0,1;
    start_vel<<0,0,0;
    start_acc<<0,0,0;
    endvel<<0,0,0;
    initState.resize(3,3);
    finState.resize(3,3);
    Config config;
    ros::NodeHandle nh_priv("~");
    waypoints_sub= nh_priv.subscribe( "waypoints", 1, rcvWaypointsCallback );
    _odom_sub = nh_priv.subscribe("odom", 1, odom_callback);
    config.loadParameters(ros::NodeHandle("~"));
    glbPlanner = new MavGlobalPlanner(config, nh_priv);
    std::string host_ip = "localhost";
    nh_priv.getParam("host_ip", host_ip);
    Eigen::Matrix3d enutoned;
    enutoned << 0,1,0,
                1,0,0,
                0,0,-1;
    msr::airlib::RpcLibClientBase airsim_client(host_ip);
    airsim_client.confirmConnection();

    /*obtain the gate center pose*/

    std::vector<string> gates_list;
    gates_list = airsim_client.simListSceneObjects("gate.*");
    int gate_num = gates_list.size()/4;
    for(int gate_idx = 0;gate_idx<gate_num;gate_idx++){
        //
        Eigen::Vector3d position_0,position_1,position_2,position_3;
        string corner_0 = string("gate")+to_string(gate_idx)+to_string(1);
        string corner_1 = string("gate")+to_string(gate_idx)+to_string(2);
        string corner_2 = string("gate")+to_string(gate_idx)+to_string(3);
        string corner_3 = string("gate")+to_string(gate_idx)+to_string(4);
        
        msr::airlib::Pose pose_0 = airsim_client.simGetObjectPose(corner_0);
        msr::airlib::Pose pose_1 = airsim_client.simGetObjectPose(corner_1);
        msr::airlib::Pose pose_2 = airsim_client.simGetObjectPose(corner_2);
        msr::airlib::Pose pose_3 = airsim_client.simGetObjectPose(corner_3);
        position_0<<pose_0.position.y(),pose_0.position.x(),-pose_0.position.z();
        position_1<<pose_1.position.y(),pose_1.position.x(),-pose_1.position.z();
        position_2<<pose_2.position.y(),pose_2.position.x(),-pose_2.position.z();
        position_3<<pose_3.position.y(),pose_3.position.x(),-pose_3.position.z();
        Eigen::Vector3d centrl = (position_0+position_1+position_2+position_3)/4;
        gate_list.push_back(centrl);
    }

    /*goal gate pose*/
    Eigen::Vector3d goal_0,goal_1,goal_2,goal_3;
    string corner_0 = string("goal1");
    string corner_1 = string("goal2");
    string corner_2 = string("goal3");
    string corner_3 = string("goal4");
    msr::airlib::Pose pose_0 = airsim_client.simGetObjectPose(corner_0);
    msr::airlib::Pose pose_1 = airsim_client.simGetObjectPose(corner_1);
    msr::airlib::Pose pose_2 = airsim_client.simGetObjectPose(corner_2);
    msr::airlib::Pose pose_3 = airsim_client.simGetObjectPose(corner_3);
    goal_0<<pose_0.position.y(),pose_0.position.x(),-pose_0.position.z();
    goal_1<<pose_1.position.y(),pose_1.position.x(),-pose_1.position.z();
    goal_2<<pose_2.position.y(),pose_2.position.x(),-pose_2.position.z();
    goal_3<<pose_3.position.y(),pose_3.position.x(),-pose_3.position.z();
    target_pt = (goal_0+goal_1+goal_2+goal_3)/4;
    std::cout<<"goal: "<<target_pt.transpose()<<std::endl;

    
    sleep(2);
    ros::Rate rate(100);
    bool status = ros::ok();
    while(status){
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}
void rcvWaypointsCallback(const nav_msgs::Path & wp)                                                                                      
{  
    if( wp.poses[0].pose.position.z < 0.0 )
        return;
    ROS_INFO_STREAM("TARGET="<<target_pt);
    ROS_INFO("[node] receive the planning target");
    initState <<start_pos,Zero3d,Zero3d;
    finState <<target_pt,Zero3d,Zero3d;
    // glbPlanner->plan(initState,finState,&wp_list);
    glbPlanner->plan(initState,finState,&gate_list);
    // glbPlanner->plan(initState,finState);
}
void odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);
  odom_position = position;
  odom_velocity = velocity;                               
  Eigen::Quaterniond q;
  q.w() = odom->pose.pose.orientation.w;
  q.x() = odom->pose.pose.orientation.x;
  q.y() = odom->pose.pose.orientation.y;  
  q.z() = odom->pose.pose.orientation.z;
  q = q.normalized();  
}

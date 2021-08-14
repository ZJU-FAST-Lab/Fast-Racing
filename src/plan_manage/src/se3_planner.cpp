#include<plan_manage/se3_planner.h>

MavGlobalPlanner::MavGlobalPlanner(Config &conf, ros::NodeHandle &nh_)
    : config(conf), nh(nh_),
      visualization(config, nh),
      jps_pathfinder(true)
{
    point_cloud_sub_ = nh.subscribe("PointCloud_in", 10, &MavGlobalPlanner::point_cloud_cb, this,ros::TransportHints().tcpNoDelay());
    _traj_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 50);
    map_util = std::make_shared<VoxelMapUtil>();
    map_util->setParam(nh);
    jps_pathfinder.setParam(nh);
    jps_pathfinder.setMapUtil(map_util);

}
MavGlobalPlanner::~MavGlobalPlanner()
{
    delete obs_pointer;
}
void MavGlobalPlanner::point_cloud_cb(const sensor_msgs::PointCloud2 & pointcloud_map){
    if(has_map) return;
    ROS_INFO("please wait~");
    sensor_msgs::PointCloud cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(pointcloud_map,cloud);
    cloud.header.frame_id = config.odomFrame;
    obs_pointer  = new vec_Vec3f();
    *obs_pointer = DecompROS::cloud_to_vec(cloud);
    has_map = true;
    ROS_WARN("POINT BE BUILD!!!!!!!!!!!");
}
quadrotor_msgs::PolynomialTrajectory MavGlobalPlanner::traj2msg(Trajectory traj){
    static int count=0;
    quadrotor_msgs::PolynomialTrajectory traj_msg;
    traj_msg.header.seq = count;
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.header.frame_id = config.odomFrame;
    traj_msg.trajectory_id = count;
    traj_msg.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
    traj_msg.num_order = traj[0].getOrder(); // the order of polynomial
    traj_msg.num_segment = traj.getPieceNum();
    traj_msg.start_yaw = 0;
    traj_msg.final_yaw = 0;
    for(unsigned int i=0; i<traj_msg.num_segment; i++)
    {
        for (unsigned int j = 0; j <= traj[i].getOrder(); j++)
        {
          CoefficientMat coemat = traj[i].normalizePosCoeffMat();
          traj_msg.coef_x.push_back(coemat(0,j));
          traj_msg.coef_y.push_back(coemat(1,j));
          traj_msg.coef_z.push_back(coemat(2,j));
        }
        traj_msg.time.push_back(traj[i].getDuration());
        traj_msg.order.push_back(traj[i].getOrder());
    }
    traj_msg.mag_coeff = 1;
    count++;
    return traj_msg;
}


Visualization::Visualization(Config &conf, ros::NodeHandle &nh_)
    : config(conf), nh(nh_)
{
    trajPub = nh.advertise<visualization_msgs::Marker>("/visualization/trajectory", 1);
    ellipsoidPub = nh.advertise<visualization_msgs::MarkerArray>("/visualization/ellipsoid", 1);
    quadrotorPub = nh.advertise<visualization_msgs::MarkerArray>("/visualization/quadrotor", 1);
    hPolyPub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("/visualization/polyhedra", 1);
    tiltRatePub = nh.advertise<std_msgs::Float64>("/visualization/tilt_rate", 1);
    thrMagPub = nh.advertise<std_msgs::Float64>("/visualization/thrust_magnitute", 1);
}

void Visualization::visualize(const Trajectory &traj, const int samples)
{
    visualization_msgs::Marker trajMarker;

    trajMarker.id = 0;
    trajMarker.header.stamp = ros::Time::now();
    trajMarker.header.frame_id = config.odomFrame;
    trajMarker.pose.orientation.w = 1.00;
    trajMarker.action = visualization_msgs::Marker::ADD;
    trajMarker.type = visualization_msgs::Marker::LINE_STRIP;
    trajMarker.header.frame_id = config.odomFrame;
    trajMarker.ns = "trajectory";
    trajMarker.color.r = config.trajVizRGB(0);
    trajMarker.color.g = config.trajVizRGB(1);
    trajMarker.color.b = config.trajVizRGB(2);
    trajMarker.color.a = 1.00;
    trajMarker.scale.x = config.trajVizWidth;

    double dt = traj.getTotalDuration() / samples;
    geometry_msgs::Point point;
    Eigen::Vector3d pos;
    for (int i = 0; i <= samples; i++)
    {
        pos = traj.getPos(dt * i);
        point.x = pos(0);
        point.y = pos(1);
        point.z = pos(2);
        trajMarker.points.push_back(point);
    }

    trajPub.publish(trajMarker);
}

void Visualization::visualizeEllipsoid(const Trajectory &traj, const int samples)
{
    visualization_msgs::Marker ellipsoidMarker;
    visualization_msgs::MarkerArray ellipsoidMarkers;

    ellipsoidMarker.id = 0;
    ellipsoidMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
    ellipsoidMarker.mesh_resource = config.ellipsoidPath;
    ellipsoidMarker.header.stamp = ros::Time::now();
    ellipsoidMarker.header.frame_id = config.odomFrame;
    ellipsoidMarker.pose.orientation.w = 1.00;
    ellipsoidMarker.action = visualization_msgs::Marker::ADD;
    ellipsoidMarker.ns = "ellipsoids";
    ellipsoidMarker.color.r = config.ellipsoidVizRGBA(0);
    ellipsoidMarker.color.g = config.ellipsoidVizRGBA(1);
    ellipsoidMarker.color.b = config.ellipsoidVizRGBA(2);
    ellipsoidMarker.color.a = config.ellipsoidVizRGBA(3);
    ellipsoidMarker.scale.x = config.horizHalfLen * 2.0;
    ellipsoidMarker.scale.y = config.horizHalfLen * 2.0;
    ellipsoidMarker.scale.z = config.vertHalfLen * 2.0;

    ellipsoidMarker.action = visualization_msgs::Marker::DELETEALL;
    ellipsoidMarkers.markers.push_back(ellipsoidMarker);
    ellipsoidPub.publish(ellipsoidMarkers);
    ellipsoidMarker.action = visualization_msgs::Marker::ADD;
    ellipsoidMarkers.markers.clear();

    double dt = traj.getTotalDuration() / samples;
    geometry_msgs::Point point;
    Eigen::Vector3d pos;
    Eigen::Matrix3d rotM;
    Eigen::Quaterniond quat;
    for (int i = 0; i <= samples; i++)
    {
        pos = traj.getPos(dt * i);
        ellipsoidMarker.pose.position.x = pos(0);
        ellipsoidMarker.pose.position.y = pos(1);
        ellipsoidMarker.pose.position.z = pos(2);
        traj.getRotation(dt * i, 0.0, config.gravAcc, rotM);
        quat = Eigen::Quaterniond(rotM);
        ellipsoidMarker.pose.orientation.w = quat.w();
        ellipsoidMarker.pose.orientation.x = quat.x();
        ellipsoidMarker.pose.orientation.y = quat.y();
        ellipsoidMarker.pose.orientation.z = quat.z();
        ellipsoidMarkers.markers.push_back(ellipsoidMarker);
        ellipsoidMarker.id++;
    }

    ellipsoidPub.publish(ellipsoidMarkers);
}

void Visualization::visualizeQuadrotor(const Trajectory &traj, const int samples)
{
    visualization_msgs::Marker quadrotorMarker;
    visualization_msgs::MarkerArray quadrotorMarkers;

    quadrotorMarker.id = 0;
    quadrotorMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
    quadrotorMarker.mesh_use_embedded_materials = true;
    quadrotorMarker.mesh_resource = config.quadrotorPath;
    quadrotorMarker.header.stamp = ros::Time::now();
    quadrotorMarker.header.frame_id = config.odomFrame;
    quadrotorMarker.pose.orientation.w = 1.00;
    quadrotorMarker.action = visualization_msgs::Marker::ADD;
    quadrotorMarker.ns = "quadrotor";
    quadrotorMarker.color.r = 0.0;
    quadrotorMarker.color.g = 0.0;
    quadrotorMarker.color.b = 0.0;
    quadrotorMarker.color.a = 0.0;
    quadrotorMarker.scale.x = (config.horizHalfLen+config.safeMargin*3.3+0.2) * sqrt(2.0);
    quadrotorMarker.scale.y = (config.horizHalfLen+config.safeMargin*3.3+0.2) * sqrt(2.0);
    quadrotorMarker.scale.z = config.vertHalfLen * 8.0;

    quadrotorMarker.action = visualization_msgs::Marker::DELETEALL;
    quadrotorMarkers.markers.push_back(quadrotorMarker);
    quadrotorPub.publish(quadrotorMarkers);
    quadrotorMarker.action = visualization_msgs::Marker::ADD;
    quadrotorMarkers.markers.clear();

    // double dt = traj.getTotalDuration() / samples;
    //hzc30
    double dt = 5.0/samples;
    geometry_msgs::Point point;
    Eigen::Vector3d pos;
    Eigen::Matrix3d rotM;
    Eigen::Quaterniond quat;
    for (int i = 0; i <= samples; i++)
    {
        pos = traj.getPos(6+dt * i);//4.5
        quadrotorMarker.pose.position.x = pos(0);
        quadrotorMarker.pose.position.y = pos(1);
        quadrotorMarker.pose.position.z = pos(2);
        // traj.getRotation(6+dt * i, M_PI_4, config.gravAcc, rotM);hzc
        traj.getRotation(6+dt * i, M_PI_2, config.gravAcc, rotM);
        quat = Eigen::Quaterniond(rotM);
        quadrotorMarker.pose.orientation.w = quat.w();
        quadrotorMarker.pose.orientation.x = quat.x();
        quadrotorMarker.pose.orientation.y = quat.y();
        quadrotorMarker.pose.orientation.z = quat.z();
        quadrotorMarkers.markers.push_back(quadrotorMarker);
        quadrotorMarker.id++;
    }

    quadrotorPub.publish(quadrotorMarkers);
}

void Visualization::visualizePolyH(const vec_E<Polyhedron3D> &polyhedra)
{
    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polyhedra);
    poly_msg.header.frame_id = config.odomFrame;
    poly_msg.header.stamp = ros::Time::now();
    hPolyPub.publish(poly_msg);
}

void Visualization::visualizeProfile(const Trajectory &traj, const double &t)
{
    Eigen::Vector3d tiltRate = traj.getTiltRate(t, config.gravAcc);
    Eigen::Vector3d thrAcc = traj.getAcc(t);
    thrAcc(2) += config.gravAcc;

    std_msgs::Float64 tilt, thr;
    tilt.data = tiltRate.norm();
    thr.data = thrAcc.norm();

    tiltRatePub.publish(tilt);
    thrMagPub.publish(thr);
}

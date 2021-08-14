#ifndef SE3_PLANNER_h
#define SE3_PLANNER_h
#include <ros/package.h>
#include <cmath>
#include <iostream>
#include <Eigen/Eigen>
#include <memory>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "nav_msgs/Path.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include <decomp_basis/data_type.h>
#include <decomp_geometry/polyhedron.h>
#include "decomp_ros_utils/data_ros_utils.h"
#include "decomp_util/ellipsoid_decomp.h"
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <jps_planner/jps_planner/jps_planner.h>
#include <jps_collision/map_util.h>
#include <std_msgs/Float64.h>
#include "se3gcopter/trajectory.hpp"

using namespace std;        
using namespace JPS;
struct Config
{


    std::string odomFrame;

    // Params
    double scaleSI;
    double mapHeight;
    Eigen::Vector3d polyhedronBox;
    double rho;
    double totalT;
    int qdIntervals;
    double horizHalfLen;
    double vertHalfLen;
    double safeMargin;
    double velMax;
    double thrustAccMin;
    double thrustAccMax;
    double bodyRateMax;
    double gravAcc;
    Eigen::Vector4d penaltyPVTB;
    bool useC2Diffeo;
    double optRelTol;
    double trajVizWidth;
    Eigen::Vector3d trajVizRGB;
    std::string routeStoragePath;
    std::string ellipsoidPath;
    Eigen::Vector4d ellipsoidVizRGBA;
    std::string quadrotorPath;
    inline void loadParameters(const ros::NodeHandle &nh_priv)
    {
        std::vector<double> vecPolyhedronBox, vecPenaltyPVTB, vecTrajVizRGB, vecEllipsoidVizRGBA;
        std::string packagePath = ros::package::getPath("plan_manage");
        packagePath += packagePath.back() == '/' ? "" : "/";
        std::string packageUrl = "package://plan_manage/";
        nh_priv.getParam("OdomFrame", odomFrame);
        nh_priv.getParam("ScaleSI", scaleSI);
        nh_priv.getParam("MapHeight", mapHeight);
        nh_priv.getParam("PolyhedronBox", vecPolyhedronBox);
        polyhedronBox << vecPolyhedronBox[0], vecPolyhedronBox[1], vecPolyhedronBox[2];
        nh_priv.getParam("Rho", rho);
        nh_priv.getParam("TotalT", totalT);
        nh_priv.getParam("QdIntervals", qdIntervals);
        nh_priv.getParam("HorizHalfLen", horizHalfLen);
        nh_priv.getParam("VertHalfLen", vertHalfLen);
        nh_priv.getParam("SafeMargin", safeMargin);
        nh_priv.getParam("VelMax", velMax);
        nh_priv.getParam("ThrustAccMin", thrustAccMin);
        nh_priv.getParam("ThrustAccMax", thrustAccMax);
        nh_priv.getParam("BodyRateMax", bodyRateMax);
        nh_priv.getParam("GravAcc", gravAcc);
        nh_priv.getParam("PenaltyPVTB", vecPenaltyPVTB);
        penaltyPVTB << vecPenaltyPVTB[0], vecPenaltyPVTB[1], vecPenaltyPVTB[2], vecPenaltyPVTB[3];
        nh_priv.getParam("UseC2Diffeo", useC2Diffeo);
        nh_priv.getParam("OptRelTol", optRelTol);
        nh_priv.getParam("TrajVizWidth", trajVizWidth);
        nh_priv.getParam("TrajVizRGB", vecTrajVizRGB);
        trajVizRGB << vecTrajVizRGB[0], vecTrajVizRGB[1], vecTrajVizRGB[2];
        nh_priv.getParam("RouteStoragePath", routeStoragePath);
        routeStoragePath = packagePath + routeStoragePath;
        nh_priv.getParam("EllipsoidPath", ellipsoidPath);
        ellipsoidPath = packageUrl + ellipsoidPath;
        nh_priv.getParam("EllipsoidVizRGBA", vecEllipsoidVizRGBA);
        ellipsoidVizRGBA << vecEllipsoidVizRGBA[0], vecEllipsoidVizRGBA[1], vecEllipsoidVizRGBA[2], vecEllipsoidVizRGBA[3];
        nh_priv.getParam("QuadrotorPath", quadrotorPath);
        quadrotorPath = packageUrl + quadrotorPath;
    }
};
class Visualization
{
public:
  Visualization(Config &conf, ros::NodeHandle &nh_);

  Config config;
  ros::NodeHandle nh;
  
  ros::Publisher trajPub;
  ros::Publisher ellipsoidPub;
  ros::Publisher quadrotorPub;
  ros::Publisher hPolyPub;
  ros::Publisher tiltRatePub;
  ros::Publisher thrMagPub;

  void visualize(const Trajectory &traj, const int samples = 1000);
  void visualizeEllipsoid(const Trajectory &traj, const int samples = 1000);
  void visualizeQuadrotor(const Trajectory &traj, const int samples = 1000);
  void visualizePolyH(const vec_E<Polyhedron3D> &polyhedra);
  void visualizeProfile(const Trajectory &traj, const double &t);
};

class MavGlobalPlanner
{
public:
  MavGlobalPlanner(Config &conf, ros::NodeHandle &nh_);
  ~MavGlobalPlanner();

  Config config;
  ros::NodeHandle nh;

  ros::Subscriber targetSub;
  ros::Subscriber point_cloud_sub_;
  void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);

  ros::Publisher cloud_pub;
  ros::Publisher path_pub;
  ros::Publisher _traj_pub;

  vec_Vec3f *obs_pointer;
  std::shared_ptr<VoxelMapUtil> map_util;
  JPSPlanner3D jps_pathfinder;
  Visualization visualization;
  ros::Time lastIniStamp;
  quadrotor_msgs::PolynomialTrajectory traj2msg(Trajectory traj);
  bool has_map_(){return has_map;};
  void plan(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,std::vector<Eigen::Vector3d>* wp_list = NULL);
private:
  bool has_map = false;
  void point_cloud_cb(const sensor_msgs::PointCloud2 & pointcloud_map);
  
 
};




#endif

#include "ros/ros.h"
#include "airsim_ros_wrapper.h"
#include <ros/spinner.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <math.h>
#include <random>
#include<geometry_msgs/Point32.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap_server/OctomapServer.h>
#define map_f 1
using namespace octomap;
using namespace octomap_msgs;
using namespace octomap_server;
ros::Publisher  airsim_map_pub;
sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;
//octree
OctomapServer* server_drone;
bool use_octree;
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "map_node");
    ros::NodeHandle nh("~");
    ros::NodeHandle private_nh("~");
    std::string host_ip;
    double resolution;
    std::string world_frameid;
    Eigen::Matrix3d enutoned;
    enutoned << 0,1,0,
                1,0,0,
                0,0,-1;
    nh.param("host_ip", host_ip,std::string("localhost"));
    nh.param("resolution",resolution,0.1);
    nh.param("world_frame_id",world_frameid,std::string("/world_enu"));
    nh.param("use_octree",use_octree,false);
    if(use_octree)
      server_drone = new OctomapServer(private_nh, nh,world_frameid);
    airsim_map_pub   = nh.advertise<sensor_msgs::PointCloud2>("/airsim_global_map", 1);  
    msr::airlib::RpcLibClientBase airsim_client_map_(host_ip);
    airsim_client_map_.confirmConnection();
    vector<string> objects_list;
    ros::Rate rate(map_f);
    int count=0;
    while(ros::ok()){
        ros::spinOnce();
        if(count<=10){
          cloudMap.points.clear();
          if(use_octree)
            server_drone->m_octree->clear();
          objects_list = airsim_client_map_.simListSceneObjects("Cube.*");
          for(int i =0;i<objects_list.size();i++){  
              msr::airlib::Pose cube_pose;
              msr::airlib::Vector3r cube_scale;
              string object_name;
              object_name = objects_list[i];
              cube_pose = airsim_client_map_.simGetObjectPose(object_name);
              cube_scale = airsim_client_map_.simGetObjectScale(object_name);
              Eigen::Vector3d position;
              Eigen::Quaterniond q;
              Eigen::Matrix3d body2world;
              q.w() = cube_pose.orientation.w();
              q.x() = cube_pose.orientation.x();
              q.y() = cube_pose.orientation.y();
              q.z() = cube_pose.orientation.z();
              if(world_frameid==std::string("/world_enu")){
                  position<<cube_pose.position.y(),cube_pose.position.x(),-cube_pose.position.z();
                  body2world = enutoned*q.toRotationMatrix();
              }
              else if(world_frameid==std::string("/world_ned")) {
                  position << cube_pose.position.x(),cube_pose.position.y(),cube_pose.position.z();
                  body2world = q.toRotationMatrix();
              }
              else{
                ROS_ERROR("wrong map frame id!");
              }
              if(count==0)
                ROS_INFO("We are sending global map~ Please wait~");
              double lx,ly,lz;
                for(lx = -cube_scale.x()/2; lx<cube_scale.x()/2+resolution;lx+=resolution){
                  for(ly = -cube_scale.y()/2; ly<cube_scale.y()/2+resolution;ly+=resolution){
                      for(lz = -cube_scale.z()/2; lz<cube_scale.z()/2+resolution;lz+=resolution){
                          Eigen::Vector3d obs_body;
                          obs_body << lx,ly,lz;
                          Eigen::Vector3d obs_world;
                          obs_world = body2world*obs_body+position;
                          pcl::PointXYZ pt;
                          pt.x = obs_world[0];
                          pt.y = obs_world[1];
                          pt.z = obs_world[2];
                          cloudMap.points.push_back(pt); 
                          geometry_msgs::Point32 cpt;
                          cpt.x = pt.x;
                          cpt.y = pt.y;
                          cpt.z = pt.z;
                          if(use_octree)
                            server_drone->m_octree->updateNode(point3d(pt.x+1e-5,pt.y+1e-5,pt.z+1e-5), true);
                      }
                  }
              }
          }
          if(use_octree)
            server_drone->publishAll();
          count++;
          cloudMap.width = cloudMap.points.size();
          cloudMap.height = 1;
          cloudMap.is_dense = true;
          pcl::toROSMsg(cloudMap, globalMap_pcd);
          globalMap_pcd.header.frame_id = world_frameid; 
          airsim_map_pub.publish(globalMap_pcd);
          ROS_INFO("send global map");
        }
        rate.sleep();
    }
    return 0;
} 
/*
{
  "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/master/docs/settings.md",
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "Vehicles": {
    "drone_1": {
      "VehicleType": "SimpleFlight",
      "DefaultVehicleState": "Armed",
      "Sensors": {
        "Barometer": {
          "SensorType": 1,
          "Enabled" : false
        },
        "Imu": {
          "SensorType": 2,
          "Enabled" : true
        },
        "Gps": {
          "SensorType": 3,
          "Enabled" : false
        },
        "Magnetometer": {
          "SensorType": 4,
          "Enabled" : false
        },
        "Distance": {
          "SensorType": 5,
          "Enabled" : false
        },
        "Lidar": {
          "SensorType": 6,
          "Enabled" : false,
          "NumberOfChannels": 16,
          "RotationsPerSecond": 10,
          "PointsPerSecond": 100000,
          "X": 0, "Y": 0, "Z": -1,
          "Roll": 0, "Pitch": 0, "Yaw" : 0,
          "VerticalFOVUpper": 0,
          "VerticalFOVLower": -0,
          "HorizontalFOVStart": -90,
          "HorizontalFOVEnd": 90,
          "DrawDebugPoints": true,
          "DataFrame": "SensorLocalFrame"
        }
      },
      "Cameras": {
        "front_center_custom": {
          "CaptureSettings": [
            {
              "PublishToRos": 1,
              "ImageType": 3,
              "Width": 320,
              "Height": 240,
              "FOV_Degrees": 90,
              "DepthOfFieldFstop": 2.8,
              "DepthOfFieldFocalDistance": 200.0, 
              "DepthOfFieldFocalRegion": 200.0,
              "TargetGamma": 1.5
            }
          ],
          "Pitch": 0.0,
          "Roll": 0.0,
          "X": 0.25,
          "Y": 0.0,
          "Yaw": 0.0,
          "Z": 0.3
        }
      },
      "X": 0, "Y": 0, "Z": 0,
      "Pitch": 0, "Roll": 0, "Yaw": 0
    }
  },
  "SubWindows": [
    {"WindowID": 1, "ImageType": 3, "CameraName": "front_center_custom", "Visible": false}
  ] 
}

*/
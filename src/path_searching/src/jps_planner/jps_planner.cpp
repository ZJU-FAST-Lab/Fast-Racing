#include <jps_planner/jps_planner/jps_planner.h>
#include <geometry_msgs/PoseStamped.h>

template <int Dim>
JPSPlanner<Dim>::JPSPlanner(bool verbose): planner_verbose_(verbose) {
  planner_verbose_ = verbose;
  if(planner_verbose_)
    printf(ANSI_COLOR_CYAN "JPS PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
}

template<int Dim>
void JPSPlanner<Dim>::setParam(ros::NodeHandle& nh){
  nh.param("world_frame_id",world_frame_id,std::string("/world_enu"));
  /*visualization*/
  rawpath_jps_pub_ = nh.advertise<nav_msgs::Path>("/rawpath_jps_vis",1);
  optipath_jps_pub_ = nh.advertise<nav_msgs::Path>("/optipath_jps_vis",1);
  key_grid_jps_pub_ = nh.advertise<visualization_msgs::Marker>("/keygrid_jps_vis", 1);
  sample_path_jps_pub_ = nh.advertise<visualization_msgs::Marker>("/sample_path_jps_vis", 1);
  map_update_timer = nh.createTimer(ros::Duration(1.0),&JPSPlanner::map_update_timer_cb,this);
  // ma
  // map_update_timer = nh.create

}

template<int Dim>
void JPSPlanner<Dim>::map_update_timer_cb(const ros::TimerEvent& event){
  if(has_map){
    return;}
  if(!map_util_set) {
    return;}
  if(!map_util_->has_map_()) {
    return;}
  updateMap();
  has_map = true;
  
  ROS_WARN("JPS Planner map has been updated");
}
template <int Dim>
void JPSPlanner<Dim>::setMapUtil(const std::shared_ptr<JPS::MapUtil<Dim>> &map_util) {
  map_util_ = map_util;
  map_util_set = true;
}

template <int Dim>
int JPSPlanner<Dim>::status() { return status_; }

template <int Dim>
vec_Vecf<Dim> JPSPlanner<Dim>::getPath() { return path_; }

template <int Dim>
vec_Vecf<Dim> JPSPlanner<Dim>::getRawPath() { return raw_path_; }
template <int Dim>
vec_Vecf<Dim> JPSPlanner<Dim>::getSamplePath(){ return sample_path_;}
template <int Dim>
vec_Vecf<Dim> JPSPlanner<Dim>::removeCornerPts(const vec_Vecf<Dim> &path) {
  if (path.size() < 2)
    return path;

  // cut zigzag segment
  vec_Vecf<Dim> optimized_path;
  Vecf<Dim> pose1 = path[0];
  Vecf<Dim> pose2 = path[1];
  Vecf<Dim> prev_pose = pose1;
  optimized_path.push_back(pose1);
  decimal_t cost1, cost2, cost3;

  if (!map_util_->isBlocked(pose1, pose2))
    cost1 = (pose1 - pose2).norm();
  else
    cost1 = std::numeric_limits<decimal_t>::infinity();

  for (unsigned int i = 1; i < path.size() - 1; i++) {
    pose1 = path[i];
    pose2 = path[i + 1];
    if (!map_util_->isBlocked(pose1, pose2))
      cost2 = (pose1 - pose2).norm();
    else
      cost2 = std::numeric_limits<decimal_t>::infinity();

    if (!map_util_->isBlocked(prev_pose, pose2))
      cost3 = (prev_pose - pose2).norm();
    else
      cost3 = std::numeric_limits<decimal_t>::infinity();

    if (cost3 < cost1 + cost2)
      cost1 = cost3;
    else {
      optimized_path.push_back(path[i]);
      cost1 = (pose1 - pose2).norm();
      prev_pose = pose1;
    }
  }

  optimized_path.push_back(path.back());
  return optimized_path;
}

template <int Dim>
vec_Vecf<Dim> JPSPlanner<Dim>::removeLinePts(const vec_Vecf<Dim> &path) {
  if (path.size() < 3)
    return path;

  vec_Vecf<Dim> new_path;
  new_path.push_back(path.front());
  for (unsigned int i = 1; i < path.size() - 1; i++) {
    Vecf<Dim> p = (path[i + 1] - path[i]) - (path[i] - path[i - 1]);
    if(Dim == 3) {
      if (fabs(p(0)) + fabs(p(1)) + fabs(p(2)) > 1e-2)
        new_path.push_back(path[i]);
    }
    else {
      if (fabs(p(0)) + fabs(p(1)) > 1e-2)
        new_path.push_back(path[i]);
    }
  }
  new_path.push_back(path.back());
  return new_path;
}
template<int Dim>
vec_Vecf<Dim> JPSPlanner<Dim>::samplePath(const vec_Vecf<Dim> & path){
  vec_Vecf<Dim> new_path;
  new_path.push_back(path.front());
  Veci<Dim> last_index =  map_util_->floatToInt(path.front());
  for(int i=0;i<path.size()-1;i++){
    //i i+1
    Vecf<Dim> p1 = path[i];
    Vecf<Dim> p2 = path[i+1];
    double d,l;
    l = (p2-p1).norm();
    for(d=0;d<=l;d+=0.02){
      Vecf<Dim> tmp_pos = (l-d)/l*p1+d/l*p2;
      Veci<Dim> index = map_util_->floatToInt(tmp_pos);
      if(index!=last_index){
        new_path.push_back(map_util_->intToFloat(index));
        last_index = index;
      }
    }
    if(d<l){
      d=l;
      Vecf<Dim> tmp_pos = (l-d)/l*p1+d/l*p2;
      Veci<Dim> index = map_util_->floatToInt(tmp_pos);
      if(index!=last_index){
        new_path.push_back(map_util_->intToFloat(index));
        last_index = index;
      }
    }
  }
  return new_path;
}


template <int Dim>
vec_Vecf<Dim> JPSPlanner<Dim>::getOpenSet() const {
  vec_Vecf<Dim> ps;
  const auto ss = graph_search_->getOpenSet();
  for(const auto& it: ss) {
    if(Dim == 3) {
      Veci<Dim> pn;
      pn << it->x, it->y, it->z;
      ps.push_back(map_util_->intToFloat(pn));
    }
    else
      ps.push_back(map_util_->intToFloat(Veci<Dim>(it->x, it->y)));
  }
  return ps;
}
/*
void publish_rawpath();
    void publish_optipath();
    void publish_keygrid();
    void publish_samplepath();
    void publishAll();
*/
template<int Dim>
void JPSPlanner<Dim>::publish_rawpath(){
  nav_msgs::Path path_msg;
  for(int i = 0;i<raw_path_.size();i++){
        geometry_msgs::PoseStamped pt;
        pt.pose.position.x =  raw_path_[i](0);
        pt.pose.position.y =  raw_path_[i](1);
        pt.pose.position.z =  raw_path_[i](2);
        path_msg.poses.push_back(pt);//维护waypoints
  }
  path_msg.header.stamp       = ros::Time::now();
  path_msg.header.frame_id    = world_frame_id;
  rawpath_jps_pub_.publish(path_msg);
}
template<int Dim>
void JPSPlanner<Dim>::publish_optipath(){
  nav_msgs::Path path_msg;
  for(int i = 0;i<path_.size();i++){
      geometry_msgs::PoseStamped pt;
      pt.pose.position.x =  path_[i][0];
      pt.pose.position.y =  path_[i][1];
      pt.pose.position.z =  path_[i][2];
      path_msg.poses.push_back(pt);//维护waypoints
  }
  path_msg.header.stamp       = ros::Time::now();
  path_msg.header.frame_id    = world_frame_id;
  optipath_jps_pub_.publish(path_msg);
}
template<int Dim>
void JPSPlanner<Dim>::publish_keygrid(){
    if(raw_path_.size()<1)
        return ;
    visualization_msgs::Marker grid_vis;
    grid_vis.header.stamp       = ros::Time::now();
    grid_vis.header.frame_id    = world_frame_id;
    grid_vis.ns = "jps/keygrid";
    grid_vis.action = visualization_msgs::Marker::ADD;
    //_vis_traj_width = 0.05;
    grid_vis.scale.x = 0.1;
    grid_vis.scale.y = 0.1;
    grid_vis.scale.z = 0.1;
    grid_vis.pose.orientation.x = 0.0;
    grid_vis.pose.orientation.y = 0.0;
    grid_vis.pose.orientation.z = 0.0;
    grid_vis.pose.orientation.w = 1.0;
    grid_vis.id = 0;
    grid_vis.color.a = 1.0;
    grid_vis.color.r = 1.0;
    grid_vis.color.g = 0.0;
    grid_vis.color.b = 1.0;//purple
    grid_vis.type = visualization_msgs::Marker::CUBE_LIST;
    grid_vis.points.clear();
    geometry_msgs::Point pt;
    for(int i=0;i<raw_path_.size();i++){
        pt.x = raw_path_[i](0);
        pt.y = raw_path_[i](1);
        pt.z = raw_path_[i](2);
        grid_vis.points.push_back(pt);
    }
    key_grid_jps_pub_.publish(grid_vis);
}
template<int Dim>
void JPSPlanner<Dim>::publish_samplepath(){
   if(sample_path_.size()<1)
        return ;
    visualization_msgs::Marker grid_vis;
    grid_vis.header.stamp       = ros::Time::now();
    grid_vis.header.frame_id    = world_frame_id;
    grid_vis.ns = "jps/sample";
    grid_vis.action = visualization_msgs::Marker::ADD;
    //_vis_traj_width = 0.05;
    grid_vis.scale.x = 0.1;
    grid_vis.scale.y = 0.1;
    grid_vis.scale.z = 0.1;
    grid_vis.pose.orientation.x = 0.0;
    grid_vis.pose.orientation.y = 0.0;
    grid_vis.pose.orientation.z = 0.0;
    grid_vis.pose.orientation.w = 1.0;
    grid_vis.id = 0;
    grid_vis.color.a = 1.0;
    grid_vis.color.r = 1.0;
    grid_vis.color.g = 0.0;
    grid_vis.color.b = 0.0;//purple
    grid_vis.type = visualization_msgs::Marker::CUBE_LIST;
    grid_vis.points.clear();
    geometry_msgs::Point pt;
    for(int i=0;i<sample_path_.size();i++){
        pt.x = sample_path_[i](0);
        pt.y = sample_path_[i](1);
        pt.z = sample_path_[i](2);
        grid_vis.points.push_back(pt);
    }
    sample_path_jps_pub_.publish(grid_vis);
}
template <int Dim>
void JPSPlanner<Dim>::publishAll(){
  publish_rawpath();
  publish_optipath();
  publish_keygrid();
  publish_samplepath();
}

template <int Dim>
vec_Vecf<Dim> JPSPlanner<Dim>::getCloseSet() const {
  vec_Vecf<Dim> ps;
  const auto ss = graph_search_->getCloseSet();
  for(const auto& it: ss) {
    if(Dim == 3) {
      Veci<Dim> pn;
      pn << it->x, it->y, it->z;
      ps.push_back(map_util_->intToFloat(pn));
    }
    else
      ps.push_back(map_util_->intToFloat(Veci<Dim>(it->x, it->y)));
  }
  return ps;
}

template <int Dim>
vec_Vecf<Dim> JPSPlanner<Dim>::getAllSet() const {
  vec_Vecf<Dim> ps;
  const auto ss = graph_search_->getAllSet();
  for(const auto& it: ss) {
    if(Dim == 3) {
      Veci<Dim> pn;
      pn << it->x, it->y, it->z;
      ps.push_back(map_util_->intToFloat(pn));
    }
    else
      ps.push_back(map_util_->intToFloat(Veci<Dim>(it->x, it->y)));
  }
  return ps;
}

template <int Dim>
void JPSPlanner<Dim>::updateMap() {
  Veci<Dim> dim = map_util_->getDim();

  if(Dim == 3) {
    cmap_.resize(dim(0)*dim(1)*dim(2));
    for( int z = 0; z < dim(2); ++z) {
      for( int y = 0; y < dim(1); ++y) {
        for( int x = 0; x < dim(0); ++x) {
          Veci<Dim> pn;
          pn << x, y, z;
          cmap_[x+y*dim(0)+z*dim(0)*dim(1)] = map_util_->isOccupied(pn) ? 1:0;
        }
      }
    }
  }
  else {
    cmap_.resize(dim(0)*dim(1));
      for( int y = 0; y < dim(1); ++y)
        for( int x = 0; x < dim(0); ++x)
          cmap_[x+y*dim(0)] = map_util_->isOccupied(Veci<Dim>(x,y)) ? 1:0;
  }
}

template <int Dim>
bool JPSPlanner<Dim>::plan(const Vecf<Dim> &start, const Vecf<Dim> &goal, decimal_t eps, bool use_jps) {
  if(planner_verbose_){
    std::cout <<"Start: " << start.transpose() << std::endl;
    std::cout <<"Goal:  " << goal.transpose() << std::endl;
    std::cout <<"Epsilon:  " << eps << std::endl;
  }

  path_.clear();
  raw_path_.clear();
  status_ = 0;

  const Veci<Dim> start_int = map_util_->floatToInt(start);
  if (!map_util_->isFree(start_int)) {
    if(planner_verbose_) {
      if (map_util_->isOccupied(start_int))
        printf(ANSI_COLOR_RED "start is occupied!\n" ANSI_COLOR_RESET);
      else if (map_util_->isUnknown(start_int))
        printf(ANSI_COLOR_RED "start is unknown!\n" ANSI_COLOR_RESET);
      else {
        printf(ANSI_COLOR_RED "start is outside!\n" ANSI_COLOR_RESET);
        std::cout << "startI: " << start_int.transpose() << std::endl;
        std::cout <<"Map origin: " << map_util_->getOrigin().transpose() << std::endl;
        std::cout <<"Map dim: " << map_util_->getDim().transpose() << std::endl;
      }
    }
    status_ = 1;
    return false;
  }

  const Veci<Dim> goal_int = map_util_->floatToInt(goal);
  if (!map_util_->isFree(goal_int)) {
    if(planner_verbose_)
      printf(ANSI_COLOR_RED "goal is not free!\n" ANSI_COLOR_RESET);
    status_ = 2;
    return false;
  }

  if(cmap_.empty()) {
    if(planner_verbose_)
      printf(ANSI_COLOR_RED "need to set cmap, call updateMap()!\n" ANSI_COLOR_RESET);
    return -1;
  }

  const Veci<Dim> dim = map_util_->getDim();

  if(Dim == 3) {
    graph_search_ = std::make_shared<JPS::GraphSearch>(cmap_.data(), dim(0), dim(1), dim(2), eps, planner_verbose_);
    graph_search_->plan(start_int(0), start_int(1), start_int(2), goal_int(0), goal_int(1), goal_int(2), use_jps);
  }
  else {
    graph_search_ = std::make_shared<JPS::GraphSearch>(cmap_.data(), dim(0), dim(1), eps, planner_verbose_);
    graph_search_->plan(start_int(0), start_int(1), goal_int(0),  goal_int(1), use_jps);
  }

  const auto path = graph_search_->getPath();
  if (path.size() < 1) {
    if(planner_verbose_)
      std::cout << ANSI_COLOR_RED "Cannot find a path from " << start.transpose() <<" to " << goal.transpose() << " Abort!" ANSI_COLOR_RESET << std::endl;
    status_ = -1;
    return false;
  }

  //**** raw path, s --> g
  vec_Vecf<Dim> ps;
  for (const auto &it : path) {
    if(Dim == 3) {
      Veci<Dim> pn;
      pn << it->x, it->y, it->z;
      ps.push_back(map_util_->intToFloat(pn));
    }
    else
      ps.push_back(map_util_->intToFloat(Veci<Dim>(it->x, it->y)));
  }

  raw_path_ = ps;
  std::reverse(std::begin(raw_path_), std::end(raw_path_));

  // Simplify the raw path
  //path_ = removeLinePts(raw_path_);
  //path_ = removeCornerPts(path_);
  path_ = removeCornerPts(raw_path_);
  std::reverse(std::begin(path_), std::end(path_));
  path_ = removeCornerPts(path_);
  std::reverse(std::begin(path_), std::end(path_));
  path_ = removeLinePts(path_);
  sample_path_ = samplePath(path_);
  return true;
}


template class JPSPlanner<2>;

template class JPSPlanner<3>;


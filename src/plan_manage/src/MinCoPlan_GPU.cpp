#include<plan_manage/se3_planner.h>
#include<se3gcopter/se3gcopter_gpu.hpp>
void MavGlobalPlanner::plan(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,vector<Eigen::Vector3d>* wp_list){
    if(!has_map||!jps_pathfinder.has_map_()) return;
    Vector3d zeroVec(0.0,0.0,0.0);
    Vector3d start_pt;
    Vector3d end_pt;
    start_pt = iniState.col(0);
    end_pt  = finState.col(0);
    vec_Vec3f path;
    if(!wp_list){
        double t1 = ros::Time::now().toSec();
        jps_pathfinder.plan(Vecf<3>(start_pt[0],start_pt[1],start_pt[2]),Vecf<3>(end_pt[0],end_pt[1],end_pt[2]),1,false);
        double t2 = ros::Time::now().toSec();
        path = jps_pathfinder.getSamplePath();
        double t3 = ros::Time::now().toSec();
        jps_pathfinder.publishAll();
    }else{
        vec_Vec3f path_0;
        jps_pathfinder.plan(Vecf<3>(start_pt[0],start_pt[1],start_pt[2]),Vecf<3>((*wp_list)[0][0],(*wp_list)[0][1],(*wp_list)[0][2]),1,false);
        path_0 = jps_pathfinder.getSamplePath();       
        for(int i = 0;i< wp_list->size()-1;i++){
            vec_Vec3f tmp_path;
            jps_pathfinder.plan(Vecf<3>((*wp_list)[i][0],(*wp_list)[i][1],(*wp_list)[i][2]),Vecf<3>((*wp_list)[i+1][0],(*wp_list)[i+1][1],(*wp_list)[i+1][2]),1,false);
            tmp_path = jps_pathfinder.getSamplePath();
            path_0.pop_back();
            path_0.insert(path_0.end(),tmp_path.begin(),tmp_path.end());
        }
        vec_Vec3f end_path;
        jps_pathfinder.plan(Vecf<3>((*wp_list)[wp_list->size()-1][0],(*wp_list)[wp_list->size()-1][1],(*wp_list)[wp_list->size()-1][2]),
        Vecf<3>(end_pt[0],end_pt[1],end_pt[2]),1,false);
        end_path = jps_pathfinder.getSamplePath();
        path_0.pop_back();
        path_0.insert(path_0.end(),end_path.begin(),end_path.end());
        path = path_0;
    }
    EllipsoidDecomp3D decomp_util;
    decomp_util.set_obs(*obs_pointer);
    decomp_util.set_local_bbox(Eigen::Vector3d(config.polyhedronBox(0),
                                               config.polyhedronBox(1),
                                               config.polyhedronBox(2)));
    vec_E<Polyhedron3D> decompPolys;
    vec_E<Ellipsoid3D> ellips; 
    for(int i = 0;i<path.size()-1;){
        //find the farest unblocked point
        int k;
        for(k = i+1;k<path.size();k++){
            if(map_util->isBlocked(path[i],path[k])||((path[i]-path[k]).norm()>=4.0)){
                k--;
                break;
            }
        }
        if(k<i+1){
            k = i+1;
        }
        if(k>=path.size()) k = path.size()-1;
        vec_Vec3f line;
        line.push_back(path[i]);
        line.push_back(path[k]);
        decomp_util.dilate(line);
        Polyhedron3D poly = decomp_util.get_polyhedrons()[0];
        Ellipsoid3D ellip = decomp_util.get_ellipsoids()[0];
        decompPolys.push_back(poly);
        ellips.push_back(ellip);
        //find the nearest one to the boundry of poly.
        int j;
        for(j=k;j<path.size();j++){
            Vec3f pt;
            pt[0] = path[j][0];
            pt[1] = path[j][1];
            pt[2] = path[j][2];
            if(!poly.inside(pt)){
                break;
            }
        }
        j--;
        if(j>=path.size()-1){
            break;
        }
        int wp;
        wp = round((1*i+4*j)/5);
        i = wp;
    }
    std::cout << "Number of polyhedra from map: " << decompPolys.size() << std::endl;
    for (size_t i = 0; i < decompPolys.size(); i++)
    {
        decompPolys[i].add(Hyperplane3D(Eigen::Vector3d(0.0, 0.0, config.mapHeight),
                                          Eigen::Vector3d(0.0, 0.0, 1.0)));
        decompPolys[i].add(Hyperplane3D(Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(0.0, 0.0, -1.0)));
    }
    visualization.visualizePolyH(decompPolys);   
    vector<MatrixXd> hPolys;
    Eigen::MatrixXd current_poly;
    for (uint i = 0; i < decompPolys.size(); i++)
    {
        vec_E<Hyperplane3D> current_hyperplanes = decompPolys[i].hyperplanes();
        current_poly.resize(6, current_hyperplanes.size());
        for (uint j = 0; j < current_hyperplanes.size(); j++)
        {
            current_poly.col(j) << current_hyperplanes[j].n_, current_hyperplanes[j].p_;
            //outside
        }
        hPolys.push_back(current_poly);
    }
    std::cout << "Number of polyhedra from map: " << hPolys.size() << std::endl;
    std::cout<<"-----------------------------------------------\n";
    for(int i = 0;i<hPolys.size();i++){
        std::cout<<"poly id: "<<i<<endl;
        std::cout<<hPolys[i]<<std::endl;
    }
    std::cout<<"--------------------------------------------\n";
    ROS_INFO("Begin to optimize the traj~");
    std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();
    SE3GCOPTER nonlinOpt;
    Trajectory traj;
    if (!nonlinOpt.setup(config.rho, config.totalT, iniState, finState, hPolys, INFINITY,
                            config.qdIntervals, config.horizHalfLen, config.vertHalfLen,
                            config.safeMargin, config.velMax, config.thrustAccMin, config.thrustAccMax,
                            config.bodyRateMax, config.gravAcc, config.penaltyPVTB, config.useC2Diffeo))
    {
        return;
    }
    double finalObj = nonlinOpt.optimize(traj, config.optRelTol);
    std::chrono::high_resolution_clock::time_point toc = std::chrono::high_resolution_clock::now();
    double compTime = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1.0e-3;
    nonlinOpt.kill_kernel();
    
    printf("finished!!!\n");
    std::cout << "Optimization time usage: " << compTime << " ms" << std::endl;
    std::cout << "Final cost: " << finalObj << std::endl;
    std::cout << "Maximum Vel: " << traj.getMaxVelRate() << std::endl;
    std::cout << "Maximum Acc: " << traj.getMaxAccRate() << std::endl;
    std::cout << "Total Duation: " << traj.getTotalDuration() << std::endl;

    if (traj.getPieceNum() > 0)
    {
        lastIniStamp = ros::Time::now();

        visualization.visualize(traj);
        visualization.visualizeEllipsoid(traj,450);
        // visualization.visualizeQuadrotor(traj, 70);
    }
    quadrotor_msgs::PolynomialTrajectory traj_msg;
    traj_msg = traj2msg(traj);
    _traj_pub.publish(traj_msg);
}

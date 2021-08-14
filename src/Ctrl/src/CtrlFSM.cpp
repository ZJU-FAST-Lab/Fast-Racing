#include "CtrlFSM.h"
#include <uav_utils/converters.h>

using namespace Eigen;
using std::cout;
using std::endl;
using namespace uav_utils;

CtrlFSM::CtrlFSM(Parameter_t& param_, Controller& controller_, HovThrKF& hov_thr_kf_):
	param(param_), controller(controller_), hov_thr_kf(hov_thr_kf_)
{
	state = MANUAL_CTRL;
	hover_pose.setZero();
}

void CtrlFSM::process()
{
	ros::Time now_time = ros::Time::now();
	Controller_Output_t u;
	SO3_Controller_Output_t u_so3;

	// if (!rc_is_received(now_time))
	// {
	// 	ROS_ERROR("RC lost for %3f seconds!!!!", (now_time - rc_data.rcv_stamp).toSec());
	// }

	// // cout << state << " " << rc_data.is_api_mode  << endl;

	// switch (state)
	// {
	// 	case MANUAL_CTRL:
	// 	{
	// 		if ( rc_data.enter_api_mode ) 
	// 		{
	// 			rc_data.enter_api_mode = false;
	// 			if ( odom_is_received(now_time) )
	// 			{
	// 				state = AUTO_HOVER;
	// 				controller.config_gain(param.hover_gain);
	// 				set_hov_with_odom();
	// 				toggle_offboard_mode(true);

	// 				ROS_INFO("\033[32m[Ctrl] Start AUTO_HOVER by Ctrl!\033[32m");
	// 			}
	// 			else
	// 			{
	// 				ROS_ERROR("[Ctrl] Reject AUTO_HOVER. No odom!");
	// 			}
	// 		}

	// 		break;
	// 	}

	// 	case AUTO_HOVER:
	// 	{
	// 		if ( !rc_data.is_api_mode || !odom_is_received(now_time) )
	// 		{
	// 			state = MANUAL_CTRL;
	// 			toggle_offboard_mode(false);

	// 			ROS_WARN("[Ctrl] Return to manual control!");
	// 		}
	// 		else if ( rc_data.is_command_mode && cmd_is_received(now_time) )
	// 		{
	// 			if ( state_data.current_state.mode == "OFFBOARD" )
	// 			{
	// 				state = CMD_CTRL;
	// 				controller.config_gain(param.track_gain);
	// 				process_cmd_control(u, u_so3);
	// 				ROS_INFO("\033[32m[Ctrl] position commands received!\033[32m");
	// 			}
	// 		}
	// 		else
	// 		{
	// 			process_hover_control(u, u_so3);
	// 			if ( rc_data.enter_command_mode )
	// 			{
	// 				rc_data.enter_command_mode = false;
	// 				publish_trigger(odom_data.msg);
	// 			}
	// 		}

	// 		break;
	// 	}

	// 	case CMD_CTRL:
	// 	{
	// 		if ( !rc_data.is_api_mode || !odom_is_received(now_time) )
	// 		{
	// 			state = MANUAL_CTRL;
	// 			toggle_offboard_mode(false);
				
	// 			ROS_WARN("[Ctrl] Return to MANUAL_CTRL!");
	// 		}
	// 		else if ( !cmd_is_received(now_time) )
	// 		{
	// 			state = AUTO_HOVER;
    //         	controller.config_gain(param.hover_gain);
    //         	set_hov_with_odom();
	// 			process_hover_control(u, u_so3);
	// 			ROS_WARN("[Ctrl] Return to AUTO_HOVER!");
	// 		}
	// 		else
	// 		{
	// 			process_cmd_control(u, u_so3);
	// 		}
			
	// 		break;
	// 	}
	
	// 	default:
	// 		break;
	// }


	
	controller.config_gain(param.track_gain);
	process_cmd_control(u, u_so3);
	// align_with_imu(u);
	// u.yaw = cmd_data.yaw;
	controller.publish_ctrl(u, now_time);
	hov_thr_kf.simple_update(u.des_v_real, odom_data.v );
	// This line may not take effect according to param.hov.use_hov_percent_kf
	param.config_full_thrust(hov_thr_kf.get_hov_thr());
}

void CtrlFSM::publish_trigger(const nav_msgs::Odometry& odom_msg)
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;
	
    traj_start_trigger_pub.publish(msg);
}

bool CtrlFSM::rc_is_received(const ros::Time& now_time)
{
	return (now_time - rc_data.rcv_stamp).toSec() < param.msg_timeout.rc;
}

bool CtrlFSM::cmd_is_received(const ros::Time& now_time)
{
	return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
}

bool CtrlFSM::odom_is_received(const ros::Time& now_time)
{
	return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
}

bool CtrlFSM::imu_is_received(const ros::Time& now_time)
{
	return (now_time - imu_data.rcv_stamp).toSec() < param.msg_timeout.imu;
}

double CtrlFSM::get_yaw_from_odom()
{
	return get_yaw_from_quaternion(odom_data.q);
}

void CtrlFSM::process_hover_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
	Desired_State_t des;
	des.p = hover_pose.head<3>();
	des.v = Vector3d::Zero();
	des.yaw = hover_pose(3);
	des.a = Vector3d::Zero();
	des.jerk = Vector3d::Zero();
	controller.update(des, odom_data, u, u_so3);

	//publish_desire(des);
}

void CtrlFSM::process_cmd_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
	Desired_State_t des;
	des.p = cmd_data.p;
	des.v = cmd_data.v;
	des.yaw = cmd_data.yaw;
	des.a = cmd_data.a;
	des.jerk = cmd_data.jerk;
	des.head_rate = cmd_data.head_rate;
	// ROS_INFO_STREAM("desp: "<<des.p<<" desv: "<<des.v<<" desyaw: "<<des.yaw<<" desa "<<des.a);
	// ROS_INFO_STREAM("odomp: "<<odom_data.p<<" odomv: "<<odom_data.v);
	// ROS_INFO_STREAM("qo "<<odom_data.q.w()<<" q1 "<<odom_data.q.x() <<" q2 "<<odom_data.q.y()<<" q3 "<<odom_data.q.z());
	controller.update(des, odom_data, u, u_so3);
	// ROS_INFO_STREAM("pitch: "<<u.pitch<<"roll: "<<u.roll<<" u.yaw: "<<u.yaw);

	//publish_desire(des);	
}

void CtrlFSM::align_with_imu(Controller_Output_t& u)
{
	double imu_yaw = get_yaw_from_quaternion(imu_data.q); 
	double odom_yaw = get_yaw_from_odom();
	double des_yaw = u.yaw;
	// ROS_INFO_STREAM("imu yaw: "<<imu_yaw<<" odom_yaw: "<<odom_yaw);
	u.yaw = yaw_add(yaw_add(des_yaw, -odom_yaw), imu_yaw); 

	//out << "imu_yaw=" << imu_yaw << " odom_yaw=" << odom_yaw << " des_yaw=" << des_yaw << " u.yaw=" << u.yaw << endl;
};

void CtrlFSM::set_hov_with_odom()
{
	hover_pose.head<3>() = odom_data.p;
	hover_pose(3) = get_yaw_from_odom();
}

// void CtrlFSM::publish_desire(const Desired_State_t& des)
// {
// 	geometry_msgs::PoseStamped msg;
// 	msg.header = odom_data.msg.header;

// 	msg.pose.position.x = des.p(0);
// 	msg.pose.position.y = des.p(1);
// 	msg.pose.position.z = des.p(2);

// 	Eigen::Quaterniond q = yaw_to_quaternion(des.yaw);

// 	msg.pose.orientation.w = q.w();
// 	msg.pose.orientation.x = q.x();
// 	msg.pose.orientation.y = q.y();
// 	msg.pose.orientation.z = q.z();

// 	des_pose_pub.publish(msg);
// }

void CtrlFSM::toggle_offboard_mode(bool on_off)
{	
	mavros_msgs::SetMode offb_set_mode;
	ros::Time last_request = ros::Time::now();

	if ( on_off )
	{
		offb_set_mode.request.custom_mode = "OFFBOARD";
		controller.set_FCU_mode.call(offb_set_mode);
		// int count = 0;
		// while(count < 5 && ros::ok())
		// {
		// 	offb_set_mode.request.custom_mode = "OFFBOARD";
		// 	if( state_data.current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
		// 	{
		// 		if( controller.set_FCU_mode.call(offb_set_mode) && offb_set_mode.response.mode_sent)
		// 		{
		// 			ROS_INFO("Offboard enabled");
		// 			return;
		// 		}
		// 		last_request = ros::Time::now();
		// 		ROS_WARN("on Again.");
		// 		count++;
		// 	}
        // 	ros::spinOnce();
		// }
		// ROS_WARN("Toggle OFFBOARD mode on failed.");
	}
	else
	{
		offb_set_mode.request.custom_mode = "ALTCTL";
		controller.set_FCU_mode.call(offb_set_mode);
		// int count = 0;
		// while(count < 5 && ros::ok())
		// {
		// 	if ( state_data.current_state.mode == "OFFBOARD" )
		// 	{
		// 		ROS_INFO("Not in OFFBOARD mode");
		// 		return;
		// 	}
		// 	offb_set_mode.request.custom_mode = "ALTCTL";
		// 	if( state_data.current_state.mode == "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
		// 	{
		// 		if( controller.set_FCU_mode.call(offb_set_mode) && offb_set_mode.response.mode_sent)
		// 		{
		// 			ROS_INFO("Return from OFFBOARD mode");
		// 			return;
		// 		}
		// 		ROS_WARN("off Again.");
		// 		last_request = ros::Time::now();
		// 		count++;
		// 	}
        // 	ros::spinOnce();
		// }
		// ROS_ERROR("Toggle OFFBOARD mode off failed. EMERGENCY!!!!!");
	}
	
}
bool CtrlFSM::px4_init(){
	return (odom_data.odom_init&&imu_data.imu_init&&cmd_data.cmd_init);
}

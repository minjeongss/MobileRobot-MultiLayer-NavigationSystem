/* Copyright 2021 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <xarm_planner/pose_plan.h>
#include <xarm_planner/joint_plan.h>
#include <xarm_planner/exec_plan.h>
#include <xarm_planner/single_straight_plan.h>
#include <stdlib.h>
#include <vector>

static const std::string target_frame = "/link_base";
static const std::string source_frame = "/up";

// *********** configuration:
static ros::ServiceClient exec_client;
static xarm_planner::exec_plan exec_srv;

static ros::ServiceClient line_client;
static xarm_planner::single_straight_plan line_srv;

static ros::ServiceClient pose_client;
static xarm_planner::pose_plan pose_srv;


bool request_plan(ros::ServiceClient& client, xarm_planner::single_straight_plan& srv)
{
	if(client.call(srv))
	{
		return srv.response.success;
	}
	else
	{
		ROS_ERROR("Failed to call service single_straight_plan");
		return false;
	}
}

bool request_pose_plan(ros::ServiceClient& client, xarm_planner::pose_plan& srv)
{
	if(client.call(srv))
	{
		return srv.response.success;
	}
	else
	{
		ROS_ERROR("Failed to call service pose_plan");
		return false;
	}
}

bool request_exec(ros::ServiceClient& client, xarm_planner::exec_plan& srv)
{
	if(client.call(srv))
	{
		return srv.response.success;
	}
	else
	{
		ROS_ERROR("Failed to call service exec_plan");
		return false;
	}
}

int grasp_configuration(ros::NodeHandle& nh)
{
	// make sure xarm_planner services are ready, use "ros::service::waitForService("xarm_straight_plan")" if needed
	exec_client = nh.serviceClient<xarm_planner::exec_plan>("xarm_exec_plan");
	line_client = nh.serviceClient<xarm_planner::single_straight_plan>("xarm_straight_plan");
	pose_client = nh.serviceClient<xarm_planner::pose_plan>("xarm_pose_plan");

	return 0;
}

void go_pose(const geometry_msgs::Pose& target)
{
	pose_srv.request.target = target;
	if(request_pose_plan(pose_client, pose_srv))
	{
		ROS_INFO("Plan SUCCESS! Executing... ");
		exec_srv.request.exec = true;
		if(request_exec(exec_client, exec_srv))
		{
			return;
		}
	}
	ROS_ERROR("Failed to move");
	exit(-1);
}

void go_line(const geometry_msgs::Pose& target)
{
	line_srv.request.target = target;
	if(request_plan(line_client, line_srv))
	{
		ROS_INFO("Plan SUCCESS! Executing... ");
		exec_srv.request.exec = true;
		if(!(request_exec(exec_client, exec_srv)))
		{
			go_pose(target);
		}
	}
	else
	{
		go_pose(target);
	}
}

void go_straight(const geometry_msgs::Pose& target)
{
	line_srv.request.target = target;
	if(request_plan(line_client, line_srv))
	{
		ROS_INFO("Plan SUCCESS! Executing... ");
		exec_srv.request.exec = true;
		if(request_exec(exec_client, exec_srv))
		{
			return;
		}
	}
	ROS_ERROR("Failed to straight move");
}

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "grasp_test_xarm_planner");
	ros::NodeHandle nh;

	// *********** configuration:
	grasp_configuration(nh);

	// *********** go to start pose:
	ROS_INFO("Go to start pose");
	geometry_msgs::Pose start_pose;
	
	start_pose.position.x = 0.0;
	start_pose.position.y = 0.24;
	start_pose.position.z = 0.22;
	start_pose.orientation.x = 0.5;
	start_pose.orientation.y = 0.5;
	start_pose.orientation.z = 0.5;
	start_pose.orientation.w = -0.5;

	go_line(start_pose);

	// *********** set searching poses:
	ROS_INFO("Go to search pose");
	geometry_msgs::Pose search_pose_out;
	geometry_msgs::Pose search_pose_in;

	search_pose_out.position.x = 0.35;
	search_pose_out.position.y = 0.0;
	search_pose_out.position.z = 0.12;
	search_pose_out.orientation.x = 0.7071068;
	search_pose_out.orientation.y = 0.0;
	search_pose_out.orientation.z = 0.7071068;
	search_pose_out.orientation.w = 0.0;

	search_pose_in.position.x = 0.0761529;
	search_pose_in.position.y = 0.00142891;
	search_pose_in.position.z = 0.4;
	search_pose_in.orientation.x = 0.706265;
	search_pose_in.orientation.y = 0.00397285;
	search_pose_in.orientation.z = 0.707904;
	search_pose_in.orientation.w = -0.0068836;

	go_line(search_pose_out);
	ros::Duration(0.3).sleep();

	// *********** Second, wait for the recognition result from continuous_detection:
	ROS_INFO("Search the button");
	tf::TransformListener listener;
	tf::StampedTransform transform;

	try {
		// timeout here is 15 seconds, default object frame name: "a"
	    listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(5.0) );
	    listener.lookupTransform( target_frame, source_frame, ros::Time(0), transform);
	} catch (tf::TransformException ex) {
	    ROS_ERROR("%s",ex.what());
		go_line(start_pose);
	    exit(-1);
	}

	double x = transform.getOrigin().x();
	double y = transform.getOrigin().y();
	double z = transform.getOrigin().z();

	tf::Quaternion q = transform.getRotation();
	double qx = q.getX();
	double qy = q.getY();
	double qz = q.getZ();
	double qw = q.getW();
	ROS_WARN("recognition listener: x: %lf, y: %lf, z: %lf, q: [%lf, %lf, %lf, %lf]", x,y,z, qw, qx, qy, qz);


	// *********** Third, go to 40mm in front of target:
	ROS_INFO("Go to front of the button");
	geometry_msgs::Pose next_target;
	next_target.position.x = x-0.04;
	next_target.position.y = y+0.01;
	next_target.position.z = z-0.005;

	next_target.orientation.x = 0.7071068;
	next_target.orientation.y = 0;
	next_target.orientation.z = 0.7071068;
	next_target.orientation.w = 0;

	go_line(next_target);
	ros::Duration(0.3).sleep();

	// *********** Fourth, click the button:
	//ROS_INFO("Touch the button");
	//next_target.position.x = x;
	//go_straight(next_target);
	//ros::Duration(1.0).sleep();

	ROS_INFO("Press the button");
	next_target.position.x = x+0.008;
	go_straight(next_target);
	ros::Duration(0.2).sleep();
    
	ROS_INFO("Go backward");
	next_target.position.x = x-0.01;
	go_straight(next_target);
	ros::Duration(0.2).sleep();
    
	ROS_INFO("Go backward");
	go_line(search_pose_out);
	ros::Duration(0.2).sleep();

	// *********** Last step, go back to start pose:
	ROS_INFO("Go to start pose again");
	go_line(start_pose);

	return 0;

}

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
static const std::string source_frame = "/a";

// *********** configuration:
static ros::ServiceClient exec_client;
static xarm_planner::exec_plan exec_srv;

static ros::ServiceClient line_client;
static xarm_planner::single_straight_plan line_srv;


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
	line_client = nh.serviceClient<xarm_planner::pose_plan>("xarm_straight_plan");

	return 0;
}

int go_line(const geometry_msgs::Pose& target)
{
	line_srv.request.target = target;
	if(request_plan(line_client, line_srv))
	{
		ROS_INFO("Plan SUCCESS! Executing... ");
		exec_srv.request.exec = true;
		request_exec(exec_client, exec_srv);
		return 0;
	}
	return -1;
}

int main(int argc, char** argv)
{	
	ros::init(argc, argv, "grasp_test_xarm_planner");
	ros::NodeHandle nh;

	// *********** configuration:
	grasp_configuration(nh);


	// *********** Second, wait for the recognition result from find_object_3d:
	tf::TransformListener listener;
	tf::StampedTransform transform;

	try {
		// timeout here is 15 seconds, default object frame name: "object_1"
	    listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(15.0) );
	    listener.lookupTransform( target_frame, source_frame, ros::Time(0), transform);
	} catch (tf::TransformException ex) {
	    ROS_ERROR("%s",ex.what());
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


	return 0;

}

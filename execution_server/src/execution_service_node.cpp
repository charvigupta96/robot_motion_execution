/*
 * Copyright (c) 2017 Felix Burget <burgetf@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: May 20, 2017
 *      Author: Felix Burget <burgetf@informatik.uni-freiburg.de>
 * 	  Filename: execution_service_node.cpp
 */

#include <ros/ros.h>
#include <execution_server/execution_server.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "execution_service_node");
	ros::NodeHandle n;

	execution_server::ExecutionServer execution_server;

	ros::AsyncSpinner spinner(0);
	spinner.start();

	ros::waitForShutdown();

	return 0;
}


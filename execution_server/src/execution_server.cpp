/*
 * Copyright (c) 2017 Felix Burget <burgetf@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: May 20, 2017
 *      Author: Felix Burget <burgetf@informatik.uni-freiburg.de>
 * 	  Filename: execution_server.cpp
 */

#include <execution_server/execution_server.h>

namespace execution_server
{

//Constructor
ExecutionServer::ExecutionServer()
{
	ros::NodeHandle nh("~");


    //+++++++++ Bind Services for Trajectory Execution (RViz) +++++++++

    m_execute_trajectory_rviz = nh.advertiseService("execute_trajectory_rviz", &ExecutionControlCenter::execute_trajectory_rviz, &m_execution_control_center);

    
    //+++++++++ Bind Services for Trajectory Execution (Real Robot) +++++++++

    m_execute_trajectory_robot = nh.advertiseService("execute_trajectory_robot", &ExecutionControlCenter::execute_trajectory_robot, &m_execution_control_center);

}

//Destructor
ExecutionServer::~ExecutionServer()
{
}

} /* namespace planning_server */




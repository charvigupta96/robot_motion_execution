/*
 * Copyright (c) 2017 Felix Burget <burgetf@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: May 20, 2017
 *      Author: Felix Burget <burgetf@informatik.uni-freiburg.de>
 * 	  Filename: execution_control_center.cpp
 */

#include <ros/ros.h>

#include <execution_server/execution_control_center.h>


namespace execution_server
{

//Constructor
ExecutionControlCenter::ExecutionControlCenter(): nh_("~")
{
    //Read planning group name from parameter server
    nh_.param("planning_group", m_planning_group, std::string("omnirob_lbr_sdh"));

    //Trajectory Execution RViz
    m_execution_rviz = boost::shared_ptr<trajectory_execution::MotionCommanderRVIZ>(new trajectory_execution::MotionCommanderRVIZ(m_planning_group));
    
    //Trajectory Execution Real Robot
    m_execution_robot = boost::shared_ptr<trajectory_execution::MotionCommanderRobot>(new trajectory_execution::MotionCommanderRobot(m_planning_group));

    ROS_INFO_STREAM("Trajectory execution active");
    ROS_INFO_STREAM("Planning group for execution set to: " << m_planning_group);

}

//Destructor
ExecutionControlCenter::~ExecutionControlCenter()
{
}


//+++++++++ Planning Execution Service Callbacks (Simulation) +++++++++

bool ExecutionControlCenter::execute_trajectory_rviz(execution_msgs::execute_trajectory_rviz::Request& req,
                                                     execution_msgs::execute_trajectory_rviz::Response& res){

    res.execution_success = m_execution_rviz->execute(req.planner_type,req.run_id);
}


//+++++++++ Planning Execution Service Callbacks (Real Robot) +++++++++

bool ExecutionControlCenter::execute_trajectory_robot(execution_msgs::execute_trajectory_robot::Request& req,
        execution_msgs::execute_trajectory_robot::Response& res){


    res.execution_success = m_execution_robot->execute(req.planner_type,req.run_id);
}


} /* namespace execution_server */




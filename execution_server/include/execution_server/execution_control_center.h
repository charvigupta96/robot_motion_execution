/*
 * Copyright (c) 2017 Felix Burget <burgetf@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: May 20, 2017
 *      Author: Felix Burget <burgetf@informatik.uni-freiburg.de>
 * 	  Filename: execution_control_center.h
 */
 

#ifndef EXECUTION_CONTROL_CENTER_H_
#define EXECUTION_CONTROL_CENTER_H_


#include <ros/ros.h>

//Inclide Messages and Services
#include <execution_msgs/execute_trajectory_rviz.h>
#include <execution_msgs/execute_trajectory_robot.h>

//Include Planners
#include <motion_trajectory_execution/trajectory_execution_rviz.h>
#include <motion_trajectory_execution/trajectory_execution_robot.h>


namespace execution_server
{
//* PlanningControlCenter
/**
 * Class providing service callbacks for joint trajectory execution
 */
class ExecutionControlCenter
{
public:
	ExecutionControlCenter();
	virtual ~ExecutionControlCenter();

    //+++++++++ Planning Execution Service Callbacks (Simulation) +++++++++

    bool execute_trajectory_rviz(execution_msgs::execute_trajectory_rviz::Request& req,
            execution_msgs::execute_trajectory_rviz::Response& res); /**< Service callback to execute joint trajectory in RViz */

    //+++++++++ Planning Execution Service Callbacks (Real Robot) +++++++++

    bool execute_trajectory_robot(execution_msgs::execute_trajectory_robot::Request& req,
            execution_msgs::execute_trajectory_robot::Response& res); /**< Service callback to execute joint trajectory on real robot */



protected:

  
private:

    //Node Handle
    ros::NodeHandle nh_;

    //Planning Group
    std::string m_planning_group;

    //Trajectory execution RViz and Real Robot
    boost::shared_ptr<trajectory_execution::MotionCommanderRVIZ> m_execution_rviz;
    boost::shared_ptr<trajectory_execution::MotionCommanderRobot> m_execution_robot;


};

} /* namespace execution_server */

#endif /* EXECUTION_CONTROL_CENTER_H_ */


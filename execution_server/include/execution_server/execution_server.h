/*
 * Copyright (c) 2017 Felix Burget <burgetf@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: May 20, 2017
 *      Author: Felix Burget <burgetf@informatik.uni-freiburg.de>
 * 	  Filename: execution_server.h
 */

#ifndef EXECUTION_SERVER_H_
#define EXECUTION_SERVER_H_

#include <ros/ros.h>
#include <execution_server/execution_control_center.h>

namespace execution_server
{
//* ExecutionServer
/**
* Class providing services for joint trajectory execution
*/
class ExecutionServer
{
public:
    /*!
    * \brief ExecutionServer Constructor
    */
	ExecutionServer();
    /*!
    * \brief ExecutionServer Destructor
    */
	~ExecutionServer();

protected:
	//World Database Object
    ExecutionControlCenter m_execution_control_center;    /**< Execution control center providing service callbacks */


    //+++++++++ Services for Trajectory Execution (RViz) +++++++++
   
    //Execute Joint Trajectory in RViz
    ros::ServiceServer m_execute_trajectory_rviz; /**< Service to execute joint trajectory in RViz */


    //+++++++++ Services for Trajectory Execution (Real Robot) +++++++++

    //Execute Joint Trajectory on Robot
    ros::ServiceServer m_execute_trajectory_robot; /**< Service to execute joint trajectory in RViz */



private:


	
};

}/* namespace execution_server */
#endif /* EXECUTION_SERVER_H_ */



/*
 * trajectory_execution_rviz.h
 *
 *  Created on: June 21, 2015
 *      Author: Felix Burget
 */


// --- Includes -- 
#include <ros/ros.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <kuka_motion_control/kdl_kuka_model.h>

//Trajectoy Planning (Generating Time-optimal Trajectory from Path)
#include <trajectory_planning/Trajectory.h>
#include <trajectory_planning/Path.h>

//Moveit Trajectory Message
#include <moveit_msgs/DisplayTrajectory.h>

//Planning World Builder
#include <planning_world_builder/planning_world_builder.h>


#ifndef TRAJECTORY_EXECUTION_RVIZ_H
#define TRAJECTORY_EXECUTION_RVIZ_H

// --Namespaces --
using namespace std;


namespace trajectory_execution{


class MotionCommanderRVIZ
{
	public:
    MotionCommanderRVIZ();
    MotionCommanderRVIZ(string planning_group);
    MotionCommanderRVIZ(string planning_group, string planning_scene);
    MotionCommanderRVIZ(string robot_description_name, string robot_ns, string planning_group, string planning_scene);
    ~MotionCommanderRVIZ();

    //Load Joint Trajectory from file and execute it
    virtual void executeJointTrajectory(char *joint_trajectory_file, char* ee_trajectory_file);

    //Execute Joint Trajectory given as input
    void executeJointTrajectory(vector< vector<double> > joint_trajectory, vector< vector<double> > ee_trajectory);



    protected:
    //Load a Joint Trajectory
    void loadJointTrajectory(char* joint_trajectory_file);

    //Load a EE Trajectory
    void loadEETrajectory(char* ee_trajectory_file);

    //Get Joint Trajectory
    vector< vector<double> > getJointTrajectory();

    //Get EE Trajectory
    vector< vector<double> > getEETrajectory();

    //Execute planned trajectory
    virtual void execute_trajectory();
    
    private:

    //ROS Node Handle
    ros::NodeHandle m_nh;

    //Planning Group
    string m_planning_group;

    //Planning frame (from virtual joint in srdf)
    string m_planning_frame;

    //Namespace prefix for robot
    string m_ns_prefix_robot;

    //Planning Scene Name
    string m_planning_scene_name;

    //Generate KDL tree from Kuka urdf
    boost::shared_ptr<kuka_motion_controller::KDLRobotModel> m_KDLManipulatorModel;

    //Joint Names
    vector<string> m_joint_names;

    //Number of joints
    int m_num_joints;
    
    //Visualization of Motion in Planning Scene
    //boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
    //Publisher for planning scene topic
    //ros::Publisher scene_pub_;

    //Joint Trajectory
    vector< vector<double> > m_joint_trajectory;

    //EE Trajectory
    vector< vector<double> > m_ee_trajectory;

    //Trajectory publisher
    ros::Publisher m_pub_traj;

    //Planning World and Environment Size
    boost::shared_ptr<planning_world::PlanningWorldBuilder> m_planning_world;

    //Set the planning scene (given by input)
    void setPlanningScene(string planning_scene);



    
};


}//end of namespace

#endif // TRAJECTORY_EXECUTION_RVIZ_H

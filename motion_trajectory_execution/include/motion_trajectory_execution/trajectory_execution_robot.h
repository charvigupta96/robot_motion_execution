/*
 * trajectory_execution_robot.h
 *
 *  Created on: July 14, 2015
 *      Author: Felix Burget
 */


// --- Includes -- 
#include <ros/ros.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <kuka_motion_control/kdl_kuka_model.h>

#include <motion_trajectory_execution/trajectory_execution_rviz.h>

//Trajectoy Planning (Generating Time-optimal Trajectory from Path)
#include <trajectory_planning/Trajectory.h>
#include <trajectory_planning/Path.h>

//Trajectory Message
#include <control_msgs/FollowJointTrajectoryAction.h>

//Abstract Robot Commander Interface
#include <robot_interface_definition/robot_interface.h>

//Local Planner for adaptive trajectory execution
#include <eband_local_planner/eband_local_planner_ros.h>


#ifndef TRAJECTORY_EXECUTION_ROBOT_H
#define TRAJECTORY_EXECUTION_ROBOT_H

// --Namespaces --
using namespace std;


namespace trajectory_execution{


class MotionCommanderRobot : public trajectory_execution::MotionCommanderRVIZ , public robot_interface_definition::RobotInterface
{
	public:
    MotionCommanderRobot(string planning_group);
    ~MotionCommanderRobot();

    //Execute Trajectory
    bool execute();

	//Load Joint Trajectory from file and execute it
    void executeJointTrajectory(char *joint_trajectory_file, bool adaptive_execution = false);
    
    protected:
    
    //Execute planned trajectory
    void execute_trajectory();

    //Execute planned trajectory for arm
    void execute_arm_trajectory();
    void execute_arm_trajectory_async();

    //Execute planned trajectory for mobile base
    void execute_base_trajectory();
    void execute_adaptive_base_trajectory();

    //Execute planned trajectory for base + arm
    void execute_base_arm_trajectory();

    //Compute error between current robot pose and desired robot pose (in map frame)
    vector<double> compute_base_pose_error(vector<double> base_wp_pose, tf::StampedTransform initial_transform_map_to_base);

    
    private:

    //ROS Node Handle
    ros::NodeHandle m_nh_;
    
    //Planning Group
    string m_planning_group;

    //Namespace prefix for robot
    string m_ns_prefix_robot;
    
    //Number of Joints and their Names
    int m_num_joints;
    vector<string> m_joint_names;
    
    //Generate KDL tree from Kuka urdf
    boost::shared_ptr<kuka_motion_controller::KDLRobotModel> m_KDLManipulatorModel;
    
    //Joint Path from motion planner
    vector< vector<double> > m_joint_path;
    
    //Convert the Motion Plan to a Joint Trajectory
    void convertMotionPlanToTrajectory();
    void printTrajectoryData(double duration, double time_step);

    //Convert vector of double vectors to vector of geometry_msgs::PoseStamped
    void convertVectorToPoses(vector<geometry_msgs::PoseStamped> &poses);

    
    //Resulting Joint Trajectory
    boost::shared_ptr<Trajectory> m_joint_trajectory;

    //Transform Listener (for robot pose)
    tf::TransformListener m_listener;

    // +++++++ Base Controller +++++++
    //Publisher for mobile base velocity commands
    ros::Publisher m_base_vel_pub;
    bool m_adaptive_execution;

    // +++++++ Elastic Band Local Controller +++++++
    boost::shared_ptr<eband_local_planner::EBandPlannerROS> m_eband_planner;

    // +++++++ Arm Controller (async)+++++++

    //Publisher for manipulator joint trajectory
    ros::Publisher m_jointStatePublisher;
    //Subscriber for manipulator joint config
    ros::Subscriber m_jointStateSubscriber;

    //LBR Joint State Subscriber Callback
    void callback_lbr_joint_states(const sensor_msgs::JointState::ConstPtr& msg);
    //Flag indicating that lbr joint state is available
    bool m_lbr_joint_state_received;
    //Current Lbr joint state
    vector<double> m_lbr_joint_state;
    //Map current arm config to trajectory time instance;
    double map_conf_to_time();

   
   
    
};


}//end of namespace

#endif // TRAJECTORY_EXECUTION_ROBOT_H


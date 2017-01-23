/*
 * trajectory_execution_vrep.h
 *
 *  Created on: June 21, 2015
 *      Author: Felix Burget
 */


// --- Includes -- 
#include <ros/ros.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <kuka_motion_control/kdl_kuka_model.h>

#include <motion_trajectory_execution/trajectory_execution_rviz.h>

// Used data structures:
#include <motion_trajectory_execution/v_repConst.h>

//#include "vrep_common/ProximitySensorData.h"
#include "vrep_common/VrepInfo.h"
#include "vrep_common/JointSetStateData.h"
#include "vrep_common/simRosSetObjectPose.h"
#include "vrep_common/simRosGetObjectPose.h"
#include "vrep_common/simRosGetJointState.h"
#include "vrep_common/simRosSetRobotPose.h"
#include "vrep_common/simRosStartSimulation.h"
#include "vrep_common/simRosSetObjectIntParameter.h"


// Used API services:
#include "vrep_common/simRosEnablePublisher.h"
#include "vrep_common/simRosEnableSubscriber.h"

//Trajectoy Planning (Generating Time-optimal Trajectory from Path)
#include <trajectory_planning/Trajectory.h>
#include <trajectory_planning/Path.h>

//Moveit Trajectory Message
#include <moveit_msgs/DisplayTrajectory.h>



#ifndef TRAJECTORY_EXECUTION_VREP_H
#define TRAJECTORY_EXECUTION_VREP_H

// --Namespaces --
using namespace std;


namespace trajectory_execution{


class MotionCommanderVREP : public MotionCommanderRVIZ
{
	public:
    MotionCommanderVREP();
    MotionCommanderVREP(string planning_group);
    MotionCommanderVREP(string planning_group, string planning_scene);
    ~MotionCommanderVREP();

    //Load Joint Trajectory from file and execute it
    void executeJointTrajectory(char *joint_trajectory_file);

    
    private:

    //ROS Node Handle
    ros::NodeHandle m_nh_;

    //Planning Group
    string m_planning_group;

    //Namespace prefix for robot
    string m_ns_prefix_robot;

    //Generate KDL tree from Kuka urdf
    boost::shared_ptr<kuka_motion_controller::KDLRobotModel> m_KDLManipulatorModel;
    
    //Visualization of Motion in Planning Scene
    //boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
    //Publisher for planning scene topic
    //ros::Publisher scene_pub_;


    //Convert the Motion Plan to a Joint Trajectory
    void convertMotionPlanToTrajectory();

    //Resulting Joint Trajectory
    boost::shared_ptr<Trajectory> m_trajectory;

    //Set initial Robot Pose (Start Config)
    void setInitialRobotPose(int &start_idx_manip_joints_conf_vector);

    //Set initial Control Modes for Joints
    void setInitialControlModes(int &start_idx_manip_joints_motor_handle_vector);

    //Get Offset between current base pose and desired trajectory base point
    void getTrajectoryOffsetBase(double &delta_x, double &delta_y, double &delta_theta, double &curr_theta_global, double trajectory_time);

    //Get Offset between current manipulator config and desired trajectory point manipulator config
    vector<double> getTrajectoryOffsetManipulator(int start_idx_manip_joints_motor_handle_vector, int start_idx_manip_joints_conf_vector, double trajectory_time);

    //Compute required Base Velocity to reach Trajectory Target Point in local Base Frame
    void computeBaseVelocityLocal(const double delta_x, const double delta_y, const double delta_theta, const double curr_theta_global, double &x_vel_local, double &y_vel_local, double &theta_vel_local);

    // +++++ Robot specific  mapping from local base velocity to wheel velocities ++++

    //OMNIROB BASE: Compute and execute required wheel speeds given the linear and angular base velocity
    void setWheelSpeedsOmnirobBase(const double x_vel_local, const double y_vel_local, const double theta_vel_local);

    //ROBOTINO BASE:Compute and execute required wheel speeds given the linear and angular base velocity
    void setWheelSpeedsRobotinoBase(const double x_vel_local, const double y_vel_local, const double theta_vel_local);

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


    //Compute Joint Velocities for Manipulator given the offset to the current trajectory point
    vector<double> computeJointVelocities(vector<double> delta_joint_configs);

    //Compute and execute required joint velocities
    void setJointSpeedsManipulator(int start_idx_manip, vector<double> joint_velocities);


    //Gains for linear and angular velocity components
    double m_linear_vel_base_gain;  // - for omnirob base translational
    double m_angular_vel_base_gain; // - for omnirob base rotational
    double m_angular_vel_lbr_gain;  // - for lbr manipulator joints

    //Joint Trajectory
    vector< vector<double> > m_joint_pos_trajectory;
    vector< vector<double> > m_joint_vel_trajectory;

    //Base rotation offset between rviz and vrep robot representation
    double m_base_rot_offset_rviz_vrep;

    //Comparison of LBR joint rotation axes between rviz and vrep LBR robot representation
    vector<double> m_dir_rotation_axes_rviz_vrep;

    //Three axis rotation (used to get euler angles from quaternion)
    vector<double> three_axis_rot(double r11, double r12, double r21, double r31, double r32);


    // -------- VREP Stuff

    //Vrep motor handle ID's
    vector<int> m_motor_handles;

    //Subcriber for motor handles
    ros::Subscriber m_subMotorHandles;

    //Control modes for the robot joints
    vector<int> m_joint_control_modes;

    //Motor Handles callback (triggered by vrep_ros_communication node)
    void motorhandlesCallback(const std_msgs::Int32MultiArray::ConstPtr& handles_array);

    //Set the joint control modes
    bool setJointControlModes(vector<int> joint_control_modes);

    //Flag indicating whether motor handles are available
    bool m_motor_handles_available;

    //Callback for VREP info topic
    void infoCallback(const vrep_common::VrepInfo::ConstPtr& info);

    // Global variables (modified by topic subscribers):
    bool m_simulationRunning;
    float m_simulationTime;

    //Subscriber for V-REP's info stream (that stream is the only one enabled by default,
    // and the only one that can run while no simulation is running):
    ros::Subscriber m_subInfo;

    //Publisher for omnirob and robotino wheel speeds / LBR joints motor speeds:
    ros::Publisher m_omnirob_motor_speedPub;
    ros::Publisher m_robotino_motor_speedPub;
    ros::Publisher m_lbr_motor_speedPub;
    ros::Publisher m_schunk_sdh2_motor_speedPub;

    //Functions to close and open the Schunk SDH2 Hand
    float m_gripper_opening_closing_speed;
    void prepareGrasp();
    void closeHand();
    void lockHand();
    void openHand();

    // Service client to get current mobile platform base pose
    ros::ServiceClient client_base_pose;
    // Service client to get current joint positions of manipulator
    ros::ServiceClient client_joint_config;

    //Stop motion of manipulator / base
    bool stopManipulatorMotion(int start_idx_manip);
    bool stopManipulatorJointMotion(int start_idx_manip, int joint_num);
    bool stopBaseMotion();

    
};


}//end of namespace

#endif // TRAJECTORY_EXECUTION_RVIZ_H

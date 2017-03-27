/*
 *run_motion_execution_rviz_multi_robot.cpp
 *
 *  Created on: May 25, 2016
 *      Author: Felix Burget
 */


#include <ros/ros.h>
#include <motion_trajectory_execution/trajectory_execution_rviz.h>


using namespace std;

int main(int argc, char** argv)
{
    //Init Node
    ros::init(argc, argv, "motion_execution_rviz_multi_robot");

    //Node Handle
    ros::NodeHandle nh;

    string PLANNING_SCENE = "three_gates";
    string PLANNING_RUN = "0";

    //Set scene name
    if(argc > 1) {
      stringstream s(argv[1]);
      s >> PLANNING_SCENE;
    }
    //Set planning run number
    if(argc > 2) {
      stringstream s(argv[2]);
      s >> PLANNING_RUN;
    }

    //Set planner name
    string PLANNER_NAME = "bi_informed_rrt_star";

    //Set planning groups
    string planning_group_rob_1 = "omnirob_lbr_sdh";
    string planning_group_rob_2 = "robotino_robot";

    //Get names of robot_description parameters
    string robot_description_first_robot;
    string robot_description_second_robot;
    nh.param("robot_description_first_robot", robot_description_first_robot, std::string("omnirob_robot_description"));
    nh.param("robot_description_second_robot", robot_description_second_robot, std::string("robotino_robot_description"));

    //Get namespaces of robots
    string ns_first_robot;
    string ns_second_robot;
    nh.param("ns_first_robot", ns_first_robot, std::string("omnirob_group/"));
    nh.param("ns_second_robot", ns_second_robot, std::string("robotino_group/"));


    //Read package path for planner from parameter server
    //string planner_package_path;
    //nh.param("omnirob_group/planner_package_path", planner_package_path, std::string("/home/burgetf/catkin_ws/src/manipulator_motion_control/rrt_star_algorithm"));

    //Get package path of "planner_statistics"
    string planner_package_path;
    planner_package_path = ros::package::getPath("planner_statistics");

    //Set path to the file that stores the planned joint trajectory for robot 1
    string folder_path = planner_package_path + "/data/" + PLANNER_NAME + "/" + PLANNING_SCENE + "_scene_" + "joint_trajectory_rob_1_run_"+ PLANNING_RUN +".txt";
    char *file_path_joint_trajectory_rob_1 = new char[folder_path.size() + 1];
    copy(folder_path.begin(), folder_path.end(), file_path_joint_trajectory_rob_1);
    file_path_joint_trajectory_rob_1[folder_path.size()] = '\0'; // don't forget the terminating 0
    //cout<<file_path_joint_trajectory_rob_1<<endl;

    //Set path to the file that stores the planned joint trajectory for robot 2
    folder_path = planner_package_path + "/data/" + PLANNER_NAME + "/" + PLANNING_SCENE + "_scene_" + "joint_trajectory_rob_2_run_"+ PLANNING_RUN +".txt";
    char *file_path_joint_trajectory_rob_2 = new char[folder_path.size() + 1];
    copy(folder_path.begin(), folder_path.end(), file_path_joint_trajectory_rob_2);
    file_path_joint_trajectory_rob_2[folder_path.size()] = '\0'; // don't forget the terminating 0
    //cout<<file_path_joint_trajectory_rob_2<<endl;

    //Set path to the file that stores the planned ee trajectory for robot 1
    folder_path = planner_package_path + "/data/" + PLANNER_NAME + "/" + PLANNING_SCENE + "_scene_" + "ee_trajectory_rob_1_run_"+ PLANNING_RUN +".txt";
    char *file_path_ee_trajectory_rob_1 = new char[folder_path.size() + 1];
    copy(folder_path.begin(), folder_path.end(), file_path_ee_trajectory_rob_1);
    file_path_ee_trajectory_rob_1[folder_path.size()] = '\0'; // don't forget the terminating 0
    //cout<<file_path_ee_trajectory_rob_1<<endl;

    //Set path to the file that stores the planned ee trajectory for robot 2
    folder_path = planner_package_path + "/data/" + PLANNER_NAME + "/" + PLANNING_SCENE + "_scene_" + "ee_trajectory_rob_2_run_"+ PLANNING_RUN +".txt";
    char *file_path_ee_trajectory_rob_2 = new char[folder_path.size() + 1];
    copy(folder_path.begin(), folder_path.end(), file_path_ee_trajectory_rob_2);
    file_path_ee_trajectory_rob_2[folder_path.size()] = '\0'; // don't forget the terminating 0
    //cout<<file_path_ee_trajectory_rob_2<<endl;
    

    //Motion Commander Object
    trajectory_execution::MotionCommanderRVIZ mc_rob_1(robot_description_first_robot, ns_first_robot, planning_group_rob_1, PLANNING_SCENE);
    trajectory_execution::MotionCommanderRVIZ mc_rob_2(robot_description_second_robot, ns_second_robot, planning_group_rob_2, PLANNING_SCENE);

    //Execute joint trajectory from file
    mc_rob_1.executeJointTrajectory(file_path_joint_trajectory_rob_1, file_path_ee_trajectory_rob_1);

    mc_rob_2.executeJointTrajectory(file_path_joint_trajectory_rob_2, file_path_ee_trajectory_rob_2);

    cout<<"Finished trajectory visualization"<<endl;
   	
    ros::shutdown();

    return 0;
}



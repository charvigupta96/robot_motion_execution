#include <motion_trajectory_execution/trajectory_execution_robot.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace trajectory_execution{


//Constructor with planning group
MotionCommanderRobot::MotionCommanderRobot(string planning_group)
{
    //Planning Group
    m_planning_group = planning_group;

    //Get namespaces of robot
    m_nh_.param("ns_prefix_robot", m_ns_prefix_robot, std::string(""));

    //Get name of robot_description parameter
    string robot_description_robot;
    m_nh_.param("robot_description_robot", robot_description_robot, std::string("robot_description"));

    //Create Robot model
    m_KDLRobotModel = boost::shared_ptr<kuka_motion_controller::KDLRobotModel>(new kuka_motion_controller::KDLRobotModel(robot_description_robot, m_ns_prefix_robot + "planning_scene", m_ns_prefix_robot + "endeffector_trajectory", planning_group));

    //Get Joint Names for Robot
    m_joint_names = m_KDLRobotModel->getJointNames();

    //Get Number of Joints and their Names
    m_num_joints = m_KDLRobotModel->getNumJoints();
    m_num_joints_revolute = m_KDLRobotModel->getNumRevoluteJoints();
    m_num_joints_prismatic = m_KDLRobotModel->getNumPrismaticJoints();

    //Clear joint trajectory
    m_joint_path.clear();

    //Base Velocity Publisher
    m_base_vel_pub = m_nh_.advertise<geometry_msgs::Twist>(m_ns_prefix_robot + "omnirob/cmd_vel", 1);

    //Elastic Band Local Controller
  //  m_eband_planner = boost::shared_ptr<eband_local_planner::EBandPlannerROS>(new eband_local_planner::EBandPlannerROS());

    //Flag to select rigid or adaptive base trajectory execution
    m_adaptive_execution = false;


    //Linear and angular velocity gain for base motion
    m_nh_.param("linear_vel_base_gain", m_linear_vel_base_gain, 0.6);
    m_nh_.param("angular_vel_base_gain", m_angular_vel_base_gain, 0.6);

    //Linear and angular velocity limit for base motion
    m_nh_.param("linear_vel_limit", m_linear_vel_limit, 0.4);
    m_nh_.param("angular_vel_limit", m_angular_vel_limit, 0.5);

    //Trajectory sampling step width for base motion
    m_nh_.param("traj_sampling_step", m_traj_sampling_step, 0.1);


}



//Destructor
MotionCommanderRobot::~MotionCommanderRobot()
{
    //Clear joint trajectory
    m_joint_path.clear();
}


void MotionCommanderRobot::reset_execution_data()
{
    //Clear configuration space path  and its associated trajectory
    m_joint_path.clear();
    m_joint_trajectory.reset();

    //Reset data in base class (MotionCommanderRVIZ)
    reset_data();
}


//Convert the Motion Plan to a Joint Trajectory
void MotionCommanderRobot::convertMotionPlanToTrajectory(){

    //------------- Convert Trajectory from planner into Joint Velocity Commands-----------------------

    //Convert Trajectory into waypoints list
    list<Eigen::VectorXd> waypoints;
    for (int wp = 0; wp < m_joint_path.size(); wp++)
    {
        Eigen::VectorXd waypoint(m_joint_path[wp].size());
        for (int j = 0; j < m_joint_path[wp].size(); j++)
        {
            waypoint[j] = m_joint_path[wp][j];

        }
        waypoints.push_back(waypoint);
    }

    //Set Velocity and Acceleration Bounds
    Eigen::VectorXd maxAcceleration;
    Eigen::VectorXd maxVelocity;
    maxAcceleration.resize(m_num_joints);
    maxVelocity.resize(m_num_joints);

    for(int base_joint = 0 ; base_joint < m_num_joints_prismatic; base_joint++)
    {
         maxAcceleration[base_joint] = 0.2;
         maxVelocity[base_joint] = 0.5;
    }

    for(int rot_joint = 0 ; rot_joint < m_num_joints_revolute; rot_joint++)
    {
         maxAcceleration[m_num_joints_prismatic+rot_joint] = 0.2;
         maxVelocity[m_num_joints_prismatic+rot_joint] = 0.5;
    }

//    if (m_planning_group == "kuka_complete_arm")
//    {
//        maxAcceleration.resize(7);
//        maxVelocity.resize(7);
//        maxAcceleration << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
//        maxVelocity << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2;
//    }
//    else if (m_planning_group == "omnirob_base")
//    {
//        maxAcceleration.resize(3);
//        maxVelocity.resize(3);
//        maxAcceleration << 1.0, 1.0, 0.5;
//        maxVelocity << 1.0, 1.0, 1.0;
//    }
//    else if(m_planning_group == "omnirob_lbr_sdh")
//    {
//        maxAcceleration.resize(10);
//        maxVelocity.resize(10);
//        maxAcceleration << 1.0, 1.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 ;
//        maxVelocity << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
//    }
//    else
//        ROS_ERROR("Unknown Planning Group");




    //Compute Trajectory (taking velocity and acceleration upper bounds into account)
    m_joint_trajectory = boost::shared_ptr<Trajectory>(new Trajectory(Path(waypoints, 0.1), maxVelocity, maxAcceleration));

    //Trajectory trajectory(Path(waypoints, 0.1), maxVelocity, maxAcceleration);

    //Write Trajectory Data to File
    m_joint_trajectory->outputPhasePlaneTrajectory();
    //Get duration of trajectory
    double duration = m_joint_trajectory->getDuration();
    if(m_joint_trajectory->isValid())
    {
        cout << "Trajectory duration: " << duration << " s" << endl << endl;
        cout << "Trajectory generation completed successfully." << endl;   
    }
    else {
        cout << "Trajectory generation failed." << endl;
    }
}


//LBR Joint State Subscriber Callback
void MotionCommanderRobot::callback_lbr_joint_states(const sensor_msgs::JointState::ConstPtr& msg){

    //Get current position
    m_lbr_joint_state[0] = msg->position[0];
    m_lbr_joint_state[1] = msg->position[1];
    m_lbr_joint_state[2] = msg->position[2];
    m_lbr_joint_state[3] = msg->position[3];
    m_lbr_joint_state[4] = msg->position[4];
    m_lbr_joint_state[5] = msg->position[5];
    m_lbr_joint_state[6] = msg->position[6];

    //Flag indicating that lbr joint state is available
    m_lbr_joint_state_received = true;
}


//Map current arm config to trajectory time instance;
double  MotionCommanderRobot::map_conf_to_time(){


}


//Execute Trajectory
bool MotionCommanderRobot::execute(const string planner_type, const string run_id){

    //Read package path for planner output
    string trajectory_package_path = ros::package::getPath("planner_statistics");

    //Set path to the file that will store the planned joint trajectory
    string folder_path = trajectory_package_path + "/data/" + planner_type + "/motion_plan_joint_trajectory_run_" + run_id + ".txt";
    char *file_path_joint_trajectory = new char[folder_path.size() + 1];
    copy(folder_path.begin(), folder_path.end(), file_path_joint_trajectory);
    file_path_joint_trajectory[folder_path.size()] = '\0'; // don't forget the terminating 0

    //Show path to joint trajectory file
    cout<<file_path_joint_trajectory<<endl;

    //Execute joint trajectory from file
    executeJointTrajectory(file_path_joint_trajectory);
    //executeJointTrajectory(file_path_joint_trajectory,true); //with elastic band

    cout<<"Finished trajectory execution"<<endl;

    return true;
}

//Load Joint Trajectory from file and execute it
void MotionCommanderRobot::executeJointTrajectory(char *joint_trajectory_file, bool adaptive_execution){
	
	//----- Get Trajectory from planner 

    //Get Joint Trajecotry data
    loadJointTrajectory(joint_trajectory_file);

    //Get the loaded Joint Trajectory
    m_joint_path = getJointTrajectory();

    //Execution rigid or adaptive
    m_adaptive_execution = adaptive_execution;
   

	//----- Execute Joint Trajectory 
    //if (m_planning_group == "kuka_complete_arm")
    //Trajectory is for a robot arm
    if (m_num_joints_prismatic == 0 && m_num_joints_revolute > 1)
    {
        //Convert Motion Plan to Joint Trajectory
        convertMotionPlanToTrajectory();

        //Execute trajectory for arm
        execute_arm_trajectory();
    }
    //else if (m_planning_group == "omnirob_base")
    //Trajectory is for a robot base
    else if(m_num_joints_prismatic == 2 && m_num_joints_revolute == 1)
    {
        if(!m_adaptive_execution){

            //Convert Motion Plan to Joint Trajectory
            convertMotionPlanToTrajectory();

            //Execute trajectory for base
            execute_base_trajectory();

            //Trajectory tracking control using curvature of trajectory -> Not evaluated yet
            //execute_base_trajectory_angle_control();
        }
        else{

            //Adaptive base trajectory execution using elastic bands local planner
            execute_adaptive_base_trajectory();
        }
    }
    //else if(m_planning_group == "omnirob_lbr_sdh")
    //Trajectory is for a robot arm + base
    else if(m_num_joints_prismatic == 2 && m_num_joints_revolute > 1)
    {
        //Convert Motion Plan to Joint Trajectory
        convertMotionPlanToTrajectory();

        //Execute trajectory for base + arm
        execute_base_arm_trajectory();
    }
    else
        ROS_ERROR("Planning Group is not found");



    //Reset trajectory data for current class instance
    reset_execution_data();
}



//Execute planned trajectory for arm
void MotionCommanderRobot::execute_arm_trajectory(){

    // Create the action client
    // -> action client constructor also takes two arguments, the server name to connect to and a boolean option to automatically spin a thread.
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> fjt(m_ns_prefix_robot + "follow_joint_trajectory", true);

    // Wait for the action server to start
    ROS_INFO("'execute_trajectory': Waiting for action server to start.");
    fjt.waitForServer(); //will wait for infinite time

    //Follow Joint Trajectory Action Goal
    control_msgs::FollowJointTrajectoryGoal follow_joint_traj_goal;

    //Joint Trajectory Message
    trajectory_msgs::JointTrajectory joint_trajectory_msg;


    //Generate Joint Trajectory message if trajectory generation succeeded
    if(m_joint_trajectory->isValid())
    {
        ROS_INFO("Generated arm trajectory is valid");

        //Get duration of trajectory
        double duration = m_joint_trajectory->getDuration();

        //Number of configs along trajectory depends on trajectory sampling step width
        int num_configs = floor(duration/m_traj_sampling_step) + 1;

        //Set size of arrays storing the joint names and trajectory points
        joint_trajectory_msg.joint_names.resize(m_num_joints);
        joint_trajectory_msg.points.resize(num_configs);

        //Set Frame ID in header
        joint_trajectory_msg.header.frame_id = m_ns_prefix_robot + "lbr_0_link";

        //Set joint names
        for (int i = 0 ; i < m_num_joints; i++)
        {
            //Set Joint Name
            joint_trajectory_msg.joint_names[i] = m_joint_names[i];
        }

        //Take samples from the generated trajectory
        int curr_point_idx = 0; //Current point index
        for(double t = 0.0; t < duration; t += m_traj_sampling_step)
        {
            //Set size of positions array
            joint_trajectory_msg.points[curr_point_idx].positions.resize(m_num_joints);
            joint_trajectory_msg.points[curr_point_idx].velocities.resize(m_num_joints);
            //joint_trajectory_msg.points[curr_point_idx].accelerations.resize(m_num_joints);

            for (int j = 0 ; j < m_num_joints; j++)
            {
              //Set joint position
              joint_trajectory_msg.points[curr_point_idx].positions[j] = m_joint_trajectory->getPosition(t)[j] ;
              //Set joint velocity
              //joint_trajectory_msg.points[curr_point_idx].velocities[j] = m_joint_trajectory->getVelocity(t)[j];
              //Set joint acceleration
              //joint_trajectory_msg.points[curr_point_idx].accelerations[j] = m_joint_trajectory->getAcceleration(t)[j];
              //Set time from start
              joint_trajectory_msg.points[curr_point_idx].time_from_start = ros::Duration(t);
            }

            //Set index to next trajectory point
            curr_point_idx++;
        }


        // Set the joint trajectory to follow
        follow_joint_traj_goal.trajectory = joint_trajectory_msg;

        // -- TODO: Option to be implemented

        // Tolerances applied to the joints as the trajectory is executed.  If violated, the goal aborts with error_code set to PATH_TOLERANCE_VIOLATED.
        //control_msgs::JointTolerance joint_tolerance;
        //follow_joint_traj_goal.path_tolerance = joint_tolerance;

        // If the joints are not within goal_tolerance after "trajectory finish time" + goal_time_tolerance, the goal aborts with error_code set to GOAL_TOLERANCE_VIOLATED
        //follow_joint_traj_goal.goal_tolerance = ;
        //follow_joint_traj_goal.goal_time_tolerance = ;

        // ----------------------------------

        //Call Action
        fjt.sendGoal(follow_joint_traj_goal);


    }//end if generated trajectory from path is valid
    else{
        ROS_INFO("Generated arm trajectory is invalid");
    }

}


void MotionCommanderRobot::execute_arm_trajectory_async(){

    //---------------- TRAJECTORY DATA -----------------

    //Get duration of trajectory
    double duration = m_joint_trajectory->getDuration();

    //Number of configs along trajectory depends on trajectory sampling step width
    int num_configs = floor(duration/m_traj_sampling_step) + 1;

    //---------------- SETUP LBR ARM -----------------

    //Publisher for arm trajectory
    m_jointStatePublisher = m_nh_.advertise<trajectory_msgs::JointTrajectory>(m_ns_prefix_robot + "omnirob/cmd_real_time_trajectory", 1);

    //Subscriber for current arm config
    m_jointStateSubscriber = m_nh_.subscribe(m_ns_prefix_robot + "joint_states", 1, &MotionCommanderRobot::callback_lbr_joint_states, this);


    //Joint Trajectory Message
    trajectory_msgs::JointTrajectory joint_trajectory_msg;

    //Trajectory Waypoints Preview
    int num_wp_preview = 10;

    //Start index of LBR Joints
    int lbr_joints_start_idx = 0;

    //Set size of arrays storing the joint names and trajectory points for the LBR arm
    joint_trajectory_msg.joint_names.resize(m_num_joints-lbr_joints_start_idx);

    //Set Frame ID in header
    joint_trajectory_msg.header.frame_id = m_ns_prefix_robot + "lbr_0_link";

    //Set joint names
    for (int i = lbr_joints_start_idx ; i < m_num_joints; i++)
    {
        //Set Joint Name
        joint_trajectory_msg.joint_names[i-lbr_joints_start_idx] = m_joint_names[i];
    }



    //---------------- EXECUTION -----------------

    //Keep the execution running until last trajectory section is published
    bool run_execution = true;

    //Generate Joint Trajectory message if trajectory generation succeeded
    if(m_joint_trajectory->isValid())
    {
        while(run_execution)
        {
            if(!m_lbr_joint_state_received)
            {
                ROS_INFO("Current LBR Joint State not available yet");

                ros::spinOnce();

            }
            else
            {

                //Find trajectory time instance for current arm config
                double t = map_conf_to_time(); //find time instance for config m_lbr_joint_state

                //End time instance of preview
                double t_preview_end = t + num_wp_preview*m_traj_sampling_step;

                //To avoid exeeding the trajectory duration
                if(duration <= t_preview_end)
                {
                    cout<<"Proceeding to last trajectory"<<endl;

                    //Find number of samples from current "t" to the end of the trajectory "durtion"
                    int num_samples = 0;
                    for(double traj_t = t; traj_t < duration+m_traj_sampling_step; traj_t += m_traj_sampling_step)
                    {
                        num_samples++;
                    }

                    //Consider "num_samples" waypoints for execution
                    joint_trajectory_msg.points.resize(num_samples);

                    for (int wp = 0 ; wp < num_samples; wp++)
                    {
                        //Set size of joint positions array
                        joint_trajectory_msg.points[wp].positions.resize(m_num_joints-lbr_joints_start_idx);
                        joint_trajectory_msg.points[wp].velocities.resize(m_num_joints-lbr_joints_start_idx);
                    }

                    //Take samples from current "t" to the end of the trajectory "durtion"
                    int idx_curr_wp = 0;
                    for(double traj_t = t; traj_t < duration+m_traj_sampling_step; traj_t += m_traj_sampling_step)
                    {
                        //Get lbr joint config of waypoints
                        for (int j = lbr_joints_start_idx ; j < m_num_joints; j++)
                        {
                          //Set joint position
                          joint_trajectory_msg.points[idx_curr_wp].positions[j-lbr_joints_start_idx] = m_joint_trajectory->getPosition(traj_t)[j] ;
                          //Set joint velocity
                          //joint_trajectory_msg.points[curr_point_idx].velocities[j] = m_joint_trajectory->getVelocity(t)[j];
                          //Set time from start
                          joint_trajectory_msg.points[idx_curr_wp].time_from_start = ros::Duration(traj_t);
                        }

                        idx_curr_wp++;
                    }

                    //Stop execution after last iteration
                    run_execution = false;

                }
                else
                {
                    //Consider "num_wp_preview" waypoints for execution
                    joint_trajectory_msg.points.resize(num_wp_preview);

                    for (int wp = 0 ; wp < num_wp_preview; wp++)
                    {
                        //Set size of joint positions array
                        joint_trajectory_msg.points[wp].positions.resize(m_num_joints-lbr_joints_start_idx);
                        joint_trajectory_msg.points[wp].velocities.resize(m_num_joints-lbr_joints_start_idx);
                    }

                    //+++++ Waypoint processing for LBR Arm +++++

                    //Get waypoints
                    int idx_curr_wp = 0;
                    for (int wp = t ; wp < t_preview_end; wp+= m_traj_sampling_step)
                    {
                        //Get lbr joint config of waypoints
                        for (int j = lbr_joints_start_idx ; j < m_num_joints; j++)
                        {
                          //Set joint position
                          joint_trajectory_msg.points[idx_curr_wp].positions[j-lbr_joints_start_idx] = m_joint_trajectory->getPosition(wp)[j] ;
                          //Set joint velocity
                          //joint_trajectory_msg.points[curr_point_idx].velocities[j] = m_joint_trajectory->getVelocity(t)[j];
                          //Set time from start
                          joint_trajectory_msg.points[idx_curr_wp].time_from_start = ros::Duration(wp);
                        }

                        idx_curr_wp++;
                    }

                }

                //Publish preview trajectory
                m_jointStatePublisher.publish(joint_trajectory_msg);

                //Reset flag for current arm config callback
                m_lbr_joint_state_received = false;
            }

        }

        //Trajectory executed
        ROS_INFO("Trajectory executed successfully!!!");


    }//end if generated trajectory from path is valid
    else{
        ROS_ERROR("Generated Trajectory is invalid!!!");
    }

}




//Execute planned trajectory for mobile base
void MotionCommanderRobot::execute_base_trajectory(){


	//Error treshold triggering motion towards next waypoint (used for mobile base trajectory execution)
	m_nh_.param("error_thr_trans", m_error_thr_trans, 0.1);
	m_nh_.param("error_thr_rot", m_error_thr_rot, 0.1);

    //---------------- SETUP -----------------
    //Velocity commanded to the base
    geometry_msgs::Twist base_vel;

    //Print the Trajectory Data
    //printTrajectoryData(duration,m_traj_sampling_step);

    //Get initial robot pose (in map frame)
    tf::StampedTransform start_transform_map_to_base;
    try {
        m_listener.waitForTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), ros::Duration(10.0) );
        m_listener.lookupTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), start_transform_map_to_base);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    //Current waypoint pose along the trajectory
    vector<double> base_wp_pose(3);

    //Current base pose error w.r.t current waypoint
    vector<double> base_pose_error(3);

    //---------------- EXECUTION -----------------

    //Generate Joint Trajectory message if trajectory generation succeeded
    if(m_joint_trajectory->isValid())
    {
        //Get duration of trajectory
        double duration = m_joint_trajectory->getDuration();

        //Number of configs along trajectory depends on trajectory sampling step width
        int num_configs = floor(duration/m_traj_sampling_step) + 1;

        //Take samples from the generated trajectory
        for(double t = 0.0; t < duration+m_traj_sampling_step; t += m_traj_sampling_step)
        {
            //To avoid exeeding the trajectory duration
            if(duration <= t)
            {
                t = duration;

                //Set waypoint distance threshold lower for last trajectory point
                //-> in order to reach final base pose accurately
                m_error_thr_trans = 0.01;
                m_error_thr_rot = 0.01;

                cout<<"Proceeding to last trajectory point"<<endl;
            }
            else
            {
                //cout<<"Proceeding to next trajectory point at time: "<<t<<endl;
            }

            //Get pose of waypoint
            base_wp_pose[0] = m_joint_trajectory->getPosition(t)[0]; //X pos
            base_wp_pose[1] = m_joint_trajectory->getPosition(t)[1]; //Y pos
            base_wp_pose[2] = m_joint_trajectory->getPosition(t)[2]; //Theta orientation       

            //Init deviation of current base pose from waypoint
            base_pose_error = compute_base_pose_error(base_wp_pose,start_transform_map_to_base);

            //cout<<"x err: "<<base_pose_error[0]<<endl;
            //cout<<"y err: "<<base_pose_error[1]<<endl;
            //cout<<"theta err: "<<base_pose_error[2]<<endl;

            while(m_error_thr_trans < fabs(base_pose_error[0]) || m_error_thr_trans < fabs(base_pose_error[1]) || m_error_thr_rot < fabs(base_pose_error[2]))
            {

                //Set base translational velocity (in X and Y direction)
                base_vel.linear.x = m_linear_vel_base_gain * base_pose_error[0];
                base_vel.linear.y = m_linear_vel_base_gain * base_pose_error[1];
                base_vel.linear.z = 0.0;

                //Set base rotational velocity
                base_vel.angular.x = 0.0;
                base_vel.angular.y = 0.0;
                base_vel.angular.z = m_angular_vel_base_gain * base_pose_error[2];

                //cout<<"x vel: "<<m_linear_vel_base_gain * base_pose_error[0]<<endl;
                //cout<<"y vel: "<<m_linear_vel_base_gain * base_pose_error[1]<<endl;
                //cout<<"theta vel: "<<m_angular_vel_base_gain * base_pose_error[2]<<endl;


                //Check for velocity limits
                if(fabs(base_vel.linear.x) <= m_linear_vel_limit && fabs(base_vel.linear.y) <= m_linear_vel_limit && fabs(base_vel.angular.z) <= m_angular_vel_limit)
                {
                  //Send velocity command to platform
                  m_base_vel_pub.publish(base_vel);

                  //ROS_INFO("Send velocities to base!!!");
                }
                else
                {
                    //Scale velocities to obey the joint speed limits
                    double speed_scaling = 1.0;
                    //Compute speed scaling factor
                    if(m_linear_vel_limit < fabs(base_vel.linear.x)){
                        speed_scaling = m_linear_vel_limit / fabs(base_vel.linear.x);
                    } else if (m_linear_vel_limit < fabs(base_vel.linear.y)){
                        speed_scaling = m_linear_vel_limit / fabs(base_vel.linear.y);
                    } else if(m_angular_vel_limit < fabs(base_vel.angular.z)){
                        speed_scaling = m_angular_vel_limit / fabs(base_vel.angular.z);
                    }

                    //Scale velocities proportionally
                    base_vel.linear.x *= speed_scaling;
                    base_vel.linear.y *= speed_scaling;
                    base_vel.angular.z *= speed_scaling;

                    ROS_INFO("Base velocities scaled to obey defined speed limits!!!");
                }

                //Update error between current robot pose and desired pose (given by waypoint)
                base_pose_error = compute_base_pose_error(base_wp_pose,start_transform_map_to_base);

            } //End of approaching the current waypoint

        }//end of iteration through trajectory

        //Trajectory executed
        ROS_INFO("Trajectory executed successfully!!!");


    }//end if generated trajectory from path is valid
    else{
        ROS_ERROR("Generated Trajectory is invalid!!!");
    }
}


//Execute planned trajectory for mobile base and select target points according to curvature of trajectory
void MotionCommanderRobot::execute_base_trajectory_angle_control(){

	//Error treshold triggering motion towards next waypoint (used for mobile base trajectory execution)
	m_nh_.param("error_thr_trans", m_error_thr_trans, 0.1);
	m_nh_.param("error_thr_rot", m_error_thr_rot, 0.1);

    //---------------- SETUP -----------------
    //Velocity commanded to the base
    geometry_msgs::Twist base_vel;

    //Print the Trajectory Data
    //printTrajectoryData(duration,m_traj_sampling_step);

    //Get initial robot pose (in map frame)
    tf::StampedTransform start_transform_map_to_base;
    try {
        m_listener.waitForTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), ros::Duration(10.0) );
        m_listener.lookupTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), start_transform_map_to_base);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    //Current and next waypoint pose along the trajectory
    vector<double> base_wp_pose(3);
    vector<double> base_wp_pose_next(3);
    //Waypoint that travels along trajectory until max_via_wp_angle has been reached
    vector<double> base_wp_pose_explore(3);
    //Waypoint selected for execution
    vector<double> base_wp_pose_execute(3);

    //Current base pose error w.r.t current waypoint
    vector<double> base_pose_error(3);

    //Max. angle between line connecting current and next base position and current select trajectory target point
    double max_via_wp_angle = 45.0; //in degrees
    //Init angle between line connecting current and next base position and current select trajectory target point
    double curr_via_wp_angle = 0.0;

    //---------------- EXECUTION -----------------

    //Generate Joint Trajectory message if trajectory generation succeeded
    if(m_joint_trajectory->isValid())
    {
        //Get duration of trajectory
        double duration = m_joint_trajectory->getDuration();

        //Set initial trajectory time instance
        double t = 0.0;

        //Take samples from the generated trajectory
        while (t < duration+m_traj_sampling_step)
        {
            //To avoid exeeding the trajectory duration
            //if(duration <= t || duration <= (t+m_traj_sampling_step))
            if(duration <= (t+m_traj_sampling_step))
            {
                //Set "t" to last time instance of trajectory
                t = duration;

                //Set waypoint distance threshold lower for last trajectory point
                //-> in order to reach final base pose accurately
                m_error_thr_trans = 0.01;
                m_error_thr_rot = 0.01;
            }
            else
            {
                //Get pose of waypoint at t
                base_wp_pose[0] = m_joint_trajectory->getPosition(t)[0]; //X pos
                base_wp_pose[1] = m_joint_trajectory->getPosition(t)[1]; //Y pos
                base_wp_pose[2] = m_joint_trajectory->getPosition(t)[2]; //Theta orientation

                //Get pose of waypoint at t+1
                base_wp_pose_next[0] = m_joint_trajectory->getPosition(t+m_traj_sampling_step)[0]; //X pos
                base_wp_pose_next[1] = m_joint_trajectory->getPosition(t+m_traj_sampling_step)[1]; //Y pos
                base_wp_pose_next[2] = m_joint_trajectory->getPosition(t+m_traj_sampling_step)[2]; //Theta orientation

                //TODO: AVOID THAT "t+m_traj_sampling_step" becomes larger than "duration"

                //Direction of vector connecting the current waypoint and the next waypoint along the trajectory
                vector<double> base_wp_to_next_wp_dir(2);
                base_wp_to_next_wp_dir[0] = base_wp_pose_next[0]-base_wp_pose[0]; //x-dir
                base_wp_to_next_wp_dir[1] = base_wp_pose_next[1]-base_wp_pose[1]; //y-dir

                //Start point for angle computation
                double time_angle_exploration = t+m_traj_sampling_step;
                //Reset angle between line connecting current and next base position and current select trajectory target point
                curr_via_wp_angle = 0.0;
                //Direction of vector connecting the current waypoint and the explored waypoint along the trajectory
                vector<double> base_wp_to_explored_wp_dir(2);

                //Iterating along wp of the trajectory
                while (fabs(curr_via_wp_angle) < max_via_wp_angle)
                {
                    //Save time instance of last trajectory point that has found to have an angle w.r.t the tangent line lower than
                    //the threshold "max_via_wp_angle"
                    t = time_angle_exploration;

                    //Time instance of next waypoint
                    time_angle_exploration += m_traj_sampling_step;

                    if(duration <= time_angle_exploration)
                    {
                        //Set "t" to last time instance of trajectory
                        t = duration;

                        //Set waypoint distance threshold lower for last trajectory point
                        //-> in order to reach final base pose accurately
                        m_error_thr_trans = 0.01;
                        m_error_thr_rot = 0.01;

                        break;
                    }
                    else
                    {
                        //Get pose of waypoint at time "time_angle_exploration"
                        base_wp_pose_explore[0] = m_joint_trajectory->getPosition(time_angle_exploration)[0]; //X pos
                        base_wp_pose_explore[1] = m_joint_trajectory->getPosition(time_angle_exploration)[1]; //Y pos
                        base_wp_pose_explore[2] = m_joint_trajectory->getPosition(time_angle_exploration)[2]; //Theta orientation

                        //Compute direction of vector connecting the current base pose and the explored base pose along the trajectory
                        base_wp_to_explored_wp_dir[0] = base_wp_pose_explore[0]-base_wp_pose[0]; //x-dir
                        base_wp_to_explored_wp_dir[1] = base_wp_pose_explore[1]-base_wp_pose[1]; //y-dir

                        //Compute angle between the two vectors
                        // -> Vector 1 = base_wp_to_next_wp_dir
                        // -> Vector 2 = base_wp_to_explored_wp_dir
                        double dot_prod = base_wp_to_next_wp_dir[0] * base_wp_to_explored_wp_dir[0] +  base_wp_to_next_wp_dir[1] * base_wp_to_explored_wp_dir[1];      // dot product
                        double det = base_wp_to_next_wp_dir[0] * base_wp_to_explored_wp_dir[1] - base_wp_to_explored_wp_dir[0] * base_wp_to_next_wp_dir[1];      // determinant
                        curr_via_wp_angle = atan2(det, dot_prod);  // atan2(y, x) or atan2(sin, cos)
                        curr_via_wp_angle = (curr_via_wp_angle/M_PI)*180.0;
                    }

                }

            }

            //Select waypoint for execution
            base_wp_pose_execute[0] = m_joint_trajectory->getPosition(t)[0]; //X pos
            base_wp_pose_execute[1] = m_joint_trajectory->getPosition(t)[1]; //Y pos
            base_wp_pose_execute[2] = m_joint_trajectory->getPosition(t)[2]; //Theta orientation

            if(t == duration)
            {
                //Let "t" become (duration+m_traj_sampling_step) to exit trajectory tracking loop after last point has been reached
                t += m_traj_sampling_step;

                cout<<"Proceeding to last trajectory point"<<endl;
            }
            else
            {
                //"t" already set to last time instance of the trajectory "time_angle_exploration" with an angle lower than "max_via_wp_angle"
                cout<<"Proceeding to next trajectory point at time: "<<t<<endl;
            }


            //Init deviation of current base pose from waypoint
            base_pose_error = compute_base_pose_error(base_wp_pose_execute,start_transform_map_to_base);

            cout<<"x err: "<<base_pose_error[0]<<endl;
            cout<<"y err: "<<base_pose_error[1]<<endl;
            cout<<"theta err: "<<base_pose_error[2]<<endl;

            while(m_error_thr_trans < fabs(base_pose_error[0]) || m_error_thr_trans < fabs(base_pose_error[1]) || m_error_thr_rot < fabs(base_pose_error[2]))
            {

                //Set base translational velocity (in X and Y direction)
                base_vel.linear.x = m_linear_vel_base_gain * base_pose_error[0];
                base_vel.linear.y = m_linear_vel_base_gain * base_pose_error[1];
                base_vel.linear.z = 0.0;

                //Set base rotational velocity (base can only rotate around z-axis)
                base_vel.angular.x = 0.0;
                base_vel.angular.y = 0.0;
                base_vel.angular.z = m_angular_vel_base_gain * base_pose_error[2];

                //cout<<"x vel: "<<m_linear_vel_base_gain * base_pose_error[0]<<endl;
                //cout<<"y vel: "<<m_linear_vel_base_gain * base_pose_error[1]<<endl;
                //cout<<"theta vel: "<<m_angular_vel_base_gain * base_pose_error[2]<<endl;


                //Check for velocity limits
                if(fabs(base_vel.linear.x) <= m_linear_vel_limit && fabs(base_vel.linear.y) <= m_linear_vel_limit && fabs(base_vel.angular.z) <= m_angular_vel_limit)
                {
                  //Send velocity command to platform
                  m_base_vel_pub.publish(base_vel);

                  ROS_INFO("Send velocities to base!!!");
                }
                else
                {
                    //Scale velocities to obey the joint speed limits
                    double speed_scaling = 1.0;
                    //Compute speed scaling factor
                    if(m_linear_vel_limit < fabs(base_vel.linear.x)){
                        speed_scaling = m_linear_vel_limit / fabs(base_vel.linear.x);
                    } else if (m_linear_vel_limit < fabs(base_vel.linear.y)){
                        speed_scaling = m_linear_vel_limit / fabs(base_vel.linear.y);
                    } else if(m_angular_vel_limit < fabs(base_vel.angular.z)){
                        speed_scaling = m_angular_vel_limit / fabs(base_vel.angular.z);
                    }

                    //Scale velocities proportionally
                    base_vel.linear.x *= speed_scaling;
                    base_vel.linear.y *= speed_scaling;
                    base_vel.angular.z *= speed_scaling;

                    ROS_INFO("Base velocities scaled to obey defined speed limits!!!");
                }

                //Update error between current robot pose and desired pose (given by waypoint)
                base_pose_error = compute_base_pose_error(base_wp_pose_execute,start_transform_map_to_base);

            } //End of approaching the current waypoint




        }//end of iteration through trajectory

        //Trajectory executed
        ROS_INFO("Trajectory executed successfully!!!");


    }//end if generated trajectory from path is valid
    else{
        ROS_ERROR("Generated Trajectory is invalid!!!");
    }
}



//Execute planned trajectory for mobile base using the elastic bands local planner
void MotionCommanderRobot::execute_adaptive_base_trajectory(){
/*
    //Create costmap_2d
    // Details: A ROS wrapper for a 2D Costmap. Handles subscribing to topics that provide observations about
    //          obstacles in either the form of PointCloud or LaserScan messages.
    costmap_2d::Costmap2DROS costmap_2d_ros("neurobots_demo",m_listener);

    //Initialize eband local planner
    m_eband_planner->initialize("neurobots_demo",&m_listener,&costmap_2d_ros);

    //Convert vector of double vectors to vector of geometry_msgs::PoseStamped
    vector<geometry_msgs::PoseStamped> path_poses;
    convertVectorToPoses(path_poses);

    //Set the plan that the controller is following (inside plan is transformed to map frame we are working in)
    bool plan_set_ok = m_eband_planner->setPlan(path_poses);

    //Let the local controller execute the path
    if(plan_set_ok){

        while(!m_eband_planner->isGoalReached()){

            //Base velocity command
            geometry_msgs::Twist cmd_base_vel;

            //Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
            // ->True if a valid trajectory was found, false otherwise
            bool valid_traj_found = m_eband_planner->computeVelocityCommands(cmd_base_vel);

            if(!valid_traj_found){
                goto EXECUTION_FAILED;
            }
            else{
                //Send velocity command to platform
                m_base_vel_pub.publish(cmd_base_vel);
            }
        }

         ROS_INFO("Trajectory executed successfully");

    }
    else{
        goto EXECUTION_FAILED;
    }

    EXECUTION_FAILED: ROS_ERROR("Invalid Base Trajectory encountered!!! Stopping local elastic band controller");
*/
}


//Execute planned trajectory for base + arm
void MotionCommanderRobot::execute_base_arm_trajectory(){


    //---------------- TRAJECTORY DATA -----------------

    //Get duration of trajectory
    double duration = m_joint_trajectory->getDuration();

    //Number of configs along trajectory depends on trajectory sampling step width
    int num_configs = floor(duration/m_traj_sampling_step) + 1;


    //---------------- SETUP OMNIROB BASE -----------------

    //Velocity commanded to the base
    geometry_msgs::Twist base_vel;

    //Print the Trajectory Data
    //printTrajectoryData(duration,m_traj_sampling_step);

    //Get initial robot pose (in map frame)
    tf::StampedTransform start_transform_map_to_base;
    try {
        m_listener.waitForTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), ros::Duration(10.0) );
        m_listener.lookupTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), start_transform_map_to_base);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    //Current waypoint pose along the trajectory
    vector<double> base_wp_pose(3);

    //Current base pose error w.r.t current waypoint
    vector<double> base_pose_error(3);


    //---------------- SETUP LBR ARM -----------------


    //ros::Publisher joint_state_pub = m_nh_.advertise<sensor_msgs::JointState>(m_ns_prefix_robot + "omnirob/cmd_joint_state", 1000);

    //Joint State message
    //sensor_msgs::JointState arm_conf;
    //arm_conf.position.resize(7);

    // Create the action client
    // -> action client constructor also takes two arguments, the server name to connect to and a boolean option to automatically spin a thread.
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> fjt(m_ns_prefix_robot + "follow_joint_trajectory", true);

    // Wait for the action server to start
    ROS_INFO("'execute_trajectory': Waiting for action server to start.");
    fjt.waitForServer(); //will wait for infinite time

    //Follow Joint Trajectory Action Goal
    control_msgs::FollowJointTrajectoryGoal follow_joint_traj_goal;

    //Joint Trajectory Message
    trajectory_msgs::JointTrajectory joint_trajectory_msg;

    //Start index of LBR Joints
    int lbr_joints_start_idx = 3;

    //Set size of arrays storing the joint names and trajectory points for the LBR arm
    joint_trajectory_msg.joint_names.resize(m_num_joints-lbr_joints_start_idx);
    //Only next trajectory point considered for execution
    joint_trajectory_msg.points.resize(1);

    //Set size of joint positions array
    joint_trajectory_msg.points[0].positions.resize(m_num_joints-lbr_joints_start_idx);
    joint_trajectory_msg.points[0].velocities.resize(m_num_joints-lbr_joints_start_idx);

    //Set Frame ID in header
    joint_trajectory_msg.header.frame_id = m_ns_prefix_robot + "lbr_0_link";

    //Set joint names
    for (int i = lbr_joints_start_idx ; i < m_num_joints; i++)
    {
        //Set Joint Name
        joint_trajectory_msg.joint_names[i-lbr_joints_start_idx] = m_joint_names[i];

        //Set Joint names
        //arm_conf.name.push_back(m_joint_names[i]);
    }


    //---------------- EXECUTION -----------------

    //Generate Joint Trajectory message if trajectory generation succeeded
    if(m_joint_trajectory->isValid())
    {

        //Take samples from the generated trajectory
        for(double t = 0.0; t < duration+m_traj_sampling_step; t += m_traj_sampling_step)
        {
            //To avoid exeeding the trajectory duration
            if(duration <= t)
            {
                t = duration;

                //Set waypoint distance threshold lower for last trajectory point
                //-> in order to reach final base pose accurately
                m_error_thr_trans = 0.01;
                m_error_thr_rot = 0.01;

                cout<<"Proceeding to last trajectory point"<<endl;
            }
            else
            {
                cout<<"Proceeding to next trajectory point at time: "<<t<<endl;
            }

            //+++++ Waypoint processing for Omnirob Base +++++

            //Get base pose of waypoint
            base_wp_pose[0] = m_joint_trajectory->getPosition(t)[0]; //X pos
            base_wp_pose[1] = m_joint_trajectory->getPosition(t)[1]; //Y pos
            base_wp_pose[2] = m_joint_trajectory->getPosition(t)[2]; //Theta orientation

            //Init deviation of current base pose from waypoint
            base_pose_error = compute_base_pose_error(base_wp_pose,start_transform_map_to_base);

            cout<<"x err: "<<base_pose_error[0]<<endl;
            cout<<"y err: "<<base_pose_error[1]<<endl;
            cout<<"theta err: "<<base_pose_error[2]<<endl;

            //+++++ Waypoint processing for LBR Arm +++++

            //Get lbr joint config of waypoint
            for (int j = lbr_joints_start_idx ; j < m_num_joints; j++)
            {
              //Set joint position
              joint_trajectory_msg.points[0].positions[j-lbr_joints_start_idx] = m_joint_trajectory->getPosition(t)[j] ;
              //Set joint velocity
              //joint_trajectory_msg.points[curr_point_idx].velocities[j] = m_joint_trajectory->getVelocity(t)[j];
              //Set time from start
              joint_trajectory_msg.points[0].time_from_start = ros::Duration(t);

              //arm_conf.position[j-lbr_joints_start_idx] = m_joint_trajectory->getPosition(t)[j];
            }

            // Set the joint trajectory to follow
            follow_joint_traj_goal.trajectory = joint_trajectory_msg;

            //+++++ CONTROL LOOP +++++

            //Send LBR arm joint trajectory to platform ....
            fjt.sendGoal(follow_joint_traj_goal);

            //Publish home position
            //joint_state_pub.publish(arm_conf);

            //.... meanwhile drive the omnirob base to the 2D waypoint
            while(m_error_thr_trans < fabs(base_pose_error[0]) || m_error_thr_trans < fabs(base_pose_error[1]) || m_error_thr_rot < fabs(base_pose_error[2]))
            {

                //Set base translational velocity (in X and Y direction)
                base_vel.linear.x = m_linear_vel_base_gain * base_pose_error[0];
                base_vel.linear.y = m_linear_vel_base_gain * base_pose_error[1];
                base_vel.linear.z = 0.0;

                //Set base rotational velocity
                base_vel.angular.x = 0.0;
                base_vel.angular.y = 0.0;
                base_vel.angular.z = m_angular_vel_base_gain * base_pose_error[2];

                //cout<<"x vel: "<<m_linear_vel_base_gain * base_pose_error[0]<<endl;
                //cout<<"y vel: "<<m_linear_vel_base_gain * base_pose_error[1]<<endl;
                //cout<<"theta vel: "<<m_angular_vel_base_gain * base_pose_error[2]<<endl;


                //Check for velocity limits
                if(fabs(base_vel.linear.x) <= m_linear_vel_limit && fabs(base_vel.linear.y) <= m_linear_vel_limit && fabs(base_vel.angular.z) <= m_angular_vel_limit)
                {

                  ROS_INFO("Base velocities are within joint limits");
                }
                else
                {
                    //Scale velocities to obey the joint speed limits
                    double speed_scaling = 1.0;
                    //Compute speed scaling factor
                    if(m_linear_vel_limit < fabs(base_vel.linear.x)){
                        speed_scaling = m_linear_vel_limit / fabs(base_vel.linear.x);
                    } else if (m_linear_vel_limit < fabs(base_vel.linear.y)){
                        speed_scaling = m_linear_vel_limit / fabs(base_vel.linear.y);
                    } else if(m_angular_vel_limit < fabs(base_vel.angular.z)){
                        speed_scaling = m_angular_vel_limit / fabs(base_vel.angular.z);
                    }

                    //Scale velocities proportionally
                    base_vel.linear.x *= speed_scaling;
                    base_vel.linear.y *= speed_scaling;
                    base_vel.angular.z *= speed_scaling;

                    ROS_INFO("Base velocities scaled to obey defined speed limits!!!");
                }

                //Send base velocity command to platform
                m_base_vel_pub.publish(base_vel);

                //Update error between current robot pose and desired pose (given by waypoint)
                base_pose_error = compute_base_pose_error(base_wp_pose,start_transform_map_to_base);

            } //End of approaching the current waypoint

        }//end of iteration through trajectory

        //Trajectory executed
        ROS_INFO("Trajectory executed successfully!!!");


    }//end if generated trajectory from path is valid
    else{
        ROS_ERROR("Generated Trajectory is invalid!!!");
    }

}

//Convert vector of double vectors to vector of geometry_msgs::PoseStamped
void MotionCommanderRobot::convertVectorToPoses(vector<geometry_msgs::PoseStamped> &poses){

    //A robot pose along the path
    geometry_msgs::PoseStamped pose;
    //#Frame this data is associated with
    //# 0: no frame
    //# 1: global frame
    pose.header.frame_id = "base_link";

    //Iterate through waypoints
    for (int wp = 0 ; wp < m_joint_path.size() ; wp++){

        //Set sequence ID: consecutively increasing ID
        pose.header.seq = wp;

        //Set robot pose of waypoint
        pose.pose.position.x = m_joint_path[wp][0];
        pose.pose.position.y = m_joint_path[wp][1];
        pose.pose.position.z = 0.0;
        tf::Quaternion quat = tf::createQuaternionFromYaw(m_joint_path[wp][2]);
        pose.pose.orientation.x = quat.getX();
        pose.pose.orientation.y = quat.getY();
        pose.pose.orientation.z = quat.getZ();
        pose.pose.orientation.w = quat.getW();

        //Collect poses
        poses.push_back(pose);
    }
}

//Compute error between current robot pose and desired robot pose (in map frame)
vector<double> MotionCommanderRobot::compute_base_pose_error(vector<double> base_wp_pose, tf::StampedTransform initial_transform_map_to_base)
{
    //Pose error to be returned
    vector<double> base_pose_error(3);

    //++++ CURRENT ROBOT POSE IN MAP ++++

    //Current robot pose (in map frame)
    tf::StampedTransform curr_transform_map_to_base;
    try {
        m_listener.waitForTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), ros::Duration(10.0) );
        m_listener.lookupTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), curr_transform_map_to_base);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }


    //++++ DESIRED ROBOT POSE IN MAP ++++

    //Get desired pose from trajectory (expressed in start base_link)
    tf::StampedTransform transform_base_to_waypoint_init_base_link;
    transform_base_to_waypoint_init_base_link.setOrigin(tf::Vector3(base_wp_pose[0],base_wp_pose[1], 0.0));
    transform_base_to_waypoint_init_base_link.setRotation(tf::createQuaternionFromYaw(base_wp_pose[2]));

    //Express desired pose from trajectory in map frame
    tf::StampedTransform transform_map_to_waypoint;
    transform_map_to_waypoint.mult(initial_transform_map_to_base,transform_base_to_waypoint_init_base_link);

    //++++ CURRENT ROBOT POSE ERROR IN MAP ++++

    //Transform current robot pose to waypoint
    //tf::StampedTransform transform_curr_base_to_waypoint_map;
    //transform_curr_base_to_waypoint_map.mult(curr_transform_map_to_base.inverse(),transform_map_to_waypoint);

    //++++ ROBOT POSE ERROR IN CURRENT BASE_LINK ++++

    tf::StampedTransform transform_curr_base_to_waypoint;
    transform_curr_base_to_waypoint.mult(curr_transform_map_to_base.inverse(),transform_map_to_waypoint);


//    cout<<"Current robot pose in map:"<<endl;
//    cout<<curr_transform_map_to_base.getOrigin().x()<<endl;
//    cout<<curr_transform_map_to_base.getOrigin().y()<<endl;
//    cout<<curr_transform_map_to_base.getRotation().getAngle()<<endl;

//    cout<<"Desired robot pose in map:"<<endl;
//    cout<<transform_map_to_waypoint.getOrigin().x()<<endl;
//    cout<<transform_map_to_waypoint.getOrigin().y()<<endl;
//    cout<<transform_map_to_waypoint.getRotation().getAngle()<<endl;

//    cout<<"Desired robot pose in current pose frame:"<<endl;
//    cout<<transform_curr_base_to_waypoint.getOrigin().x()<<endl;
//    cout<<transform_curr_base_to_waypoint.getOrigin().y()<<endl;
//    cout<<transform_curr_base_to_waypoint.getRotation().getAngle()<<endl;

    //Convert transformation from current base_link to waypoint to vector representation
    tf::Vector3 error_trans = transform_curr_base_to_waypoint.getOrigin();
    tf::Quaternion error_rot = transform_curr_base_to_waypoint.getRotation();
    base_pose_error[0] = error_trans.x();
    base_pose_error[1] = error_trans.y();
    double z_dir = error_rot.getAxis().z();
    base_pose_error[2] = z_dir > 0.0 ? error_rot.getAngle() : -error_rot.getAngle();

    //Return base pose error
    return base_pose_error;
}


//Prints the Trajectory Points with sampling rate "time_step"
void MotionCommanderRobot::printTrajectoryData(double duration, double time_step){

    cout << "Time      Position            Velocity" << endl;
    for(double t = 0.0; t < duration; t += time_step)
    {
//        if (m_planning_group == "kuka_complete_arm")
//        {
//            printf("%6.2f   %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f  %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n", t, m_joint_trajectory->getPosition(t)[0], m_joint_trajectory->getPosition(t)[1], m_joint_trajectory->getPosition(t)[2], m_joint_trajectory->getPosition(t)[3], m_joint_trajectory->getPosition(t)[4], m_joint_trajectory->getPosition(t)[5], m_joint_trajectory->getPosition(t)[6],
//            m_joint_trajectory->getVelocity(t)[0], m_joint_trajectory->getVelocity(t)[1], m_joint_trajectory->getVelocity(t)[2],m_joint_trajectory->getVelocity(t)[3], m_joint_trajectory->getVelocity(t)[4], m_joint_trajectory->getVelocity(t)[5], m_joint_trajectory->getVelocity(t)[6]);
//        }
//        else if (m_planning_group == "omnirob_base")
//        {
//            printf("%6.2f   %7.2f %7.2f %7.2f   %7.2f %7.2f %7.2f\n", t, m_joint_trajectory->getPosition(t)[0], m_joint_trajectory->getPosition(t)[1], m_joint_trajectory->getPosition(t)[2], m_joint_trajectory->getVelocity(t)[0], m_joint_trajectory->getVelocity(t)[1], m_joint_trajectory->getVelocity(t)[2]);
//        }
//        else if(m_planning_group == "omnirob_lbr_sdh")
//        {
//            printf("%6.2f   %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f  %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n", t, m_joint_trajectory->getPosition(t)[0], m_joint_trajectory->getPosition(t)[1], m_joint_trajectory->getPosition(t)[2], m_joint_trajectory->getPosition(t)[3], m_joint_trajectory->getPosition(t)[4], m_joint_trajectory->getPosition(t)[5], m_joint_trajectory->getPosition(t)[6], m_joint_trajectory->getPosition(t)[7], m_joint_trajectory->getPosition(t)[8], m_joint_trajectory->getPosition(t)[9],
//            m_joint_trajectory->getVelocity(t)[0], m_joint_trajectory->getVelocity(t)[1], m_joint_trajectory->getVelocity(t)[2],m_joint_trajectory->getVelocity(t)[3], m_joint_trajectory->getVelocity(t)[4], m_joint_trajectory->getVelocity(t)[5], m_joint_trajectory->getVelocity(t)[6],m_joint_trajectory->getVelocity(t)[7], m_joint_trajectory->getVelocity(t)[8], m_joint_trajectory->getVelocity(t)[9]);
//        }
//        else
//            ROS_ERROR("Unknown Planning Group");

        for (int joint = 0 ; joint < m_num_joints ; joint++)
        {
            cout<<m_joint_trajectory->getPosition(t)[joint]<<" ";
        }
        cout<<endl;
        for (int joint = 0 ; joint < m_num_joints ; joint++)
        {
            cout<<m_joint_trajectory->getVelocity(t)[joint]<<" ";
        }
        cout<<endl;
    }

    for (int joint = 0 ; joint < m_num_joints ; joint++)
    {
        cout<<m_joint_trajectory->getPosition(duration)[joint]<<" ";
    }
    cout<<endl;
    for (int joint = 0 ; joint < m_num_joints ; joint++)
    {
        cout<<m_joint_trajectory->getVelocity(duration)[joint]<<" ";
    }
    cout<<endl;

//    if (m_planning_group == "kuka_complete_arm")
//    {
//        printf("%6.2f   %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f  %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n", duration, m_joint_trajectory->getPosition(duration)[0], m_joint_trajectory->getPosition(duration)[1], m_joint_trajectory->getPosition(duration)[2], m_joint_trajectory->getPosition(duration)[3], m_joint_trajectory->getPosition(duration)[4], m_joint_trajectory->getPosition(duration)[5], m_joint_trajectory->getPosition(duration)[6],
//        m_joint_trajectory->getVelocity(duration)[0], m_joint_trajectory->getVelocity(duration)[1], m_joint_trajectory->getVelocity(duration)[2],m_joint_trajectory->getVelocity(duration)[3], m_joint_trajectory->getVelocity(duration)[4], m_joint_trajectory->getVelocity(duration)[5], m_joint_trajectory->getVelocity(duration)[6]);
//    }
//    else if (m_planning_group == "omnirob_base")
//    {
//        printf("%6.2f   %7.2f %7.2f %7.2f   %7.2f %7.2f %7.2f\n", duration, m_joint_trajectory->getPosition(duration)[0], m_joint_trajectory->getPosition(duration)[1], m_joint_trajectory->getPosition(duration)[2], m_joint_trajectory->getVelocity(duration)[0], m_joint_trajectory->getVelocity(duration)[1], m_joint_trajectory->getVelocity(duration)[2]);
//    }
//    else if(m_planning_group == "omnirob_lbr_sdh")
//    {
//        printf("%6.2f   %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f  %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n", duration, m_joint_trajectory->getPosition(duration)[0], m_joint_trajectory->getPosition(duration)[1], m_joint_trajectory->getPosition(duration)[2], m_joint_trajectory->getPosition(duration)[3], m_joint_trajectory->getPosition(duration)[4], m_joint_trajectory->getPosition(duration)[5], m_joint_trajectory->getPosition(duration)[6], m_joint_trajectory->getPosition(duration)[7], m_joint_trajectory->getPosition(duration)[8], m_joint_trajectory->getPosition(duration)[9],
//        m_joint_trajectory->getVelocity(duration)[0], m_joint_trajectory->getVelocity(duration)[1], m_joint_trajectory->getVelocity(duration)[2],m_joint_trajectory->getVelocity(duration)[3], m_joint_trajectory->getVelocity(duration)[4], m_joint_trajectory->getVelocity(duration)[5], m_joint_trajectory->getVelocity(duration)[6], m_joint_trajectory->getVelocity(duration)[7], m_joint_trajectory->getVelocity(duration)[8], m_joint_trajectory->getVelocity(duration)[9]);
//    }
//    else
//        ROS_ERROR("Unknown Planning Group");
}




} //end of namespace



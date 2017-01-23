#include <motion_trajectory_execution/trajectory_execution_vrep.h>

namespace trajectory_execution{


//Default Constructor
MotionCommanderVREP::MotionCommanderVREP()
{

}

//Constructor with planning group
MotionCommanderVREP::MotionCommanderVREP(string planning_group) : MotionCommanderRVIZ(planning_group)
{

}

//Constructor with planning group and scene
MotionCommanderVREP::MotionCommanderVREP(string planning_group, string planning_scene) : MotionCommanderRVIZ(planning_group,planning_scene)
{
    //Store planning group name
    m_planning_group = planning_group;

    //Get namespaces of robot
    m_nh_.param("ns_omnirob_robot", m_ns_prefix_robot, std::string(""));

    //Get name of robot_description parameter
    string robot_description_robot;
    m_nh_.param("robot_description_omnirob_robot", robot_description_robot, std::string("robot_description"));


    //Create Robot model
    m_KDLManipulatorModel = boost::shared_ptr<kuka_motion_controller::KDLRobotModel>(new kuka_motion_controller::KDLRobotModel(robot_description_robot, m_ns_prefix_robot + "planning_scene", m_ns_prefix_robot + "endeffector_trajectory", planning_group));


    if (planning_group == "kuka_complete_arm")
    {
        //Control modes for the robot joints
        m_joint_control_modes.resize(13);
    }
    else if (planning_group == "omnirob_lbr_sdh")
    {
        //Control modes for the robot joints
        m_joint_control_modes.resize(17);
    }
    else
    {

    }

    //Create planning scene monitor
    //psm_ = boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));


    //Planning Scene Publisher
    //scene_pub_ = m_nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 10);
    //ros::Duration(0.5).sleep();

    //Set gripper closing/opening speed
    m_gripper_opening_closing_speed = 0.3;

    //Clear joint trajectory
    m_joint_pos_trajectory.clear();
    m_joint_vel_trajectory.clear();

    //Base rotation offset between rviz and vrep robot representation
    m_base_rot_offset_rviz_vrep = -1.57079632679; //-90 degree

    //Comparison of LBR joint rotation axes between rviz and vrep LBR robot representation
    // ->  1  : axes of rotation are identical
    // -> -1 : axes of rotation pointing in oposite directions
    m_dir_rotation_axes_rviz_vrep.push_back(1.0); //From empirical evaluation
    m_dir_rotation_axes_rviz_vrep.push_back(-1.0);
    m_dir_rotation_axes_rviz_vrep.push_back(1.0);
    m_dir_rotation_axes_rviz_vrep.push_back(1.0);
    m_dir_rotation_axes_rviz_vrep.push_back(1.0);
    m_dir_rotation_axes_rviz_vrep.push_back(-1.0);
    m_dir_rotation_axes_rviz_vrep.push_back(1.0);


    // Global variables (modified by topic subscribers):
    m_simulationRunning=true;
    m_simulationTime=0.0f;
    m_motor_handles_available = false;

    // 1. Let's subscribe to V-REP's info stream (that stream is the only one enabled by default,
    // and the only one that can run while no simulation is running):
    m_subInfo = m_nh_.subscribe("/vrep/info",1,&MotionCommanderVREP::infoCallback,this);

    //Prepare Subcriber and Publisher
    if (planning_group == "kuka_complete_arm" || planning_group == "omnirob_lbr_sdh" || planning_group == "robotino")
    {
       //Prepare Subcriber for motor handles:
       m_subMotorHandles = m_nh_.subscribe("/vrep_ros_communication/motor_handles",1,&MotionCommanderVREP::motorhandlesCallback,this);
       //Prepare Publisher for omnirob wheel speeds / LBR joints motor speeds:
       m_omnirob_motor_speedPub = m_nh_.advertise<vrep_common::JointSetStateData>("/vrep_ros_communication/omnirob_wheels",1);
       m_lbr_motor_speedPub = m_nh_.advertise<vrep_common::JointSetStateData>("/vrep_ros_communication/lbr_joints",1);
       m_schunk_sdh2_motor_speedPub = m_nh_.advertise<vrep_common::JointSetStateData>("/vrep_ros_communication/schunk_sdh2_joints",1);

       //Prepare Publisher for robotino wheel speeds
       m_robotino_motor_speedPub = m_nh_.advertise<vrep_common::JointSetStateData>("/vrep_ros_communication/robotino_wheels",1);
    }
    else
        ROS_ERROR("Unknown Planning Group");


    // Setup service to get current mobile platform base pose from Vrep
    client_base_pose = m_nh_.serviceClient<vrep_common::simRosGetObjectPose>("/vrep/simRosGetObjectPose");

    // Service client to get current joint positions of manipulator from Vrep
    client_joint_config = m_nh_.serviceClient<vrep_common::simRosGetJointState>("/vrep/simRosGetJointState");


    //Start the simulation mode in VREP
    ros::ServiceClient client_startSimulation=m_nh_.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
    vrep_common::simRosStartSimulation srv_startSimulation;
    client_startSimulation.call(srv_startSimulation);

}

//Destructor
MotionCommanderVREP::~MotionCommanderVREP()
{

    //Clear joint trajectory
    m_joint_pos_trajectory.clear();
    m_joint_vel_trajectory.clear();
}



// Topic subscriber callbacks:
void MotionCommanderVREP::infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
    m_simulationTime=info->simulationTime.data;
    m_simulationRunning=(info->simulatorState.data&1)!=0;
}


void MotionCommanderVREP::motorhandlesCallback(const std_msgs::Int32MultiArray::ConstPtr& handles_array)
{
    //Set motor handle vector size
    m_motor_handles.resize(handles_array->data.size());

    //Get the motor handle ID's from the message
    int i = 0;
    for(std::vector<int>::const_iterator it = handles_array->data.begin(); it != handles_array->data.end(); ++it)
        {
            m_motor_handles[i] = *it;
            i++;
        }

    //Motor Handles are now set
    m_motor_handles_available = true;

}



//Set the joint control modes
bool MotionCommanderVREP::setJointControlModes(vector<int> joint_control_modes)
{
    bool modes_set = false;

    //Publish control modes
    ros::ServiceClient client_setControlModes=m_nh_.serviceClient<vrep_common::simRosSetObjectIntParameter>("/vrep/simRosSetObjectIntParameter");
    vrep_common::simRosSetObjectIntParameter setControlModes;
    //Set streamCmd
    setControlModes.request.parameter = sim_jointintparam_ctrl_enabled;

    //Set Motor control mode for Robot Joints (last handle is for the entire robot -> used to get the current robot pose)
    for (int i = 0; i < m_motor_handles.size()-1; i++)
    {
        setControlModes.request.handle = m_motor_handles[i];
        setControlModes.request.parameterValue = joint_control_modes[i];
        //Set motor control modes for current joint
        modes_set = client_setControlModes.call(setControlModes);

        //cout<<m_motor_handles[i] <<" : "<<joint_control_modes[i];
    }

    return modes_set;
}


//Load Joint Trajectory from file and execute it
void MotionCommanderVREP::executeJointTrajectory(char* joint_trajectory_file)
{

    // NOTE !!!!!!!!!!!!!!!!!!!!!!
    //+++++++++ USE NEWTON PHYSICS ENGINE IN V_REP FOR CORRECT TRAJECTORY EXECUTION++++++++++++++++++++

    //------------- Get Trajectory from planner -----------------------

    //Get Joint Trajecotry data
    loadJointTrajectory(joint_trajectory_file);

    //Get the loaded Joint Trajectory
    m_joint_pos_trajectory = getJointTrajectory();

    //------------- Convert Motion Plan to Joint Trajectory -----------------------

    convertMotionPlanToTrajectory();

    //Get duration of trajectory
    double duration = m_trajectory->getDuration();


    //------------- Get Motor handles from "vrep_ros_communication" interface -----------------------

    //Wait until motor handles become available (published by vrep_ros_communication node)
    while(m_motor_handles_available == false)
    {
        ROS_INFO("Waiting for motor handles callback. Make sure to activate simulation in V-REP!!!");

        // handle ROS messages:
        ros::spinOnce();

        // sleep a bit:
        usleep(5000);
    }
    ROS_INFO("Robot motor handles available!");


    //------------- Set Initial Robot Pose -----------------------

    //Start of manipulator joints in trajectory config vector
    int start_idx_manip_joints_conf_vector = 0;

    //Set Pose
    setInitialRobotPose(start_idx_manip_joints_conf_vector);

    int t;
    cin>>t;


    //------------- Set Control Mode for Joints (Position vs. Velocity Control) -----------------------

    //Index of first manipulator motor handle
    int start_idx_manip_joints_motor_handle_vector = 0;

    //Set the joint control modes (position or velocity mode)
    setInitialControlModes(start_idx_manip_joints_motor_handle_vector);



    //------------- Open / Close Hand Initially -----------------------

    //Close the Schunk SDH2 Hand
    //closeHand();

    //cin>>t;


    //lockHand();

    //cin>>t;


    //------------- Send Position and velocity commands to the robot -----------------------

    //Set the trajectory sampling rate in seconds
    double traj_sampling_rate = 3.0;



    // 6. Finally we have the control loop:
    while (ros::ok() && m_simulationRunning)
    { // this is the control loop (very simple, just as an example)

        //Init trajectory time instance
        double trajectory_time = 0.0;

        //Set Gains for linear and angular velicity components
        m_linear_vel_base_gain = 0.3;  // - for omnirob base translational
        m_angular_vel_base_gain = 0.3; // - for omnirob base rotational
        m_angular_vel_lbr_gain = 0.5;  // - for lbr manipulator joints

        //Threshold for stepping forward along the trajectory 
        double tracking_error_threshold_base_translational = 0.2; // - for omnirob base translational
        double tracking_error_threshold_base_rotational = 0.2;    // - for omnirob base rotational
        double tracking_error_threshold_lbr_rotational = 0.1;     // - for lbr manipulator joints

        while (trajectory_time <= duration)
        {
            //Offset between current base pose and desired trajectory base point
            double delta_x, delta_y, delta_theta, curr_theta_global;

            //Offset between current manipulator config and desired trajectory point manipulator config
            vector<double> delta_joint_configs;

            //Flag indicating whether base / manipulator reached trajectory target point
            bool base_reached_target = true;
            bool manip_reached_target = true;


            //++++++++++++++++++ Compute trajectory offset +++++++++++++++++++++++++++++

            //--- For Robot Mobile Base
            if(m_planning_group == "omnirob_lbr_sdh" || m_planning_group == "omnirob"){

                //Compute offset of current base pose w.r.t desired base pose given by the trajectory point at time "trajectory_time"
                getTrajectoryOffsetBase(delta_x, delta_y, delta_theta, curr_theta_global, trajectory_time);

                //Check Base target distance
                if (fabs(delta_x) < tracking_error_threshold_base_translational && fabs(delta_y) < tracking_error_threshold_base_translational && fabs(delta_theta) < tracking_error_threshold_base_rotational)
                {
                    base_reached_target = true;
                }
                else{

                    base_reached_target = false;

                    cout<<"Approaching trajectory base pose with time stamp: "<<trajectory_time<<endl;

                    //                cout<<"delta_x: "<<delta_x<<endl;
                    //                cout<<"delta_y: "<<delta_y<<endl;
                    //                //cout<<"des_theta_global: "<<des_theta_global<<endl;
                    //                //cout<<"curr_theta_global: "<<curr_theta_global<<endl;
                    //                cout<<"des_theta_global: "<<des_theta_global<<endl;
                    //                cout<<"curr_theta_global: "<<curr_theta_global<<endl;
                    //                cout<<"delta_theta: "<<delta_theta<<endl;

                }
            }


            //--- For Manipulator Joints
            if(m_planning_group == "omnirob_lbr_sdh" || m_planning_group == "kuka_complete_arm"){

                //Compute offset of current joint angles w.r.t trajectory point joint angles
                delta_joint_configs = getTrajectoryOffsetManipulator(start_idx_manip_joints_motor_handle_vector, start_idx_manip_joints_conf_vector, trajectory_time);

                //Check Manipulator target distance
                if(fabs(delta_joint_configs[0]) <  tracking_error_threshold_lbr_rotational && fabs(delta_joint_configs[1]) <  tracking_error_threshold_lbr_rotational && fabs(delta_joint_configs[2]) <  tracking_error_threshold_lbr_rotational && fabs(delta_joint_configs[3]) <  tracking_error_threshold_lbr_rotational \
                        && fabs(delta_joint_configs[4]) <  tracking_error_threshold_lbr_rotational && fabs(delta_joint_configs[5]) <  tracking_error_threshold_lbr_rotational && fabs(delta_joint_configs[6]) <  tracking_error_threshold_lbr_rotational)
                {
                    manip_reached_target = true;
                }
                else{

                    manip_reached_target = false;

                    cout<<"Approaching trajectory manipulator joint config with time stamp: "<<trajectory_time<<endl;

                    //Print joint deltas not within error tolerance
                    for(int i = 0 ; i < 7 ; i++)
                    {
                        if(tracking_error_threshold_lbr_rotational < delta_joint_configs[i])
                        {
                            //cout<<"Joint "<<i<<" current val is: "<<curr_joint_config[i]<<endl;
                            //cout<<"Joint "<<i<<" desired val is: "<<des_joint_config[i]<<endl;
                            //cout<<"Joint "<<i<<" delta is: "<<delta_joint_configs[i]<<endl;
                            //cout<<"Joint "<<i<<" desired velocity is: "<<joint_velocities[i]<<endl;
                        }
                    }
                }
            }


            //++++++++++++++++++ Check trajectory offset (for base and manipulator)+++++++++++++++++++++++++++++

            //Check whether robot is close enough to the desired trajectory point
            if (base_reached_target && manip_reached_target){

                    if(trajectory_time == duration)
                    {
                        cout<<"Proceeding to motion stop sequence"<<endl;

                        //Leave loop when last trajectory point has been reached
                        break;
                    }

                    //Go forward along the trajectory
                    trajectory_time += traj_sampling_rate;

                    if(duration < trajectory_time)
                    {
                        trajectory_time = duration;

                        //Set lower target distance threshold for last trajectory point
                        //-> in order to reach grasp pose accurately
                        tracking_error_threshold_base_translational = 0.01;
                        tracking_error_threshold_base_rotational = 0.01;
                        tracking_error_threshold_lbr_rotational = 0.01;

                        cout<<"Proceeding to last trajectory point"<<endl;
                    }
                    else
                        cout<<"Proceeding to next trajectory point"<<endl;

                    //Start moving towards next trajectory waypoint
                    continue;
            }
            else if (base_reached_target){
                //Stop base motion
                stopBaseMotion();
            }
            else if(manip_reached_target){
                //Stop base motion
                stopManipulatorMotion(start_idx_manip_joints_motor_handle_vector);
            }
            else
                cout<<"Approaching trajectory point with time stamp: "<<trajectory_time<<endl;



            //++++++++++++++++++ Set Motor/Joint Speeds depending on Trajectory Offset +++++++++++++++++++++++++++++

            //--- For Robot Mobile Base
            if(!base_reached_target){

                //Compute Base Velocity in Local Base Frame
                double x_vel_local, y_vel_local,theta_vel_local;
                computeBaseVelocityLocal(delta_x, delta_y, delta_theta, curr_theta_global, x_vel_local, y_vel_local, theta_vel_local);

                //Compute and execute required wheel speeds given the linear and angular base velocity
                setWheelSpeedsOmnirobBase(x_vel_local, y_vel_local, theta_vel_local);
            }

            //--- For Manipulator Joints
            if(!manip_reached_target){


                //Compute Joint Velocities for Manipulator given the offset to the current trajectory point
                vector<double> joint_velocities;
                joint_velocities = computeJointVelocities(delta_joint_configs);

                //Compute and execute required joint velocities
                setJointSpeedsManipulator(start_idx_manip_joints_motor_handle_vector, joint_velocities);

            }

        } //End trajectory point iterations


        //------------- STOP MOTION ----------------------
        if(m_planning_group == "omnirob_lbr_sdh" || m_planning_group == "omnirob"){
            stopBaseMotion();
        }

        if(m_planning_group == "omnirob_lbr_sdh" || m_planning_group == "kuka_complete_arm"){
            stopManipulatorMotion(start_idx_manip_joints_motor_handle_vector);
        }

        //Close Hand
        closeHand();

        break;
        //-------------------------------------------

    }

}


//Convert the Motion Plan to a Joint Trajectory
void MotionCommanderVREP::convertMotionPlanToTrajectory(){

    //------------- Convert Trajectory from planner into Joint Velocity Commands-----------------------

    //Convert Trajectory into waypoints list
    list<Eigen::VectorXd> waypoints;
    for (int wp = 0; wp < m_joint_pos_trajectory.size(); wp++)
    {
        Eigen::VectorXd waypoint(m_joint_pos_trajectory[wp].size());
        for (int j = 0; j < m_joint_pos_trajectory[wp].size(); j++)
        {
            waypoint[j] = m_joint_pos_trajectory[wp][j];

        }
        waypoints.push_back(waypoint);
    }

    //Set Velocity and Acceleration Bounds
    Eigen::VectorXd maxAcceleration;
    Eigen::VectorXd maxVelocity;
    if (m_planning_group == "kuka_complete_arm")
    {
        maxAcceleration.resize(7);
        maxVelocity.resize(7);
        maxAcceleration << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
        maxVelocity << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    }
    else if(m_planning_group == "omnirob_lbr_sdh")
    {
        maxAcceleration.resize(10);
        maxVelocity.resize(10);
        maxAcceleration << 1.0, 1.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 ;
        maxVelocity << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    }
    else
        ROS_ERROR("Unknown Planning Group");


    //Compute Trajectory (taking velocity and acceleration upper bounds into account)
    m_trajectory = boost::shared_ptr<Trajectory>(new Trajectory(Path(waypoints, 0.1), maxVelocity, maxAcceleration));

    //Trajectory trajectory(Path(waypoints, 0.1), maxVelocity, maxAcceleration);

    //Write Trajectory Data to File
    m_trajectory->outputPhasePlaneTrajectory();
    //Get duration of trajectory
    double duration = m_trajectory->getDuration();
    if(m_trajectory->isValid())
    {
        cout << "Trajectory duration: " << duration << " s" << endl << endl;

        /*
        cout << "Time      Position                  Velocity" << endl;
        for(double t = 0.0; t < duration; t += 0.1)
        {
            if (m_planning_group == "kuka_complete_arm")
            {
                printf("%6.2f   %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f  %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n", t, m_trajectory->getPosition(t)[0], m_trajectory->getPosition(t)[1], m_trajectory->getPosition(t)[2], m_trajectory->getPosition(t)[3], m_trajectory->getPosition(t)[4], m_trajectory->getPosition(t)[5], m_trajectory->getPosition(t)[6],
                m_trajectory->getVelocity(t)[0], m_trajectory->getVelocity(t)[1], m_trajectory->getVelocity(t)[2],m_trajectory->getVelocity(t)[3], m_trajectory->getVelocity(t)[4], m_trajectory->getVelocity(t)[5], m_trajectory->getVelocity(t)[6]);
            }
            else if(m_planning_group == "omnirob_lbr_sdh")
            {
                printf("%6.2f   %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f  %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n", t, m_trajectory->getPosition(t)[0], m_trajectory->getPosition(t)[1], m_trajectory->getPosition(t)[2], m_trajectory->getPosition(t)[3], m_trajectory->getPosition(t)[4], m_trajectory->getPosition(t)[5], m_trajectory->getPosition(t)[6], m_trajectory->getPosition(t)[7], m_trajectory->getPosition(t)[8], m_trajectory->getPosition(t)[9],
                m_trajectory->getVelocity(t)[0], m_trajectory->getVelocity(t)[1], m_trajectory->getVelocity(t)[2],m_trajectory->getVelocity(t)[3], m_trajectory->getVelocity(t)[4], m_trajectory->getVelocity(t)[5], m_trajectory->getVelocity(t)[6],m_trajectory->getVelocity(t)[7], m_trajectory->getVelocity(t)[8], m_trajectory->getVelocity(t)[9]);
            }
            else
                ROS_ERROR("Unknown Planning Group");
        }
        if (m_planning_group == "kuka_complete_arm")
        {
            printf("%6.2f   %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f  %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n", duration, m_trajectory->getPosition(duration)[0], m_trajectory->getPosition(duration)[1], m_trajectory->getPosition(duration)[2], m_trajectory->getPosition(duration)[3], m_trajectory->getPosition(duration)[4], m_trajectory->getPosition(duration)[5], m_trajectory->getPosition(duration)[6],
            m_trajectory->getVelocity(duration)[0], m_trajectory->getVelocity(duration)[1], m_trajectory->getVelocity(duration)[2],m_trajectory->getVelocity(duration)[3], m_trajectory->getVelocity(duration)[4], m_trajectory->getVelocity(duration)[5], m_trajectory->getVelocity(duration)[6]);
        }
        else if(m_planning_group == "omnirob_lbr_sdh")
        {
            printf("%6.2f   %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f  %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n", duration, m_trajectory->getPosition(duration)[0], m_trajectory->getPosition(duration)[1], m_trajectory->getPosition(duration)[2], m_trajectory->getPosition(duration)[3], m_trajectory->getPosition(duration)[4], m_trajectory->getPosition(duration)[5], m_trajectory->getPosition(duration)[6], m_trajectory->getPosition(duration)[7], m_trajectory->getPosition(duration)[8], m_trajectory->getPosition(duration)[9],
            m_trajectory->getVelocity(duration)[0], m_trajectory->getVelocity(duration)[1], m_trajectory->getVelocity(duration)[2],m_trajectory->getVelocity(duration)[3], m_trajectory->getVelocity(duration)[4], m_trajectory->getVelocity(duration)[5], m_trajectory->getVelocity(duration)[6], m_trajectory->getVelocity(duration)[7], m_trajectory->getVelocity(duration)[8], m_trajectory->getVelocity(duration)[9]);
        }
        else
            ROS_ERROR("Unknown Planning Group");

        */
        cout << "Trajectory generation completed successfully." << endl;

        //Return the resulting joint trajectory
        //return trajectory;
    }
    else {
        cout << "Trajectory generation failed." << endl;
    }
}


//Set initial Robot Pose (Start Config)
void MotionCommanderVREP::setInitialRobotPose(int &start_idx_manip_joints_conf_vector){

    ros::ServiceClient client_initialRobotPose=m_nh_.serviceClient<vrep_common::simRosSetRobotPose>("/vrep/simRosSetInitialRobotPose");
    vrep_common::simRosSetRobotPose robot_pose;
    //Index of first LBR Arm joint in config vector
    start_idx_manip_joints_conf_vector = 0;
    //Initial config for omnirob base
    if(m_planning_group == "omnirob_lbr_sdh")
    {
       robot_pose.request.position.push_back(m_joint_pos_trajectory[0][0]); //base x pos w.r.t global frame
       robot_pose.request.position.push_back(m_joint_pos_trajectory[0][1]); //base y pos w.r.t global frame
       robot_pose.request.position.push_back(m_base_rot_offset_rviz_vrep + m_joint_pos_trajectory[0][2]);//base z rot w.r.t global frame -> angle in [rad]
       //Set index of of first lbr joint in the "m_joint_pos_trajectory" array
       start_idx_manip_joints_conf_vector = 3;
    }
    //Initial configuration for LBR Joints -> angle in [rad]
    robot_pose.request.position.push_back(m_dir_rotation_axes_rviz_vrep[0] * m_joint_pos_trajectory[0][start_idx_manip_joints_conf_vector]);
    robot_pose.request.position.push_back(m_dir_rotation_axes_rviz_vrep[1] * m_joint_pos_trajectory[0][start_idx_manip_joints_conf_vector+1]);
    robot_pose.request.position.push_back(m_dir_rotation_axes_rviz_vrep[2] * m_joint_pos_trajectory[0][start_idx_manip_joints_conf_vector+2]);
    robot_pose.request.position.push_back(m_dir_rotation_axes_rviz_vrep[3] * m_joint_pos_trajectory[0][start_idx_manip_joints_conf_vector+3]);
    robot_pose.request.position.push_back(m_dir_rotation_axes_rviz_vrep[4] * m_joint_pos_trajectory[0][start_idx_manip_joints_conf_vector+4]);
    robot_pose.request.position.push_back(m_dir_rotation_axes_rviz_vrep[5] * m_joint_pos_trajectory[0][start_idx_manip_joints_conf_vector+5]);
    robot_pose.request.position.push_back(m_dir_rotation_axes_rviz_vrep[6] * m_joint_pos_trajectory[0][start_idx_manip_joints_conf_vector+6]);

    //Set initial pose for robot
    client_initialRobotPose.call(robot_pose);

    cout<<"Set initial robot pose..."<<endl;

    // handle ROS messages:
    ros::spinOnce();

    // sleep a bit:
    usleep(2000);
}


//Set initial Control Modes for Joints
void MotionCommanderVREP::setInitialControlModes(int &start_idx_manip_joints_motor_handle_vector){

    //Set Control Modes
    //vector<int> control_modes_values; //position control enabled -> 0 no (i.e. velocity control is used), 1 yes
    start_idx_manip_joints_motor_handle_vector = 0;
    //Omnirob wheels (velocity mode = 0)
    if(m_planning_group == "omnirob_lbr_sdh" || m_planning_group == "omnirob")
    {
        m_joint_control_modes[0] = 0;
        m_joint_control_modes[1] = 0;
        m_joint_control_modes[2] = 0;
        m_joint_control_modes[3] = 0;
        start_idx_manip_joints_motor_handle_vector = 4;

    }
    if(m_planning_group == "omnirob_lbr_sdh" || m_planning_group == "kuka_complete_arm")
    {
        //LBR Joints (velocity mode = 0 or position mode = 1)
        m_joint_control_modes[start_idx_manip_joints_motor_handle_vector] = 0;
        m_joint_control_modes[start_idx_manip_joints_motor_handle_vector+1] = 0;
        m_joint_control_modes[start_idx_manip_joints_motor_handle_vector+2] = 0;
        m_joint_control_modes[start_idx_manip_joints_motor_handle_vector+3] = 0;
        m_joint_control_modes[start_idx_manip_joints_motor_handle_vector+4] = 0;
        m_joint_control_modes[start_idx_manip_joints_motor_handle_vector+5] = 0;
        m_joint_control_modes[start_idx_manip_joints_motor_handle_vector+6] = 0;

        //Schunk Hand Joints 1 (velocity mode = 0)
        m_joint_control_modes[start_idx_manip_joints_motor_handle_vector+7] = 1;
        m_joint_control_modes[start_idx_manip_joints_motor_handle_vector+8] = 1;
        m_joint_control_modes[start_idx_manip_joints_motor_handle_vector+9] = 1;
        //Schunk Hand Joints 2 (position mode = 1)
        m_joint_control_modes[start_idx_manip_joints_motor_handle_vector+10] = 1;
        m_joint_control_modes[start_idx_manip_joints_motor_handle_vector+11] = 1;
        m_joint_control_modes[start_idx_manip_joints_motor_handle_vector+12] = 1;
    }

    //Set the joint control modes
    setJointControlModes(m_joint_control_modes);

    // handle ROS messages:
    ros::spinOnce();
}


//Get Offset between current base pose and desired trajectory base point
void MotionCommanderVREP::getTrajectoryOffsetBase(double &delta_x, double &delta_y, double &delta_theta, double &curr_theta_global, double trajectory_time){

    //Setup for current robot pose request
    vrep_common::simRosGetObjectPose curr_robot_pose;
    curr_robot_pose.request.handle = m_motor_handles[m_motor_handles.size()-1];
    curr_robot_pose.request.relativeToObjectHandle = -1; //get the pose relative to the global frame

    //Get current robot pose
    client_base_pose.call(curr_robot_pose);

    //Get Current base position w.r.t global frame from VRep
    double curr_x_pos_global = curr_robot_pose.response.pose.pose.position.x;
    double curr_y_pos_global = curr_robot_pose.response.pose.pose.position.y;


    vector<double> quat_vector(4);
    quat_vector[0] = curr_robot_pose.response.pose.pose.orientation.w;
    quat_vector[1] = curr_robot_pose.response.pose.pose.orientation.x;
    quat_vector[2] = curr_robot_pose.response.pose.pose.orientation.y;
    quat_vector[3] = curr_robot_pose.response.pose.pose.orientation.z;

    cout<<quat_vector[0]<<" "<<quat_vector[1]<<" "<<quat_vector[2]<<" "<<quat_vector[3]<<endl;
    //Compute RPY angles
    vector<double> rpy_angles = three_axis_rot( -2.*(quat_vector[2]*quat_vector[3] - quat_vector[0]*quat_vector[1]), \
                                        quat_vector[0]*quat_vector[0] - quat_vector[1]*quat_vector[1] - quat_vector[2]*quat_vector[2] + quat_vector[3]*quat_vector[3], \
                                        2*(quat_vector[1]*quat_vector[3] + quat_vector[0]*quat_vector[2]), \
                                       -2*(quat_vector[1]*quat_vector[2] - quat_vector[0]*quat_vector[3]), \
                                        quat_vector[0]*quat_vector[0] + quat_vector[1]*quat_vector[1] - quat_vector[2]*quat_vector[2] - quat_vector[3]*quat_vector[3]);

    //Get current base rotation (yaw angle)
    curr_theta_global = rpy_angles[2];

    //cout<<"x curr: "<<curr_robot_pose.response.pose.pose.position.x<<endl;
    //cout<<"y curr: "<<curr_robot_pose.response.pose.pose.position.y<<endl;

    //Desired base position in the global frame (given by the motion planning path)
    double des_x_pos_global = m_trajectory->getPosition(trajectory_time)[0];
    double des_y_pos_global = m_trajectory->getPosition(trajectory_time)[1];
    double des_theta_global = m_base_rot_offset_rviz_vrep + m_trajectory->getPosition(trajectory_time)[2]; //Account for different initial base rotation in RViz and Vrep

    //Convert "des_theta_global" to range [-pi,pi] (before it is within the range [0,2pi])
    double angle_over_pi = fmod(des_theta_global ,M_PI);
    if(M_PI < fabs(des_theta_global))
        des_theta_global = angle_over_pi < 0 ? M_PI + angle_over_pi : -M_PI + angle_over_pi;

    //cout<<curr_theta_global<<endl;
    //cout<<des_theta_global<<endl;

    //Distance between current and desired robot position
    delta_x = des_x_pos_global - curr_x_pos_global;
    delta_y = des_y_pos_global - curr_y_pos_global;

    //Rotation between current and desired robot position (already in local frame)
    delta_theta = des_theta_global - curr_theta_global;

    //To get the smallest angle between "des_theta_global" and "curr_theta_global"
    delta_theta += (delta_theta>M_PI) ? -2*M_PI : (delta_theta<-M_PI) ? 2*M_PI : 0;
    //Verbose variant of the above expression
    //  -> delta_theta = des_theta_global - curr_theta_global
    //  -> delta_theta -= 360 if delta_theta > 180
    //  -> delta_theta += 360 if delta_theta < -180
}


//Get Offset between current manipulator config and desired trajectory point manipulator config
vector<double> MotionCommanderVREP::getTrajectoryOffsetManipulator(int start_idx_manip_joints_motor_handle_vector, int start_idx_manip_joints_conf_vector, double trajectory_time){

    vrep_common::simRosGetJointState joint_pos;
    vector<double> curr_joint_config(7);
    vector<double> des_joint_config(7);
    vector<double> delta_joint_configs(7);
    //vector<double> joint_velocities(7);

    for(int joint_handle = 0; joint_handle < 7 ; joint_handle++)
    {
        //Get motor handle of current joint (motor handles of manipulator start at "start_idx_manip_joints_motor_handle_vector" + there are 7 joints)
        joint_pos.request.handle = m_motor_handles[joint_handle + start_idx_manip_joints_motor_handle_vector] ;

        //Get current joint config from Vrep
        client_joint_config.call(joint_pos);

        //curr_joint_config[joint_handle] = joint_pos.response.state.position[joint_handle];
        curr_joint_config[joint_handle] = joint_pos.response.state.position[0];

        //Get desired joint config from trajectory (Account for different joint rotation directions between RViz and Vrep)
        des_joint_config[joint_handle] = m_dir_rotation_axes_rviz_vrep[joint_handle] * m_trajectory->getPosition(trajectory_time)[joint_handle + start_idx_manip_joints_conf_vector];

        //Rotation between current and desired joint position
        delta_joint_configs[joint_handle] = des_joint_config[joint_handle] - curr_joint_config[joint_handle];

        //To get the smallest angle between "des_joint_config" and "curr_joint_config"
        delta_joint_configs[joint_handle] += (delta_joint_configs[joint_handle]>M_PI) ? -2*M_PI : (delta_joint_configs[joint_handle]<-M_PI) ? 2*M_PI : 0;

        //Rotate joint towards des_joint_config
        //joint_velocities[joint_handle] = m_angular_vel_lbr_gain * delta_joint_configs[joint_handle];
    }

    return delta_joint_configs;
}



//Compute required Base Velocity to reach Trajectory Target Point in local Base Frame
void MotionCommanderVREP::computeBaseVelocityLocal(const double delta_x, const double delta_y, const double delta_theta, const double curr_theta_global, double &x_vel_local, double &y_vel_local, double &theta_vel_local){

    //Position vector length ang angle w.r.t global frame
    double pos_vec_length = sqrt(delta_x*delta_x+delta_y*delta_y);
    double pos_angle_global = atan2(delta_y,delta_x);

    //Express trajectory distance w.r.t robot base frame
    double x_pos_local = pos_vec_length * cos(pos_angle_global - curr_theta_global); //des_theta_global
    double y_pos_local = pos_vec_length * sin(pos_angle_global - curr_theta_global);
    //double theta_pos_local = des_vel_theta_global; //base orientation already expressed in base frame

    //cout<<"x_pos_local: "<<x_pos_local<<endl;
    //cout<<"y_pos_local: "<<y_pos_local<<endl;

    //int z;
    //cin>>z;

    //Compute unit vector in the direction of (x_pos_local,y_pos_local)
    double x_pos_direction =  x_pos_local/pos_vec_length;
    double y_pos_direction =  y_pos_local/pos_vec_length;

    //Move base into this direction
    x_vel_local = m_linear_vel_base_gain * x_pos_local;
    y_vel_local = m_linear_vel_base_gain * y_pos_local;
    //Rotate base into this direction
    theta_vel_local = m_angular_vel_base_gain * delta_theta;
}

//Compute and execute required wheel speeds given the linear and angular base velocity
void MotionCommanderVREP::setWheelSpeedsOmnirobBase(const double x_vel_local, const double y_vel_local, const double theta_vel_local){

    //Compute wheel speeds(r = wheel radius w = track base and l = wheelbase)
    float wheel_radius = 0.12; //in m
    float track_width = 0.48;
    float wheel_base = 0.695;

    float desiredRLWheelMotor;
    float desiredRRWheelMotor;
    float desiredFLWheelMotor;
    float desiredFRWheelMotor;

    //Wheel speed equations (adapted form Christoph)
    desiredFRWheelMotor= (1.0/wheel_radius) * (-x_vel_local + y_vel_local - 0.5 * (track_width+wheel_base)*theta_vel_local);
    desiredFLWheelMotor= (1.0/wheel_radius) * (-x_vel_local - y_vel_local - 0.5 * (track_width+wheel_base)*theta_vel_local);
    desiredRLWheelMotor= (1.0/wheel_radius) * (x_vel_local - y_vel_local - 0.5 * (track_width+wheel_base)*theta_vel_local);
    desiredRRWheelMotor= (1.0/wheel_radius) * (x_vel_local + y_vel_local - 0.5 * (track_width+wheel_base)*theta_vel_local);


    //+++ End: Convert linear and angular base speed into wheel speeds

    //Motor Speed Messages to be published
    vrep_common::JointSetStateData omnirob_wheels_motor_speeds;

    //Publish the motor speeds for the wheels of the Omnirob Base:
    omnirob_wheels_motor_speeds.handles.data.push_back(m_motor_handles[0]);
    omnirob_wheels_motor_speeds.handles.data.push_back(m_motor_handles[1]);
    omnirob_wheels_motor_speeds.handles.data.push_back(m_motor_handles[2]);
    omnirob_wheels_motor_speeds.handles.data.push_back(m_motor_handles[3]);

    //Modes (set Joint Dynamic Properties in V-Rep accordingly)
    // 0: sets the position
    // 1: sets the target position (when joint is dynamically enabled and in position control)
    // 2: sets the target velocity (when joint is dynamically enabled without position control, or when joint is in velocity mode)
    // 3: sets the maximum force/torque that the joint can exert
    omnirob_wheels_motor_speeds.setModes.data.push_back(2);
    omnirob_wheels_motor_speeds.setModes.data.push_back(2);
    omnirob_wheels_motor_speeds.setModes.data.push_back(2);
    omnirob_wheels_motor_speeds.setModes.data.push_back(2);

    omnirob_wheels_motor_speeds.values.data.push_back(desiredRLWheelMotor);
    omnirob_wheels_motor_speeds.values.data.push_back(desiredRRWheelMotor);
    omnirob_wheels_motor_speeds.values.data.push_back(desiredFLWheelMotor);
    omnirob_wheels_motor_speeds.values.data.push_back(desiredFRWheelMotor);

    //Publish wheel speeds
    m_omnirob_motor_speedPub.publish(omnirob_wheels_motor_speeds);
}


//ROBOTINO BASE:Compute and execute required wheel speeds given the linear and angular base velocity
void MotionCommanderVREP::setWheelSpeedsRobotinoBase(const double x_vel_local, const double y_vel_local, const double theta_vel_local){


    // +++++ FROM : http://doc.openrobotino.org/download/RobotinoAPI2/rec_robotino_api2/_omni_drive_model_8h-source.html

    //Projection matrix
    const double v0[2] = { -0.5 * sqrt( 3.0 ),  0.5 };
    const double v1[2] = {  0.0              , -1.0 };
    const double v2[2] = {  0.5 * sqrt( 3.0 ),  0.5 };

    //Radius of the Robotino Robot
    double _rb = 0.132;
    //... and some other Robotino params
    double _rw =  0.040;
    double _fctrl =  900.0;
    double _gear = 16.0;
    double _mer =  2000.0;

    //Scale omega with the radius of the robot
    double vOmegaScaled = _rb * (double)theta_vel_local ;

    //Convert from m/s to RPM
    const double k = 60.0 * _gear / ( 2.0 * M_PI * _rw );

    //Compute the desired velocity
    double motor_1 = ( v0[0] * (double)x_vel_local + v0[1] * (double)y_vel_local + vOmegaScaled ) * k ;
    double motor_2 = ( v1[0] * (double)x_vel_local + v1[1] * (double)y_vel_local + vOmegaScaled ) * k ;
    double motor_3 = ( v2[0] * (double)x_vel_local + v2[1] * (double)y_vel_local + vOmegaScaled ) * k ;

    // +++++


    //+++ End: Convert linear and angular base speed into wheel speeds

    //Motor Speed Messages to be published
    vrep_common::JointSetStateData robotino_wheels_motor_speeds;

    //Publish the motor speeds for the wheels of the Robotino Base:
    robotino_wheels_motor_speeds.handles.data.push_back(m_motor_handles[0]);
    robotino_wheels_motor_speeds.handles.data.push_back(m_motor_handles[1]);
    robotino_wheels_motor_speeds.handles.data.push_back(m_motor_handles[2]);

    //Modes (set Joint Dynamic Properties in V-Rep accordingly)
    // 0: sets the position
    // 1: sets the target position (when joint is dynamically enabled and in position control)
    // 2: sets the target velocity (when joint is dynamically enabled without position control, or when joint is in velocity mode)
    // 3: sets the maximum force/torque that the joint can exert
    robotino_wheels_motor_speeds.setModes.data.push_back(2);
    robotino_wheels_motor_speeds.setModes.data.push_back(2);
    robotino_wheels_motor_speeds.setModes.data.push_back(2);

    robotino_wheels_motor_speeds.values.data.push_back(motor_1);
    robotino_wheels_motor_speeds.values.data.push_back(motor_2);
    robotino_wheels_motor_speeds.values.data.push_back(motor_3);

    //Publish wheel speeds
    m_robotino_motor_speedPub.publish(robotino_wheels_motor_speeds);
}


//Compute Joint Velocities for Manipulator given the offset to the current trajectory point
vector<double> MotionCommanderVREP::computeJointVelocities(vector<double> delta_joint_configs){

    vector<double> joint_velocities(7);

    for(int joint_handle = 0; joint_handle < delta_joint_configs.size() ; joint_handle++)
    {
        //Rotate joint towards des_joint_config
        joint_velocities[joint_handle] = m_angular_vel_lbr_gain * delta_joint_configs[joint_handle];
    }

    return joint_velocities;
}


//Compute and execute required joint velocities
void MotionCommanderVREP::setJointSpeedsManipulator(int start_idx_manip_joints_motor_handle_vector, vector<double> joint_velocities){

    //Kuka LBR Manipulator
    float desired_LBR_joint1;
    float desired_LBR_joint2;
    float desired_LBR_joint3;
    float desired_LBR_joint4;
    float desired_LBR_joint5;
    float desired_LBR_joint6;
    float desired_LBR_joint7;

    desired_LBR_joint1 = joint_velocities[0];
    desired_LBR_joint2 = joint_velocities[1];
    desired_LBR_joint3 = joint_velocities[2];
    desired_LBR_joint4 = joint_velocities[3];
    desired_LBR_joint5 = joint_velocities[4];
    desired_LBR_joint6 = joint_velocities[5];
    desired_LBR_joint7 = joint_velocities[6];

    vrep_common::JointSetStateData lbr_motor_speeds;

    //Publish the motor speeds for the joints of the Kuka LBR:
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[start_idx_manip_joints_motor_handle_vector]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[start_idx_manip_joints_motor_handle_vector+1]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[start_idx_manip_joints_motor_handle_vector+2]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[start_idx_manip_joints_motor_handle_vector+3]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[start_idx_manip_joints_motor_handle_vector+4]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[start_idx_manip_joints_motor_handle_vector+5]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[start_idx_manip_joints_motor_handle_vector+6]);


    //Modes (set Joint Dynamic Properties in V-Rep accordingly)
    // 0: sets the position
    // 1: sets the target position (when joint is dynamically enabled and in position control)
    // 2: sets the target velocity (when joint is dynamically enabled without position control, or when joint is in velocity mode)
    // 3: sets the maximum force/torque that the joint can exert
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);

    lbr_motor_speeds.values.data.push_back(desired_LBR_joint1);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint2);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint3);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint4);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint5);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint6);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint7);

    m_lbr_motor_speedPub.publish(lbr_motor_speeds);
}



bool MotionCommanderVREP::stopManipulatorMotion(int start_idx_manip_joints_motor_handle_vector){

    vrep_common::JointSetStateData lbr_motor_speeds;

    //Kuka LBR Manipulator
    float desired_LBR_joint1;
    float desired_LBR_joint2;
    float desired_LBR_joint3;
    float desired_LBR_joint4;
    float desired_LBR_joint5;
    float desired_LBR_joint6;
    float desired_LBR_joint7;

    desired_LBR_joint1 = 0.0;
    desired_LBR_joint2 = 0.0;
    desired_LBR_joint3 = 0.0;
    desired_LBR_joint4 = 0.0;
    desired_LBR_joint5 = 0.0;
    desired_LBR_joint6 = 0.0;
    desired_LBR_joint7 = 0.0;


    //Publish the motor speeds for the joints of the Kuka LBR:
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[start_idx_manip_joints_motor_handle_vector]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[start_idx_manip_joints_motor_handle_vector+1]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[start_idx_manip_joints_motor_handle_vector+2]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[start_idx_manip_joints_motor_handle_vector+3]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[start_idx_manip_joints_motor_handle_vector+4]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[start_idx_manip_joints_motor_handle_vector+5]);
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[start_idx_manip_joints_motor_handle_vector+6]);


    //Modes (set Joint Dynamic Properties in V-Rep accordingly)
    // 0: sets the position
    // 1: sets the target position (when joint is dynamically enabled and in position control)
    // 2: sets the target velocity (when joint is dynamically enabled without position control, or when joint is in velocity mode)
    // 3: sets the maximum force/torque that the joint can exert
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);
    lbr_motor_speeds.setModes.data.push_back(2);

    lbr_motor_speeds.values.data.push_back(desired_LBR_joint1);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint2);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint3);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint4);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint5);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint6);
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint7);

    //Publish motor speeds
    m_lbr_motor_speedPub.publish(lbr_motor_speeds);


    // handle ROS messages:
    ros::spinOnce();

    // sleep a bit:
    usleep(20); //20ms -> 50Hz loop

    cout<<"Manipulator motion stopped!"<<endl;

    return true;

}


bool MotionCommanderVREP::stopManipulatorJointMotion(int start_idx_manip_joints_motor_handle_vector, int joint_num)
{
    vrep_common::JointSetStateData lbr_motor_speeds;

    //Set desired joint velocity to zero
    float desired_LBR_joint = 0.0;

    //Publish the motor speeds for the joints of the Kuka LBR:
    lbr_motor_speeds.handles.data.push_back(m_motor_handles[start_idx_manip_joints_motor_handle_vector+joint_num]);

    //Modes (set Joint Dynamic Properties in V-Rep accordingly)
    // 0: sets the position
    // 1: sets the target position (when joint is dynamically enabled and in position control)
    // 2: sets the target velocity (when joint is dynamically enabled without position control, or when joint is in velocity mode)
    // 3: sets the maximum force/torque that the joint can exert
    lbr_motor_speeds.setModes.data.push_back(2);

    //Set speed in message
    lbr_motor_speeds.values.data.push_back(desired_LBR_joint);

    //Publish motor speeds
    m_lbr_motor_speedPub.publish(lbr_motor_speeds);

    // handle ROS messages:
    ros::spinOnce();

    // sleep a bit:
    usleep(20); //20ms -> 50Hz loop

    cout<<"Motion of joint: "<<joint_num<<" stopped!"<<endl;

    return true;
}

bool MotionCommanderVREP::stopBaseMotion(){

    //If Trajectory execution includes omnirob base
    if(m_planning_group == "omnirob_lbr_sdh")
    {
        float desiredRLWheelMotor;
        float desiredRRWheelMotor;
        float desiredFLWheelMotor;
        float desiredFRWheelMotor;

        desiredFRWheelMotor= 0.0;
        desiredFLWheelMotor= 0.0;
        desiredRLWheelMotor= 0.0;
        desiredRRWheelMotor= 0.0;


        //+++ End: Convert linear and angular base speed into wheel speeds

        //Motor Speed Messages to be published
        vrep_common::JointSetStateData omnirob_wheels_motor_speeds;

        //Publish the motor speeds for the wheels of the Omnirob Base:
        omnirob_wheels_motor_speeds.handles.data.push_back(m_motor_handles[0]);
        omnirob_wheels_motor_speeds.handles.data.push_back(m_motor_handles[1]);
        omnirob_wheels_motor_speeds.handles.data.push_back(m_motor_handles[2]);
        omnirob_wheels_motor_speeds.handles.data.push_back(m_motor_handles[3]);

        //Modes (set Joint Dynamic Properties in V-Rep accordingly)
        // 0: sets the position
        // 1: sets the target position (when joint is dynamically enabled and in position control)
        // 2: sets the target velocity (when joint is dynamically enabled without position control, or when joint is in velocity mode)
        // 3: sets the maximum force/torque that the joint can exert
        omnirob_wheels_motor_speeds.setModes.data.push_back(2);
        omnirob_wheels_motor_speeds.setModes.data.push_back(2);
        omnirob_wheels_motor_speeds.setModes.data.push_back(2);
        omnirob_wheels_motor_speeds.setModes.data.push_back(2);

        omnirob_wheels_motor_speeds.values.data.push_back(desiredRLWheelMotor);
        omnirob_wheels_motor_speeds.values.data.push_back(desiredRRWheelMotor);
        omnirob_wheels_motor_speeds.values.data.push_back(desiredFLWheelMotor);
        omnirob_wheels_motor_speeds.values.data.push_back(desiredFRWheelMotor);

        //Publish wheel speeds
        m_omnirob_motor_speedPub.publish(omnirob_wheels_motor_speeds);

    }

    // handle ROS messages:
    ros::spinOnce();

    // sleep a bit:
    usleep(20); //20ms -> 50Hz loop

    cout<<"Base motion stopped!"<<endl;

    return true;
}


void MotionCommanderVREP::prepareGrasp()
{
    //Set first finger joint position
    float desired_sdh2_finger_joint_12 = -0.785;
    float desired_sdh2_finger_joint_13 = 0.0;

    //Set second finger joint position
    float desired_sdh2_finger_joint_22 = -1.57 + 0.785;
    float desired_sdh2_finger_joint_23 = 0.0;;

    //Set thumb joint position
    float desired_sdh2_thumb_joint_2 = -0.785;
    float desired_sdh2_thumb_joint_3 = 0.0;

    //Motor speed message for Schunk Hand
    vrep_common::JointSetStateData schunk_sdh2_motor_pos;

    //Motor handle start index for hand
    int motor_handle_start = 0;

    //Publish the motor speeds for the joints of the Schunk SDH2:
    //Note:
    //  - Schunk Hand Joints start with index 7 if only LBR Arm is considered
    //  - Schunk Hand Joints start with index 11 if Omnirob + LBR Arm is considered
    if (m_planning_group == "kuka_complete_arm")
    {
        motor_handle_start = 7;
    }
    else if(m_planning_group == "omnirob_lbr_sdh")
    {
        motor_handle_start = 11;
    }
    else
        ROS_ERROR("Unknown Planning Group");


    //Set Control Modes and Finger Joint positions
    for (int i = motor_handle_start ; i < motor_handle_start+6 ; i++)
    {
        //Set message motor handle
        schunk_sdh2_motor_pos.handles.data.push_back(m_motor_handles[i]);

        //Set Joint control modes (1 = position mode) for Vrep
        m_joint_control_modes[i] = 1;

        //Modes (set Joint Dynamic Properties in V-Rep accordingly)
        // 0: sets the position
        // 1: sets the target position (when joint is dynamically enabled and in position control)
        // 2: sets the target velocity (when joint is dynamically enabled without position control, or when joint is in velocity mode)
        // 3: sets the maximum force/torque that the joint can exert
        schunk_sdh2_motor_pos.setModes.data.push_back(1);

    }


    //Send Joint Control Modes to Vrep
    setJointControlModes(m_joint_control_modes);

    // handle ROS messages:
    ros::spinOnce();

    // sleep a bit:
    usleep(1000);

    schunk_sdh2_motor_pos.values.data.push_back(desired_sdh2_finger_joint_12);
    schunk_sdh2_motor_pos.values.data.push_back(desired_sdh2_finger_joint_22);
    schunk_sdh2_motor_pos.values.data.push_back(desired_sdh2_thumb_joint_2);
    schunk_sdh2_motor_pos.values.data.push_back(desired_sdh2_finger_joint_13);
    schunk_sdh2_motor_pos.values.data.push_back(desired_sdh2_finger_joint_23);
    schunk_sdh2_motor_pos.values.data.push_back(desired_sdh2_thumb_joint_3);

    //Publish initial motor speeds
    m_schunk_sdh2_motor_speedPub.publish(schunk_sdh2_motor_pos);

    // handle ROS messages:
    ros::spinOnce();

    // sleep a bit:
    usleep(1000);
}

//Functions to close and open the Schunk SDH2 Hand
void MotionCommanderVREP::closeHand()
{
    ROS_INFO("Closing SDH Hand Gripper");

    //Set first finger joint velocity
    float desired_sdh2_finger_joint_12 = m_gripper_opening_closing_speed;
    float desired_sdh2_finger_joint_13 = m_gripper_opening_closing_speed;

    //Set second finger joint position
    float desired_sdh2_finger_joint_22 = m_gripper_opening_closing_speed;
    float desired_sdh2_finger_joint_23 = m_gripper_opening_closing_speed;//-0.785;

    //Set thumb joint position
    float desired_sdh2_thumb_joint_2 = m_gripper_opening_closing_speed;
    float desired_sdh2_thumb_joint_3 = m_gripper_opening_closing_speed;


    //Motor speed message for Schunk Hand
    vrep_common::JointSetStateData schunk_sdh2_motor_speeds;

    //Motor handle start index for hand
    int motor_handle_start = 0;

    //Publish the motor speeds for the joints of the Schunk SDH2:
    //Note:
    //  - Schunk Hand Joints start with index 7 if only LBR Arm is considered
    //  - Schunk Hand Joints start with index 11 if Omnirob + LBR Arm is considered
    if (m_planning_group == "kuka_complete_arm")
    {
        motor_handle_start = 7;
    }
    else if(m_planning_group == "omnirob_lbr_sdh")
    {
        motor_handle_start = 11;
    }
    else
        ROS_ERROR("Unknown Planning Group");

    //Set Control Modes and Finger Joint positions
    for (int i = motor_handle_start ; i < motor_handle_start+6 ; i++)
    {
        //Set message motor handle
        schunk_sdh2_motor_speeds.handles.data.push_back(m_motor_handles[i]);

        //Set Joint control modes (0 = velocity mode) for Vrep
        m_joint_control_modes[i] = 0;

        //Modes (set Joint Dynamic Properties in V-Rep accordingly)
        // 0: sets the position
        // 1: sets the target position (when joint is dynamically enabled and in position control)
        // 2: sets the target velocity (when joint is dynamically enabled without position control, or when joint is in velocity mode)
        // 3: sets the maximum force/torque that the joint can exert
        schunk_sdh2_motor_speeds.setModes.data.push_back(2);

    }

    //Send Joint Control Modes to Vrep
    setJointControlModes(m_joint_control_modes);

    // handle ROS messages:
    ros::spinOnce();

    schunk_sdh2_motor_speeds.values.data.push_back(desired_sdh2_finger_joint_12);
    schunk_sdh2_motor_speeds.values.data.push_back(desired_sdh2_finger_joint_22);
    schunk_sdh2_motor_speeds.values.data.push_back(desired_sdh2_thumb_joint_2);
    schunk_sdh2_motor_speeds.values.data.push_back(desired_sdh2_finger_joint_13);
    schunk_sdh2_motor_speeds.values.data.push_back(desired_sdh2_finger_joint_23);
    schunk_sdh2_motor_speeds.values.data.push_back(desired_sdh2_thumb_joint_3);

    //Publish initial motor speeds
    m_schunk_sdh2_motor_speedPub.publish(schunk_sdh2_motor_speeds);

    // handle ROS messages:
    ros::spinOnce();

    // sleep a bit:
    usleep(1000);

}

void MotionCommanderVREP::lockHand()
{

    //Motor speed message for Schunk Hand
    vrep_common::JointSetStateData schunk_sdh2_motor_pos;

    //Service for getting current finger joint position
    vrep_common::simRosGetJointState finger_joint_pos;

    //Motor handle start index for hand
    int motor_handle_start = 0;

    //Publish the motor speeds for the joints of the Schunk SDH2:
    //Note:
    //  - Schunk Hand Joints start with index 7 if only LBR Arm is considered
    //  - Schunk Hand Joints start with index 11 if Omnirob + LBR Arm is considered
    if (m_planning_group == "kuka_complete_arm")
    {
        motor_handle_start = 7;
    }
    else if(m_planning_group == "omnirob_lbr_sdh")
    {
        motor_handle_start = 11;
    }
    else
        ROS_ERROR("Unknown Planning Group");


    //Set Control Modes and Finger Joint positions
    for (int i = motor_handle_start ; i < motor_handle_start+6 ; i++)
    {
        //Set message motor handle
        schunk_sdh2_motor_pos.handles.data.push_back(m_motor_handles[i]);

        //Modes (set Joint Dynamic Properties in V-Rep accordingly)
        // 0: sets the position
        // 1: sets the target position (when joint is dynamically enabled and in position control)
        // 2: sets the target velocity (when joint is dynamically enabled without position control, or when joint is in velocity mode)
        // 3: sets the maximum force/torque that the joint can exert
        schunk_sdh2_motor_pos.setModes.data.push_back(1);

        //Set motor handle for current finger joint
        finger_joint_pos.request.handle = m_motor_handles[i];

        //Get current finger joint position from Vrep
        client_joint_config.call(finger_joint_pos);

        //Set desired position to current position
        schunk_sdh2_motor_pos.values.data.push_back(finger_joint_pos.response.state.position[0]);

        //Set Joint control modes (1 = position mode) for Vrep
        m_joint_control_modes[i] = 1;

    }


    //Publish initial motor speeds
    m_schunk_sdh2_motor_speedPub.publish(schunk_sdh2_motor_pos);

    // handle ROS messages:
    ros::spinOnce();

    // sleep a bit:
    usleep(1000);

    //Send Joint Control Modes to Vrep
    setJointControlModes(m_joint_control_modes);

    // handle ROS messages:
    ros::spinOnce();
}


void MotionCommanderVREP::openHand()
{
    //Set lower gripper joints velocity
    float desired_sdh2_finger_joint_12 = -m_gripper_opening_closing_speed ;
    float desired_sdh2_finger_joint_22 = -m_gripper_opening_closing_speed;
    float desired_sdh2_thumb_joint_2 = -m_gripper_opening_closing_speed;
    //Set upper gripper joints position
    float desired_sdh2_finger_joint_13 = 0.0;
    float desired_sdh2_finger_joint_23 = 0.0;
    float desired_sdh2_thumb_joint_3 = 0.0;

    //Motor speed message for Schunk Hand
    vrep_common::JointSetStateData schunk_sdh2_motor_speeds;

    //Motor handle start index for hand
    int motor_handle_start = 0;

    //Publish the motor speeds for the joints of the Schunk SDH2:
    //Note:
    //  - Schunk Hand Joints start with index 7 if only LBR Arm is considered
    //  - Schunk Hand Joints start with index 11 if Omnirob + LBR Arm is considered
    if (m_planning_group == "kuka_complete_arm")
    {
        motor_handle_start = 7;
    }
    else if(m_planning_group == "omnirob_lbr_sdh")
    {
        motor_handle_start = 11;
    }
    else
        ROS_ERROR("Unknown Planning Group");


    //Set Control Modes and Finger Joint positions
    for (int i = motor_handle_start ; i < motor_handle_start+6 ; i++)
    {
        //Set message motor handle
        schunk_sdh2_motor_speeds.handles.data.push_back(m_motor_handles[i]);
    }

    //Set Joint control modes (0 = velocity mode, 1 = position mode) for Vrep
    m_joint_control_modes[motor_handle_start] = 0;
    m_joint_control_modes[motor_handle_start+1] = 0;
    m_joint_control_modes[motor_handle_start+2] = 0;
    m_joint_control_modes[motor_handle_start+3] = 1;
    m_joint_control_modes[motor_handle_start+4] = 1;
    m_joint_control_modes[motor_handle_start+5] = 1;

    //Modes (set Joint Dynamic Properties in V-Rep accordingly)
    // 0: sets the position
    // 1: sets the target position (when joint is dynamically enabled and in position control)
    // 2: sets the target velocity (when joint is dynamically enabled without position control, or when joint is in velocity mode)
    // 3: sets the maximum force/torque that the joint can exert
    schunk_sdh2_motor_speeds.setModes.data.push_back(2);
    schunk_sdh2_motor_speeds.setModes.data.push_back(2);
    schunk_sdh2_motor_speeds.setModes.data.push_back(2);
    schunk_sdh2_motor_speeds.setModes.data.push_back(1);
    schunk_sdh2_motor_speeds.setModes.data.push_back(1);
    schunk_sdh2_motor_speeds.setModes.data.push_back(1);


    schunk_sdh2_motor_speeds.values.data.push_back(desired_sdh2_finger_joint_12);
    schunk_sdh2_motor_speeds.values.data.push_back(desired_sdh2_finger_joint_22);
    schunk_sdh2_motor_speeds.values.data.push_back(desired_sdh2_thumb_joint_2);
    schunk_sdh2_motor_speeds.values.data.push_back(desired_sdh2_finger_joint_13);
    schunk_sdh2_motor_speeds.values.data.push_back(desired_sdh2_finger_joint_23);
    schunk_sdh2_motor_speeds.values.data.push_back(desired_sdh2_thumb_joint_3);

    //Publish initial motor speeds
    m_schunk_sdh2_motor_speedPub.publish(schunk_sdh2_motor_speeds);
}



//Three axis rotation
vector<double> MotionCommanderVREP::three_axis_rot(double r11, double r12, double r21, double r31, double r32)
{
    vector<double> rot(3);

    //find angles for rotations about X, Y, and Z axes
    rot[0] = atan2( r11, r12 ); //X
    rot[1] = asin( r21 );       //Y
    rot[2] = atan2( r31, r32 ); //Z

    return rot;
}




} //end of namespace


#include <motion_trajectory_execution/trajectory_execution_rviz.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group.h>

namespace trajectory_execution{

//Default Constructor
MotionCommanderRVIZ::MotionCommanderRVIZ()
{

}

//Constructor with planning group
MotionCommanderRVIZ::MotionCommanderRVIZ(string planning_group)
{
    //Get namespaces of robot
    m_nh.param("ns_prefix_robot", m_ns_prefix_robot, std::string(""));

    //Get name of robot_description parameter
    string robot_description_robot;
    m_nh.param("robot_description_robot", robot_description_robot, std::string("robot_description"));

    //Check the planning frame (from virtual joint in srdf)
    m_planning_frame =  getPlanningFrameFromSRDF(robot_description_robot);

    //Planning Group
    m_planning_group = planning_group;

    //Create Robot model
    m_KDLRobotModel = boost::shared_ptr<kuka_motion_controller::KDLRobotModel>(new kuka_motion_controller::KDLRobotModel(robot_description_robot, m_ns_prefix_robot+"planning_scene", m_ns_prefix_robot+"endeffector_trajectory", planning_group));

    //Get Joint Names for Robot
    m_joint_names = m_KDLRobotModel->getJointNames();

    //Get number of Joints
    m_num_joints = m_KDLRobotModel->getNumJoints();
    m_num_joints_revolute = m_KDLRobotModel->getNumRevoluteJoints();
    m_num_joints_prismatic = m_KDLRobotModel->getNumPrismaticJoints();

    //Create planning scene monitor
    //psm_ = boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    //Planning Scene Publisher
    //scene_pub_ = m_nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 10);
    //ros::Duration(0.5).sleep();


    //MoveIt Joint Trajectory Publisher
    m_pub_traj = m_nh.advertise<moveit_msgs::DisplayTrajectory>(m_ns_prefix_robot+"display_motion_plan", 1);

    //Clear joint trajectory
    m_joint_trajectory.clear();
    //Clear ee trajectory
    m_ee_trajectory.clear();

    //Get Planning Environment Size
    //m_planning_world = boost::shared_ptr<planning_world::PlanningWorldBuilder>(new planning_world::PlanningWorldBuilder(robot_description_robot, planning_group,m_ns_prefix_robot));
    m_planning_world = boost::shared_ptr<planning_world::PlanningWorldBuilder>(new planning_world::PlanningWorldBuilder(m_KDLRobotModel, m_ns_prefix_robot));

    //Store Planning Scene Name
    m_planning_scene_name = "none";
}


//Constructor with planning group and planning scene
MotionCommanderRVIZ::MotionCommanderRVIZ(string planning_group, string planning_scene)
{

    //Planning Group
    m_planning_group = planning_group;

    //Get namespaces of robot
    m_nh.param("ns_prefix_robot", m_ns_prefix_robot, std::string(""));

    //Get name of robot_description parameter
    string robot_description_robot;
    m_nh.param("robot_description_robot", robot_description_robot, std::string("robot_description"));

    //Check the planning frame (from virtual joint in srdf)
    m_planning_frame =  getPlanningFrameFromSRDF(robot_description_robot);

    //Create Robot model
    m_KDLRobotModel = boost::shared_ptr<kuka_motion_controller::KDLRobotModel>(new kuka_motion_controller::KDLRobotModel(robot_description_robot, m_ns_prefix_robot+"planning_scene", m_ns_prefix_robot+"endeffector_trajectory", planning_group));

    //Get Joint Names for Robot
    m_joint_names = m_KDLRobotModel->getJointNames();

    //Get number of Joints
    m_num_joints = m_KDLRobotModel->getNumJoints();
    m_num_joints_revolute = m_KDLRobotModel->getNumRevoluteJoints();
    m_num_joints_prismatic = m_KDLRobotModel->getNumPrismaticJoints();

    //Create planning scene monitor
    //psm_ = boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    //Planning Scene Publisher
    //scene_pub_ = m_nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 10);
    //ros::Duration(0.5).sleep();


    //MoveIt Joint Trajectory Publisher
    m_pub_traj = m_nh.advertise<moveit_msgs::DisplayTrajectory>(m_ns_prefix_robot + "display_motion_plan", 1);

    //Clear joint trajectory
    m_joint_trajectory.clear();
    //Clear ee trajectory
    m_ee_trajectory.clear();

    //Get Planning Environment Size
    //m_planning_world = boost::shared_ptr<planning_world::PlanningWorldBuilder>(new planning_world::PlanningWorldBuilder(robot_description_robot, planning_group));
    m_planning_world = boost::shared_ptr<planning_world::PlanningWorldBuilder>(new planning_world::PlanningWorldBuilder(m_KDLRobotModel, m_ns_prefix_robot));

    //Set the Planning Scene in the Planning World
    setPlanningScene(planning_scene);

}

//Constructor with robot_description parameter name and planning_scene topic name
MotionCommanderRVIZ::MotionCommanderRVIZ(string robot_description_name, string robot_ns, string planning_group, string planning_scene)
{

    //Set planning scene topic names
    string planning_scene_topic_rob = robot_ns + "planning_scene";

    //Set endeffector_trajectory topic names
    string endeffector_trajectory_topic_rob = robot_ns + "endeffector_trajectory";

    //Set endeffector_trajectory topic names
    string display_motion_plan_topic_rob = robot_ns + "display_motion_plan";

    //Check the planning frame (from virtual joint in srdf)
    m_planning_frame =  getPlanningFrameFromSRDF(robot_description_name);

    //Planning Group
    m_planning_group = planning_group;

    //Create Robot model
    m_KDLRobotModel = boost::shared_ptr<kuka_motion_controller::KDLRobotModel>(new kuka_motion_controller::KDLRobotModel(robot_description_name, planning_scene_topic_rob, endeffector_trajectory_topic_rob, planning_group));

    //Get Joint Names for Robot
    m_joint_names = m_KDLRobotModel->getJointNames();

    //Get number of Joints
    m_num_joints = m_KDLRobotModel->getNumJoints();
    m_num_joints_revolute = m_KDLRobotModel->getNumRevoluteJoints();
    m_num_joints_prismatic = m_KDLRobotModel->getNumPrismaticJoints();

    //Create planning scene monitor
    //psm_ = boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    //Planning Scene Publisher
    //scene_pub_ = m_nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 10);
    //ros::Duration(0.5).sleep();


    //MoveIt Joint Trajectory Publisher
    m_pub_traj = m_nh.advertise<moveit_msgs::DisplayTrajectory>(display_motion_plan_topic_rob, 1);

    //Clear joint trajectory
    m_joint_trajectory.clear();
    //Clear ee trajectory
    m_ee_trajectory.clear();

    //Get Planning Environment Size
    //m_planning_world = boost::shared_ptr<planning_world::PlanningWorldBuilder>(new planning_world::PlanningWorldBuilder(robot_description_name, planning_group));
    m_planning_world = boost::shared_ptr<planning_world::PlanningWorldBuilder>(new planning_world::PlanningWorldBuilder(m_KDLRobotModel, m_ns_prefix_robot));

    //Set the Planning Scene in the Planning World
    setPlanningScene(planning_scene);
}


//Destructor
MotionCommanderRVIZ::~MotionCommanderRVIZ()
{

    //Clear joint trajectory
    m_joint_trajectory.clear();

    //Clear ee trajectory
    m_ee_trajectory.clear();
}


//Reset planner data
void MotionCommanderRVIZ::reset_data()
{
    //Clear joint trajectory
    m_joint_trajectory.clear();

    //Clear ee trajectory
    m_ee_trajectory.clear();
}


//Get the planning frame from the SRDF description
string MotionCommanderRVIZ::getPlanningFrameFromSRDF(string robot_desciption_param)
{
    //Planning frame
    string planning_frame;

    //Check the planning frame (from virtual joint in srdf)
    boost::shared_ptr<srdf::Model> srdf_robot;
    boost::shared_ptr<urdf::ModelInterface> urdf_robot;

    //Get param content
    std::string content;
    if (!m_nh.getParam(robot_desciption_param, content))
    {
         ROS_ERROR("Robot model parameter empty '%s'?", robot_desciption_param.c_str());
         return "none";
    }

    urdf::Model* umodel = new urdf::Model();
    if (!umodel->initString(content))
    {
      ROS_ERROR("Unable to parse URDF from parameter '%s'", robot_desciption_param.c_str());
      return "none";
    }
    urdf_robot.reset(umodel);

    const std::string srdf_description(robot_desciption_param + "_semantic");
    std::string scontent;
    if (!m_nh.getParam(srdf_description, scontent))
    {
      ROS_ERROR("Robot semantic description not found. Did you forget to define or remap '%s'?", srdf_description.c_str());
     return "none";
   }

    srdf_robot.reset(new srdf::Model());
    if (!srdf_robot->initString(*urdf_robot, scontent))
    {
      ROS_ERROR("Unable to parse SRDF from parameter '%s'", srdf_description.c_str());
      srdf_robot.reset();
      return "none";
    }

    //Set planning frame class variable
    const std::vector< srdf::Model::VirtualJoint > &virtual_joint = srdf_robot->getVirtualJoints();
    planning_frame =  "/" + virtual_joint[0].parent_frame_;

    return planning_frame;
}

//Set the planning scene (given by input)
void MotionCommanderRVIZ::setPlanningScene(string planning_scene)
{

    double wall_length_ratio = 0.0;
    double wall_thickness = 0.0;


    //Store Planning Scene Name
    m_planning_scene_name = planning_scene;

    //Set Planning World

    //Set Environment Borders
    vector<double> env_size_x(2);
    env_size_x[0] = -10.0;
    env_size_x[1] = 10.0;
    vector<double> env_size_y(2);
    env_size_y[0] = -10.0;
    env_size_y[1] = 10.0;
    double env_size_z = 2.0;
    m_planning_world->insertEnvironmentBorders(env_size_x,env_size_y,env_size_z);

    if(m_planning_scene_name == "empty")
    {
        //Nothing to do
    }
    else if (m_planning_scene_name == "rack")
    {
        //Enter Environment Borders
        vector<double> env_size_x(2);
        env_size_x[0] = -5.0;
        env_size_x[1] = 5.0;
        vector<double> env_size_y(2);
        env_size_y[0] = -5.0;
        env_size_y[1] = 5.0;
        double env_size_z = 2.0;
        m_planning_world->insertEnvironmentBorders(env_size_x,env_size_y,env_size_z);


        //Insert Rack Scenario
        vector<double> rack_pos(3);
        rack_pos[0] = -4.0;
        rack_pos[1] = 0.0;
        rack_pos[2] = 0.0;
        vector<double> rack_dim(3);
        rack_dim[0] = 0.26;
        rack_dim[1] = 1.16;
        rack_dim[2] = 2.0;

        //Number of shelves and thickness of the shelves
        int num_shelves = 6;
        double shelf_thickness = 0.02;

        //Load Scene
        m_planning_world->insertRack(rack_pos,rack_dim,num_shelves,shelf_thickness);

    }
    else if (m_planning_scene_name == "door")
    {
        //Enter Environment Borders
        vector<double> env_size_x(2);
        env_size_x[0] = -5.0;
        env_size_x[1] = 5.0;
        vector<double> env_size_y(2);
        env_size_y[0] = -5.0;
        env_size_y[1] = 5.0;
        double env_size_z = 2.0;
        m_planning_world->insertEnvironmentBorders(env_size_x,env_size_y,env_size_z);

        //Manually set scene name
        m_planning_world->setSceneName("door");
    }
    else if(m_planning_scene_name == "walls")
    {
        vector<double> wall_pos(3);
        wall_pos[0] = 0.0;
        wall_pos[1] = 0.0;
        wall_pos[2] = 0.0;
        vector<double> wall_dim(3);
        wall_dim[0] = 0.2;
        wall_dim[1] = 6.0;
        wall_dim[2] = 0.6;
        m_planning_world->insertWall(wall_pos,wall_dim);
    }
    else if(m_planning_scene_name == "maze")
    {
        int num_walls = 4;
        wall_thickness = 0.2;
        wall_length_ratio = 0.5;
        m_planning_world->insertMaze(num_walls,wall_thickness,wall_length_ratio);
    }
    else if(m_planning_scene_name == "narrow_passage")
    {
        wall_thickness = 0.2;
        wall_length_ratio = 0.47;
        m_planning_world->insertNarrowPassage(wall_thickness, wall_length_ratio);
    }
    else if(m_planning_scene_name == "three_gates")
    {
        wall_thickness = 0.2;
        wall_length_ratio = 0.3;
        m_planning_world->insertThreeGatesPassage(wall_thickness, wall_length_ratio);
    }
    else if (m_planning_scene_name == "block")
    {
        //Insert Block Scenario
        vector<double> block_pos(3);
        block_pos[0] = 0.0;
        block_pos[1] = 0.0;
        block_pos[2] = 0.0;
        vector<double> block_dim(3);
        block_dim[0] = 5.0;
        block_dim[1] = 5.0;
        block_dim[2] = 2.0;
        m_planning_world->insertBlock(block_pos, block_dim);
    }
    else if (m_planning_scene_name == "parking")
    {
        //Enter Parking Slot
        double narrow_passage_offset = 0.7; //in [m]
        double wall_thickness = 1.0; //in [m]
        m_planning_world->insertParkingSlot(narrow_passage_offset,wall_thickness);
    }
    else if (m_planning_scene_name == "glass")
    {
        //Insert Glass Delivery Scenario
       double tunnel_height = 0.0;
       double tunnel_width = 0.0;
       double ceiling_height = 0.0;
       if(m_planning_group == "omnirob_lbr_sdh")
       {
           tunnel_height = 1.4;
           tunnel_width = 1.5; //Note : omnirob has width of 0.67m
           ceiling_height = 1.2;
       }
       if(m_planning_group == "robotino_robot")
       {
           tunnel_height = 0.6; //Note : robotino base has height of ??? m
           tunnel_width = 1.0; //Note : robotino has diameter of ??? m
           ceiling_height = 0.4;
       }

       m_planning_world->insertGlassDeliveryWorld(tunnel_width, tunnel_height, ceiling_height);
    }
    else if(m_planning_scene_name == "random_maze")
    {
        int num_walls = 5;
        m_planning_world->insertRandomMaze(num_walls);
    }
    else if(m_planning_scene_name == "tunnel")
    {
        double tunnel_height = 1.4;
        double tunnel_width = 1.1; //Note : omnirob has width of 0.67m
        wall_length_ratio = 0.2;
        m_planning_world->insertTunnelPassage(tunnel_width, tunnel_height, wall_length_ratio);

    }
    else if(m_planning_scene_name == "two_rooms")
    {
        wall_thickness = 0.2;
        double width_narrow_passage = 2.1; // in m (omnirob has width of 0.67m)
        m_planning_world->insertTwoRoomsOffice(wall_thickness,width_narrow_passage);

    }
    else if(m_planning_scene_name == "corridor")
    {
        double corridor_width = 2.0;
        wall_length_ratio = 0.2;
        m_planning_world->insertNarrowCorridor(corridor_width, wall_length_ratio);
    }
    else if(m_planning_scene_name == "neurobots_demo")
    {

    }
    else if(m_planning_scene_name == "none")
    {
       //ROS_ERROR("Planning World no known!");
    }
    else
    {}
}



//Load a Joint Trajectory
void MotionCommanderRVIZ::loadJointTrajectory(char* joint_trajectory_file)
{

    // --------------- Get the number of configurations in the file --------------------------
    string line;
    int num_vals_per_line = 0;

    //Input stream
    ifstream jT_file(joint_trajectory_file);


    //Check if file can be opened
    if (jT_file.is_open())
    {
        //Get first line
         std::getline (jT_file,line);

         //Get the number of values per line
         istringstream buf(line);
         istream_iterator<string> beg(buf), end;
         vector<string> substrings(beg, end); // done!

         //Get number of values per line
         num_vals_per_line = substrings.size();

    }
    else std::cout << "Unable to open file (loadJointTrajectory)";

    //Close file
    jT_file.close();



    //--------------- Fill the 2D Array --------------------------
    char * pEnd;
    int j = 0;
    char char_line[1000];

    //Open file
    jT_file.open(joint_trajectory_file);

    //Check if file is open
    if (jT_file.is_open())
    {
        while (jT_file.good() )
        {
         //read next line from file (either a configuration or eof)
         std::getline (jT_file,line);

         //Detect when end of file is reached
         if (jT_file.eof())
             break;

         //Transform string into char array
         strcpy(char_line,line.c_str());

         //Char pointer pointing on first element of char array
         char *tmp = char_line;


         //A configuration from the file
         vector<double> config;

         for (int i = 0 ; i < num_vals_per_line-1 ; ++i)
         {
           config.push_back(strtod(tmp,&pEnd));
           //cout<<strtod(tmp,&pEnd)<<" ";
           tmp = pEnd;
         }

         //Last joint value of configuration
         config.push_back(strtod(tmp,NULL));

         //cout<<strtod(tmp,NULL)<<endl;

         //Store config in trajectory
         m_joint_trajectory.push_back(config);

        }
        //Close file
        jT_file.close();
    }
    else std::cout << "Unable to open file (loadJointTrajectory)";


}


//Get Joint Trajectory
vector< vector<double> > MotionCommanderRVIZ::getJointTrajectory()
{
    if(m_joint_trajectory.size() == 0)
    {
        ROS_INFO("No joint trajectory available. Did you call loadJointTrajectory before?");
    }
    else
    {
       return  m_joint_trajectory;
    }
}


//Load a EE Trajectory
void MotionCommanderRVIZ::loadEETrajectory(char* ee_trajectory_file)
{
    // --------------- Get the number of poses in the file --------------------------
    string line;
    int num_vals_per_line = 0;

    //Input stream
    ifstream eeT_file(ee_trajectory_file);


    //Check if file can be opened
    if (eeT_file.is_open())
    {
        //Get first line
         std::getline (eeT_file,line);

         //Get the number of values per line
         istringstream buf(line);
         istream_iterator<string> beg(buf), end;
         vector<string> substrings(beg, end); // done!

         //Get number of values per line
         num_vals_per_line = substrings.size();

    }
    else std::cout << "Unable to open file (loadEETrajectory)";

    //Close file
    eeT_file.close();



    //--------------- Fill the 2D Array --------------------------
    char * pEnd;
    int j = 0;
    char char_line[1000];

    //Open file
    eeT_file.open(ee_trajectory_file);

    //Check if file is open
    if (eeT_file.is_open())
    {
        while (eeT_file.good() )
        {
         //read next line from file (either a pose or eof)
         std::getline (eeT_file,line);

         //Detect when end of file is reached
         if (eeT_file.eof())
             break;

         //Transform string into char array
         strcpy(char_line,line.c_str());

         //Char pointer pointing on first element of char array
         char *tmp = char_line;


         //A pose from the file
         vector<double> ee_pose;

         for (int i = 0 ; i < num_vals_per_line-1 ; ++i)
         {
           ee_pose.push_back(strtod(tmp,&pEnd));
           //cout<<strtod(tmp,&pEnd)<<" ";
           tmp = pEnd;
         }

         //Last parameter value of pose
         ee_pose.push_back(strtod(tmp,NULL));

         //cout<<strtod(tmp,NULL)<<endl;

         //Store pose in trajectory
         m_ee_trajectory.push_back(ee_pose);

        }
        //Close file
        eeT_file.close();
    }
    else std::cout << "Unable to open file (loadEETrajectory)";

}


//Get EE Trajectory
vector< vector<double> > MotionCommanderRVIZ::getEETrajectory()
{
    if(m_ee_trajectory.size() == 0)
    {
        ROS_INFO("No ee trajectory available. Did you call loadEETrajectory before?");
    }
    else
    {
       return  m_ee_trajectory;
    }
}


//Execute Trajectory
bool MotionCommanderRVIZ::execute(const string planner_type, const string run_id)
{
    //Read package path for planner output
    string trajectory_package_path = ros::package::getPath("planner_statistics");

    //Set path to the file that will store the planned joint trajectory
    string folder_path = trajectory_package_path + "/data/" + planner_type + "/motion_plan_joint_trajectory_run_" + run_id + ".txt";
    char *file_path_joint_trajectory = new char[folder_path.size() + 1];
    copy(folder_path.begin(), folder_path.end(), file_path_joint_trajectory);
    file_path_joint_trajectory[folder_path.size()] = '\0'; // don't forget the terminating 0
    //cout<<file_path_joint_trajectory<<endl;

    //Execute joint trajectory from file
    executeJointTrajectory(file_path_joint_trajectory);
    //executeJointTrajectory(file_path_joint_trajectory,true); //with elastic band

    cout<<"Finished trajectory execution"<<endl;

    //Reset data stored in trajectory vector
    reset_data();

    return true;
}


//Load Joint Trajectory from file and execute it
void MotionCommanderRVIZ::executeJointTrajectory(char* joint_trajectory_file)
{

    //------------- Get Trajectories from file -----------------------

    //Get Joint Trajecotry data
    loadJointTrajectory(joint_trajectory_file);

    //------------- Execute Trajectory -----------------------

    execute_trajectory();

}

//Load Joint Trajectory from file and execute it
void MotionCommanderRVIZ::executeJointTrajectory(char* joint_trajectory_file, char *ee_trajectory_file)
{

    //------------- Get Trajectories from file -----------------------

    //Get Joint Trajecotry data
    loadJointTrajectory(joint_trajectory_file);

    //Get EE Trajecotry data (only used for attaching objects to the endeffector)
    loadEETrajectory(ee_trajectory_file);

    //------------- Execute Trajectory -----------------------

    execute_trajectory();


}



//Execute Joint Trajectory given as input
void MotionCommanderRVIZ::executeJointTrajectory(vector< vector<double> > joint_trajectory, vector< vector<double> > ee_trajectory)
{
    //------------- Get Trajectories from input -----------------------

    //Get Joint Trajecotry data
    m_joint_trajectory = joint_trajectory;

    //Get EE Trajecotry data (only used for attaching objects to the endeffector)
    m_ee_trajectory = ee_trajectory;

    //------------- Execute Trajectory -----------------------

    execute_trajectory();
}


//Execute planned trajectory
void MotionCommanderRVIZ::execute_trajectory()
{
    //------------- Convert Path from planner into Joint Trajectory -----------------------

    // -> from Paper: "Time-Optimal Trajectory Generation for Path Following with Bounded Acceleration and Velocity"

    //Convert Trajectory into waypoints list
    list<Eigen::VectorXd> waypoints;
    for (int wp = 0; wp < m_joint_trajectory.size(); wp++)
    {
        Eigen::VectorXd waypoint(m_joint_trajectory[wp].size());
        for (int j = 0; j < m_joint_trajectory[wp].size(); j++)
        {
            waypoint[j] = m_joint_trajectory[wp][j];
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
//        maxAcceleration << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
//        maxVelocity << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
//    }
//    else if(m_planning_group == "omnirob_base" || m_planning_group == "pr2_base")
//    {
//        maxAcceleration.resize(3);
//        maxVelocity.resize(3);
//        //maxAcceleration << 1.0, 1.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 ;
//        maxAcceleration << 1.0, 1.0, 0.8;
//        maxVelocity << 1.0, 1.0, 1.0;
//    }
//    else if(m_planning_group == "pr2_base_arm")
//    {
//        maxAcceleration.resize(13);
//        maxVelocity.resize(13);
//        //maxAcceleration << 1.0, 1.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 ;
//        maxAcceleration << 1.0, 1.0, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8 ;
//        maxVelocity << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
//    }
//    else if(m_planning_group == "omnirob_lbr_sdh")
//    {
//        maxAcceleration.resize(10);
//        maxVelocity.resize(10);
//        //maxAcceleration << 1.0, 1.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 ;
//        maxAcceleration << 1.0, 1.0, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8 ;
//        maxVelocity << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
//    }
//    else if(m_planning_group == "robotino_robot")
//    {
//        maxAcceleration.resize(8);
//        maxVelocity.resize(8);
//        maxAcceleration << 1.0, 1.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
//        maxVelocity << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
//    }
//    else
//        ROS_ERROR("Unknown Planning Group");


    //Set maximum permitted path deviation for trajectory generation algorithm
    double max_path_deviation = 0.1;

    //Compute Trajectory (taking velocity and acceleration upper bounds into account)
    Trajectory gen_trajectory(Path(waypoints, max_path_deviation), maxVelocity, maxAcceleration);

    //Write Trajectory Data to File
    gen_trajectory.outputPhasePlaneTrajectory();
    //Init trajectory duration
    double duration = 0.0;
    if(gen_trajectory.isValid()) {
        duration = gen_trajectory.getDuration();
        cout << "Trajectory duration: " << duration << " s" << endl << endl;

        //for(double t = 0.0; t < duration; t += 0.2)
        //{
        //    printf("%6.2f  %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n", t, gen_trajectory.getVelocity(t)[0], gen_trajectory.getVelocity(t)[1], gen_trajectory.getVelocity(t)[2],gen_trajectory.getVelocity(t)[3], gen_trajectory.getVelocity(t)[4], gen_trajectory.getVelocity(t)[5], gen_trajectory.getVelocity(t)[6],gen_trajectory.getVelocity(t)[7], gen_trajectory.getVelocity(t)[8], gen_trajectory.getVelocity(t)[9]);
        //}

        cout << "Trajectory generation completed successfully." << endl;
    }
    else {
        cout << "Trajectory generation failed." << endl;
    }




    //---------------------------- Create Object Attachements --------------------------------

    //TODO:
    //  -> Get Attached Object information from planner or file

    //Dimension of attached_object
    vector<double> obj_pos(3);
    vector<double> obj_dim(3);
    //Attached Object
    moveit_msgs::AttachedCollisionObject attached_object;
    moveit_msgs::AttachedCollisionObject attached_object_modified;

    if(m_planning_scene_name == "empty" && m_planning_group == "omnirob_lbr_sdh")
    {
        //Nothing to do
    }
    else if(m_planning_scene_name == "glass" && m_planning_group == "omnirob_lbr_sdh")
    {
        obj_pos[0] = 9.0;   //x pos
        obj_pos[1] = -4.0;  //y pos
        obj_pos[2] = 0.9;   //z pos
        obj_dim[0] = 0.08;   //x-width dim
        obj_dim[1] = 0.08;   //y-length dim
        obj_dim[2] = 0.12;   //z-height dim
        attached_object = m_planning_world->insertManipulableGlass("glass", obj_pos, obj_dim);
        attached_object_modified = m_planning_world->attachObjecttoEndeffector(attached_object,m_ee_trajectory[0]);
    }
    else if(m_planning_scene_name == "block" && m_planning_group == "omnirob_lbr_sdh")
    {
        //No attached Objects
    }
    else if(m_planning_scene_name == "rack" && m_planning_group == "omnirob_lbr_sdh")
    {
        //Manipulable Object
        //Name, Position and Dimension of object
        string obj_name = "glass";
        vector<double> glass_dim(3);
        glass_dim[0] = 0.088;   //x-width dim
        glass_dim[1] = 0.088;   //y-length dim
        glass_dim[2] = 0.1233;  //z-height dim
        vector<double> glass_pos(3);
        glass_pos[0] = -4.0;   //x-width dim
        glass_pos[1] = 0.0;   //y-length dim
        glass_pos[2] = 1.12;   //z-height dim


        //Insert manipulable object into scene
        m_planning_world->insertManipulableGlass(obj_name, glass_pos, glass_dim);
    }
    else if((m_planning_scene_name == "parking" || m_planning_scene_name == "two_rooms") && m_planning_group == "omnirob_lbr_sdh")
    {
        obj_pos[0] = 9.0;   //x pos
        obj_pos[1] = -3.5;  //y pos
        obj_pos[2] = 0.0;   //z pos
        obj_dim[0] = 0.4;   //x-width dim
        obj_dim[1] = 0.5;   //y-length dim
        obj_dim[2] = 0.7;   //z-height dim
        attached_object = m_planning_world->insertManipulableCart("cart", obj_pos, obj_dim);
        attached_object_modified = m_planning_world->attachObjecttoEndeffector(attached_object,m_ee_trajectory[0]);
    }
    else if(m_planning_scene_name == "corridor" && m_planning_group == "omnirob_lbr_sdh")
    {
                obj_pos[0] = 8.0;   //x pos
                obj_pos[1] = -3.5;  //y pos
                obj_pos[2] = 0.0;   //z pos
                obj_dim[0] = 0.4;   //x-width dim
                obj_dim[1] = 0.5;   //y-length dim
                obj_dim[2] = 0.7;   //z-height dim
                attached_object = m_planning_world->insertManipulableCart("cart", obj_pos, obj_dim);
                attached_object_modified = m_planning_world->attachObjecttoEndeffector(attached_object,m_ee_trajectory[0]);

    }
    else
    {

    }



    //------------- Convert generated Trajectory into MoveIt Trajectory Message -----------------------

    //Moveit Trajectory Message
    moveit_msgs::DisplayTrajectory trajectory_rviz;


    //Set trajectory sampling step width
    double sampling_step = 0.1;



    //Generate MoveIt message if trajectory generation succeeded
    if(gen_trajectory.isValid())
    {
        //+++++++++++ Robot Start State +++++++++

        moveit_msgs::RobotState start_state;
        start_state.joint_state.name.resize(m_num_joints);
        start_state.joint_state.position.resize(m_num_joints);
        start_state.joint_state.velocity.resize(m_num_joints);

        //Set Frame ID in header
        if (m_planning_group == "kuka_complete_arm")
        {
            //Set Header frame ID
            start_state.joint_state.header.frame_id = m_ns_prefix_robot + "lbr_0_link";
            start_state.multi_dof_joint_state.header.frame_id = m_ns_prefix_robot + "lbr_0_link";
        }
        else if (m_planning_group == "pr2_arm")
        {
            //Set Header frame ID
            start_state.joint_state.header.frame_id = m_ns_prefix_robot + "torso_lift_link";
            start_state.multi_dof_joint_state.header.frame_id = m_ns_prefix_robot + "torso_lift_link";
        }
        else if(m_planning_group == "omnirob_lbr_sdh" || m_planning_group == "omnirob_base" || m_planning_group == "pr2_base" || m_planning_group == "pr2_base_arm" || m_planning_group == "robotino_robot")
        {
            //Set Header frame ID
            start_state.joint_state.header.frame_id = m_ns_prefix_robot + "base_link";
            start_state.multi_dof_joint_state.header.frame_id = m_ns_prefix_robot + "base_link";
        }
        else
            ROS_ERROR("Unknown Planning Group");


        //Flag indicating whether this scene is to be interpreted as a diff with respect to some other scene
        start_state.is_diff = true;

        // ------------- Set joint names and values of start state
        for (int j = 0 ; j < m_num_joints; j++)
        {
            //Set Joint Name
            start_state.joint_state.name[j] = m_joint_names[j];
            start_state.joint_state.position[j] =  gen_trajectory.getPosition(0)[j];
            start_state.joint_state.velocity[j] =  gen_trajectory.getVelocity(0)[j];
            //std::cout<<m_joint_names[j]<<" "<<gen_trajectory.getPosition(0)[j]<<std::endl;
        }


        //-------------- Convert start state in map frame

        //Get current pose of robot in the map frame
        tf::TransformListener listener;
        //Flag indicating whether transform between map and base_link is available
        bool transform_map_to_base_available = false;
        tf::StampedTransform transform_map_to_base;
        if(m_planning_frame == "/map")
        {
            try {
                listener.waitForTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), ros::Duration(10.0) );
                listener.lookupTransform("/map", m_ns_prefix_robot + "base_link", ros::Time(0), transform_map_to_base);
                transform_map_to_base_available = true;
            } catch (tf::TransformException ex) {
                //ROS_ERROR("%s",ex.what());
                transform_map_to_base_available = false;
            }
        }

        //If robot has a mobile base (2 prismatic and at least a revolute joint for base rotation)
        if(m_num_joints_prismatic == 2 && m_num_joints_revolute > 0)
        {
            if(transform_map_to_base_available)
            {
                //Transform base_link to sample
                tf::StampedTransform transform_base_to_wp;
                transform_base_to_wp.setOrigin(tf::Vector3(start_state.joint_state.position[0],start_state.joint_state.position[1], 0.0));
                transform_base_to_wp.setRotation(tf::createQuaternionFromYaw(start_state.joint_state.position[2]));

                //Transform map frame to sample
                tf::StampedTransform transform_map_to_wp;
                transform_map_to_wp.mult(transform_map_to_base,transform_base_to_wp);

                //Sample in map frame (as vector)
                vector<double> map_to_wp_conf(3);
                tf::Vector3 map_to_wp_trans = transform_map_to_wp.getOrigin();
                tf::Quaternion map_to_wp_rot = transform_map_to_wp.getRotation();
                map_to_wp_conf[0] = map_to_wp_trans.x();
                map_to_wp_conf[1] = map_to_wp_trans.y();
                double z_dir = transform_map_to_wp.getRotation().getAxis().z();
                map_to_wp_conf[2] = z_dir > 0.0 ? map_to_wp_rot.getAngle(): -map_to_wp_rot.getAngle();

//                cout<<"Base link in map:"<<endl;
//                cout<<transform_map_to_base.getOrigin().x()<<endl;
//                cout<<transform_map_to_base.getOrigin().y()<<endl;
//                cout<<transform_map_to_base.getRotation().getAngle()<<endl;
//                //cout<<transform_map_to_base.getRotation().getAngleShortestPath()<<endl;
//                cout<<transform_map_to_base.getRotation().getAxis().x()<<endl;
//                cout<<transform_map_to_base.getRotation().getAxis().y()<<endl;
//                cout<<transform_map_to_base.getRotation().getAxis().z()<<endl;


//                cout<<"Start state in base link:"<<endl;
//                cout<<transform_base_to_wp.getOrigin().x()<<endl;
//                cout<<transform_base_to_wp.getOrigin().y()<<endl;
//                cout<<transform_base_to_wp.getRotation().getAngle()<<endl;

//                cout<<"Start state in map:"<<endl;
//                cout<<map_to_wp_conf[0]<<endl;
//                cout<<map_to_wp_conf[1]<<endl;
//                cout<<map_to_wp_conf[2]<<endl;

                //Set joint position
                start_state.joint_state.position[0] = map_to_wp_conf[0] ;
                start_state.joint_state.position[1] = map_to_wp_conf[1] ;
                start_state.joint_state.position[2] = map_to_wp_conf[2] ;


                //Transform base_link to sample
                tf::StampedTransform transform_base_to_wp_vel;
                transform_base_to_wp_vel.setOrigin(tf::Vector3(start_state.joint_state.velocity[0],start_state.joint_state.velocity[1], 0.0));
                transform_base_to_wp_vel.setRotation(tf::createQuaternionFromYaw(start_state.joint_state.velocity[2]));

                //Transform map frame to sample
                tf::StampedTransform transform_map_to_wp_vel;
                transform_map_to_wp_vel.mult(transform_map_to_base,transform_base_to_wp_vel);

                //Sample in map frame (as vector)
                vector<double> map_to_wp_vel(3);
                tf::Vector3 map_to_wp_vel_trans = transform_map_to_wp_vel.getOrigin();
                tf::Quaternion map_to_wp_vel_rot = transform_map_to_wp_vel.getRotation();
                map_to_wp_vel[0] = map_to_wp_vel_trans.x();
                map_to_wp_vel[1] = map_to_wp_vel_trans.y();
                z_dir = transform_map_to_wp_vel.getRotation().getAxis().z();
                map_to_wp_vel[2] = z_dir > 0.0 ? map_to_wp_vel_rot.getAngle() : -map_to_wp_vel_rot.getAngle();

                //Set joint position
                start_state.joint_state.velocity[0] = map_to_wp_vel[0] ;
                start_state.joint_state.velocity[1] = map_to_wp_vel[1] ;
                start_state.joint_state.velocity[2] = map_to_wp_vel[2] ;
            }
        }


        //-------------- Start: Testing Object Attachement

        if(m_planning_scene_name != "empty" && m_planning_scene_name != "block" && m_planning_scene_name != "rack" && m_planning_group == "omnirob_lbr_sdh")
        {
            //Add attached collision objects
            start_state.attached_collision_objects.push_back(attached_object_modified);
        }


        //-------------- Start: Robot Joint Trajectory

        // The representation of the path contains position values for all the joints that are moving along the path; a sequence of trajectories may be specified
        moveit_msgs::RobotTrajectory robot_trajectory_local;

//        trajectory_msgs::MultiDOFJointTrajectoryPoint basepoint;
//        basepoint.transforms[0].translation.x = ;
//        basepoint.transforms[0].translation.y = ;
//        basepoint.transforms[0].translation.z = ;
//        basepoint.transforms[0].rotation = ;

//        robot_trajectory_local.multi_dof_joint_trajectory.joint_names.push_back("world_joint");
//        robot_trajectory_local.multi_dof_joint_trajectory.points.push_back(basepoint);


        //Set size of arrays storing the joint names and trajectory points
        robot_trajectory_local.joint_trajectory.joint_names.resize(m_num_joints);
        int num_configs = floor(duration/sampling_step) + 1; //number of configs along trajectory depends on trajectory sampling step width
        robot_trajectory_local.joint_trajectory.points.resize(num_configs); // -> to be checked!!!!!!!!!!!!!!!!!!


        //Set Frame ID in header
        if (m_planning_group == "kuka_complete_arm")
        {
            //Set Header frame ID
            robot_trajectory_local.joint_trajectory.header.frame_id = m_ns_prefix_robot + "lbr_0_link";
            robot_trajectory_local.multi_dof_joint_trajectory.header.frame_id = m_ns_prefix_robot + "lbr_0_link";
        }
        else if (m_planning_group == "pr2_arm")
        {
            //Set Header frame ID
            start_state.joint_state.header.frame_id = m_ns_prefix_robot + "torso_lift_link";
            start_state.multi_dof_joint_state.header.frame_id = m_ns_prefix_robot + "torso_lift_link";
        }
        else if(m_planning_group == "omnirob_lbr_sdh" || m_planning_group == "omnirob_base" || m_planning_group == "pr2_base" || m_planning_group == "pr2_base_arm" || m_planning_group == "robotino_robot")
        {
            //Set Header frame ID
            robot_trajectory_local.joint_trajectory.header.frame_id = m_ns_prefix_robot + "base_link";
            robot_trajectory_local.multi_dof_joint_trajectory.header.frame_id = m_ns_prefix_robot + "base_link";
        }
        else
            ROS_ERROR("Unknown Planning Group");


        //Set joint names
        for (int i = 0 ; i < m_num_joints; i++)
        {
            //Set Joint Name
            robot_trajectory_local.joint_trajectory.joint_names[i] = m_joint_names[i];
        }

        //Take samples from the generated trajectory
        int curr_point_idx = 0; //Current point index
        for(double t = 0.0; t < duration; t += sampling_step)
        {
            //Set size of positions array
            robot_trajectory_local.joint_trajectory.points[curr_point_idx].positions.resize(m_num_joints);
            robot_trajectory_local.joint_trajectory.points[curr_point_idx].velocities.resize(m_num_joints);

            for (int j = 0 ; j < m_num_joints; j++)
            {
              //Set joint position
              robot_trajectory_local.joint_trajectory.points[curr_point_idx].positions[j] = gen_trajectory.getPosition(t)[j] ;
              //Set joint velocity
              robot_trajectory_local.joint_trajectory.points[curr_point_idx].velocities[j] = gen_trajectory.getVelocity(t)[j];
              //Set time from start
              robot_trajectory_local.joint_trajectory.points[curr_point_idx].time_from_start = ros::Duration(t);
            }

            //Set index to next trajectory point
            curr_point_idx++;
        }


        //-------------- Convert trajectory in map frame

        //If robot has a mobile base (2 prismatic and at least a revolute joint for base rotation)
        if(m_num_joints_prismatic == 2 && m_num_joints_revolute > 0)
        {
            if(transform_map_to_base_available)
            {
                //Transform trajectory points in map frame
                for(int wp = 0 ; wp < num_configs ; wp++)
                {
                    //Transform base_link to sample
                    tf::StampedTransform transform_base_to_wp;
                    transform_base_to_wp.setOrigin(tf::Vector3(robot_trajectory_local.joint_trajectory.points[wp].positions[0],robot_trajectory_local.joint_trajectory.points[wp].positions[1], 0.0));
                    transform_base_to_wp.setRotation(tf::createQuaternionFromYaw(robot_trajectory_local.joint_trajectory.points[wp].positions[2]));

                    //Transform map frame to sample
                    tf::StampedTransform transform_map_to_wp;
                    transform_map_to_wp.mult(transform_map_to_base,transform_base_to_wp);

                    //Sample in map frame (as vector)
                    vector<double> map_to_wp_conf(3);
                    tf::Vector3 map_to_wp_trans = transform_map_to_wp.getOrigin();
                    tf::Quaternion map_to_wp_rot = transform_map_to_wp.getRotation();
                    map_to_wp_conf[0] = map_to_wp_trans.x();
                    map_to_wp_conf[1] = map_to_wp_trans.y();
                    double z_dir = map_to_wp_rot.getAxis().z();
                    map_to_wp_conf[2] = z_dir > 0.0 ? map_to_wp_rot.getAngle() : -map_to_wp_rot.getAngle();

                    //cout<<"WP: "<<wp<<" x dir:"<<transform_map_to_wp.getRotation().getAxis().x()<<endl;
                    //cout<<"WP: "<<wp<<" y dir:"<<transform_map_to_wp.getRotation().getAxis().y()<<endl;
                    //cout<<"WP: "<<wp<<" z dir:"<<z_dir<<endl;
                    //cout<<"WP: "<<wp<<" z dir 2:"<<map_to_wp_rot.getAxis().z()<<endl;
                    //cout<<"WP: "<<wp<<" angle in base_link frame:"<<robot_trajectory_local.joint_trajectory.points[wp].positions[2]<<endl;
                    //cout<<"WP: "<<wp<<" angle in map frame:"<<map_to_wp_conf[2]<<endl;

                    //Set joint position
                    robot_trajectory_local.joint_trajectory.points[wp].positions[0] = map_to_wp_conf[0] ;
                    robot_trajectory_local.joint_trajectory.points[wp].positions[1] = map_to_wp_conf[1] ;
                    robot_trajectory_local.joint_trajectory.points[wp].positions[2] = map_to_wp_conf[2] ;

                    //Transform base_link to sample
                    tf::StampedTransform transform_base_to_wp_vel;
                    transform_base_to_wp_vel.setOrigin(tf::Vector3(robot_trajectory_local.joint_trajectory.points[wp].velocities[0],robot_trajectory_local.joint_trajectory.points[wp].velocities[1], 0.0));
                    transform_base_to_wp_vel.setRotation(tf::createQuaternionFromYaw(robot_trajectory_local.joint_trajectory.points[wp].velocities[2]));

                    //Transform map frame to sample
                    tf::StampedTransform transform_map_to_wp_vel;
                    transform_map_to_wp_vel.mult(transform_map_to_base,transform_base_to_wp_vel);

                    //Sample in map frame (as vector)
                    vector<double> map_to_wp_vel(3);
                    tf::Vector3 map_to_wp_vel_trans = transform_map_to_wp_vel.getOrigin();
                    tf::Quaternion map_to_wp_vel_rot = transform_map_to_wp_vel.getRotation();
                    map_to_wp_vel[0] = map_to_wp_vel_trans.x();
                    map_to_wp_vel[1] = map_to_wp_vel_trans.y();
                    z_dir = transform_map_to_wp_vel.getRotation().getAxis().z();
                    map_to_wp_vel[2] = z_dir > 0.0 ? map_to_wp_vel_rot.getAngle() : -map_to_wp_vel_rot.getAngle();

                    //Set joint position
                    robot_trajectory_local.joint_trajectory.points[wp].velocities[0] = map_to_wp_vel[0] ;
                    robot_trajectory_local.joint_trajectory.points[wp].velocities[1] = map_to_wp_vel[1] ;
                    robot_trajectory_local.joint_trajectory.points[wp].velocities[2] = map_to_wp_vel[2] ;
                }
            }
        }

         //++++++++++ END: TESTING ++++++++++

        //Set data for MoveIt Message
        if (m_planning_group == "kuka_complete_arm") // ->Set model_id, i.e. The model id for which this path has been generated
        {
            trajectory_rviz.model_id = "lbr";
        }
        else if(m_planning_group == "omnirob_lbr_sdh" || m_planning_group == "omnirob_base")
        {
            trajectory_rviz.model_id = "omnirob";
        }
        else if(m_planning_group == "robotino_robot")
        {
            trajectory_rviz.model_id = "robot";
        }
        else if(m_planning_group == "pr2_base" || m_planning_group == "pr2_base_arm" || m_planning_group == "pr2_arm")
        {
            trajectory_rviz.model_id = "pr2";
        }
        else
            ROS_ERROR("Unknown Planning Group");



        //Set robot start state
        trajectory_rviz.trajectory_start = start_state;
        trajectory_rviz.trajectory.clear();
        trajectory_rviz.trajectory.push_back(robot_trajectory_local); //set robot trajectory




    }//end if generated trajectory from path is valid

    //------------- Execute Trajectory using moveit_msgs/DisplayTrajectory (MoveIt) message -----------------------

    //Publish the generated MoveIt Display Trajectory Message
    m_pub_traj.publish(trajectory_rviz);


    //+++++++++ Start: Testing Object Dettachement +++++++++

    if(m_planning_scene_name != "empty" && m_planning_scene_name != "block" && m_planning_scene_name != "rack" && m_planning_group == "omnirob_lbr_sdh")
    {
        //Remove attached collision objects
        m_planning_world->detachObjectfromEndeffector(attached_object_modified,m_ee_trajectory[m_ee_trajectory.size()-1]);
    }

    //+++++++++ End: Testing Object Dettachement +++++++++



//    //------------- Execute Trajectory using the Planning Scene Robot State -----------------------

//    //Map containing joint names and values
//    map<string, double> nvalues_;

//    //Set name of planning scene service
//    const std::string PLANNING_SCENE_SERVICE = "get_planning_scene";

//    //Get curent planning scene
//    psm_->requestPlanningSceneState(PLANNING_SCENE_SERVICE);
//    planning_scene_monitor::LockedPlanningSceneRW ps(psm_);
//    ps->getCurrentStateNonConst().update();
//    //if you want to modify it
//    planning_scene::PlanningScenePtr scene = ps->diff();
//    scene->decoupleParent();

//    //Iterate through joint trajectorys
//    for(int tp = 0 ; tp < m_joint_trajectory.size() ; tp++)
//    {
//        //Get next Joint trajectory points
//        for(int j = 0 ; j < m_joint_names.size() ; j++)
//            nvalues_[m_joint_names[j]] = m_joint_trajectory[tp][j];

//        robot_state::RobotState state(psm_->getRobotModel());
//        state.setToDefaultValues();

//        //Set current robot state
//        state.setVariablePositions(nvalues_);

//        //Apply robot state to planning scene
//        scene->setCurrentState(state);

//        //Publish state on planning scene
//        moveit_msgs::PlanningScene psmsg;
//        scene->getPlanningSceneMsg(psmsg);
//        psmsg.robot_state.is_diff = true;
//        psmsg.is_diff = true;
//        scene_pub_.publish(psmsg);
//        ros::Duration(sleep_duration).sleep();

//    }
}



} //end of namespace


#include <cstdio>
#include <iostream>
#include <string>
#include <tf/transform_broadcaster.h>


// for receiving the odometry of the robots 
#include <nav_msgs/Odometry.h>

#include <std_msgs/Float32.h>

// for robot to landmark and robot to robot (and target=robot) measurements
#include <utiasdata_to_rosbags/MCLAMMeasurementData.h>

// for once receiving the info of the landmarks GT
#include <utiasdata_to_rosbags/MCLAM_landmark_GTData.h>

// for publishing the estimatated and ground truth states of the robots
#include <utiasdata_to_rosbags/MCLAM_RobotTeamState.h>


// for receiving the robot ground truths
#include <geometry_msgs/PoseStamped.h>

#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "g2o/config.h"
#include "g2o/core/estimate_propagator.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"


#include "g2o/stuff/macros.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/filesys_tools.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/stuff/timeutil.h"

#include "g2o/apps/g2o_cli/g2o_common.h"
#include "g2o/apps/g2o_cli/dl_wrapper.h"

#include "g2o/stuff/command_args.h"
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sys/time.h>


//#define SAVE_GRAPHFILES
#undef SAVE_GRAPHFILES 

// #define M_PI        3.141592653589793238462643383280    /* pi */

// Subject (robot) Barcode (fixed for all datasets)
//  1 	   5 
//  2 	  14 
//  3 	  41 [Target[2]]
//  4 	  32 [Target[1]]
//  5 	  23 [Target[0]]


///TODO These hard coded values must go out of this place. Preferably as the arguments of the main function and therefore in the launch file.

int NUM_ROBOTS;// total number of robots in the team including self

int MAX_ROBOTS;// total number of robots in the team including self

int NUM_TARGETS; // Number of targets being tracked. In MRCLAM dataset, we simulate only one target for now: robot number 5.

int MY_ID; // Use this flag to set the ID of the robot expected to run a certain decentralized algorithm. Robot with MY_ID will be trated as the self robot running the algorithm while the rest will be considered teammates. Note that in the dataset there are 5 robots with IDs 1,2,3,4 and 5. Robot with ID=5 is treated as the tracked target.

//Below are empirically obtained coefficients in the covariance expression. See (add publications here)

//coefficients for landmark observation covariance
double K1;
double K2;

//coefficients for target observation covariance
double K3;
double K4;
double K5; 
//const float K3 = 0.2, K4 = 0.5, K5 = 0.5; 

const std::size_t ROB_HT = 0.81; //(In this dataset) fixed height of the robots above ground in meter


//Initial 2D positons of the robots and the target initialized with 0.
const double initArray[10] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

bool COP_LOC;
bool USE_LANDMARKS ;

int MAX_VERTEX_COUNT;//4*6000;
int MAX_INDIVIDUAL_STATES;
int WINDOW_SIZE; //these two are not const anymore because they are set by the main is not 
int DECAY_LAMBDA;
int WINDOW_SIZE_TGT;
int MAX_ITERATIONS;
float avgComputationTime;
double decayCoeff;
bool SLAM;

char resultsFileName[500];

using namespace ros;
using namespace std;
using namespace g2o;

//boost::mutex graph_mutex;

class Target
{
    int targetID;
  
  public:
    Target(Eigen::Vector3d _initPose): initPose(_initPose), tgtVertexID(0), tgtVertexCounter(0)
    {
    }
    
    //We use either this function to create a target node everytime a target is observed
    void addNewTGTVertexAndEdgesAround(int, double, double, int, int, Time, g2o::SparseOptimizer*);
    //Or we use this function assuming that the target nodes are created in sync with the Self-Robot Pose Nodes 
    void addObservationEdgesPoseToTarget(double, double, int, int, int, Time, g2o::SparseOptimizer*);
  
    Eigen::Vector3d initPose;
    long long unsigned curObservationTimestamp;
    long long unsigned prevObservationTimestamp;
    
    long long unsigned curNodeTimestamp;
    long long unsigned prevNodeTimestamp;
    
    int tgtVertexID;
    int tgtVertexID_prev;  
    int tgtVertexCounter;
    Eigen::Vector3d curPose;
    Eigen::Vector3d prevPose;
};



class SelfRobot
{
  NodeHandle *nh;
  //One subscriber per sensor in the robot
  Subscriber sOdom_;
  Subscriber sMeasurements_;
  Subscriber sRobotsGT_;
  
  bool notInitialized;
  
  Eigen::Isometry2d initPose; // x y theta;
  Eigen::Isometry2d prevPose;  
  
  int SE2vertexID_prev;
  int SE2vertexID; // current vertex id holder for the robot state
  //prev_id for target is only for the self robot
  int targetVertexID_prev;
  bool *ifRobotIsStarted;
  
  ///@TODO convert this to an array of pointers perhaps for multi-target case
  int *targetVertexID;  //current vertex id holder for the ball's state. it is passed as a pointer because the value is shared by all robots as it is for the same target
  int vertextCounter_; // counter of vertices for the individual robots
  int targetVertextCounter_;  
  int *totalVertextCounter;
  int solverStep;  
  int windowSolverInvokeCount;  
  
  Publisher State_publisher, virtualGTPublisher;
  Publisher diagnosticError1,diagnosticError2;
  utiasdata_to_rosbags::MCLAM_RobotTeamState msg;
  
  vector<Target*> targetsToTrack;
  vector<int>* subject_barcode;
  vector<Eigen::Isometry2d>* teamGTPose;
  
  int *currentPoseVertexIDs;
  
  FILE *mhls_utias_g2o;
  FILE *resultsFile;
  
  float *computationTime;
 
  int *dynamic_window_size;  
  double invSigmaX,invSigmaY,invSigmaTheta;
  
  public:
    SelfRobot(NodeHandle *nh, g2o::SparseOptimizer* graph, int robotNumber, int startCounter, int* totVertCount, int* tgtVertexID, Eigen::Isometry2d _initPose, vector<Target*> _targetsToTrack,int* _curPosVerID, bool *_ifRobotIsStarted, vector<int>* _subject_barcode, vector<Eigen::Isometry2d>* _teamGTPose, int* _dynamic_window_size): vertextCounter_(startCounter), totalVertextCounter(totVertCount), SE2vertexID_prev(0), SE2vertexID(0), initPose(_initPose), targetVertexID(tgtVertexID), targetVertextCounter_(0), targetsToTrack(_targetsToTrack), solverStep(0), currentPoseVertexIDs(_curPosVerID),ifRobotIsStarted(_ifRobotIsStarted), subject_barcode(_subject_barcode),teamGTPose(_teamGTPose),dynamic_window_size(_dynamic_window_size)
    {
    
      computationTime = new float[MAX_VERTEX_COUNT];
      
      sOdom_ = nh->subscribe<nav_msgs::Odometry>("/Robot_"+boost::lexical_cast<string>(robotNumber+1)+"/odometry", 1000, boost::bind(&SelfRobot::selfOdometryCallback,this, _1,robotNumber+1,graph));
      
      sMeasurements_ = nh->subscribe<utiasdata_to_rosbags::MCLAMMeasurementData>("/Robot_"+boost::lexical_cast<string>(robotNumber+1)+"/measurements", 1000, boost::bind(&SelfRobot::selfMeasurementDataCallback,this, _1,robotNumber+1,graph));
      
      sRobotsGT_ = nh->subscribe<geometry_msgs::PoseStamped>("/Robot_"+boost::lexical_cast<string>(robotNumber+1)+"/GTpose", 1000, boost::bind(&SelfRobot::selfGTCallback,this, _1,robotNumber+1,graph));      
      
      State_publisher = nh->advertise<utiasdata_to_rosbags::MCLAM_RobotTeamState>("/utias_mhlspose_estimated", 1000);
      
      virtualGTPublisher = nh->advertise<utiasdata_to_rosbags::MCLAM_RobotTeamState>("/gt_synced_utias_poses", 1000); 
      
      diagnosticError1 = nh->advertise<std_msgs::Float32>("/diagnosticError1", 1000);
      diagnosticError2 = nh->advertise<std_msgs::Float32>("/diagnosticError2", 1000);
      
      ROS_INFO(" constructing SelfRobot <<object>> and called sensor subscribers for this robot %d",robotNumber+1);   
      
      notInitialized = true;
      
      mhls_utias_g2o = fopen("mhls_utias.g2o","w");
      ROS_INFO(" file name is %s",resultsFileName);
      resultsFile = fopen(resultsFileName,"w");
      
      avgComputationTime = 0;
      dynamic_window_size[robotNumber] = WINDOW_SIZE;
      
    }

    /// Use this method to implement perception algorithms
    void selfOdometryCallback(const nav_msgs::Odometry::ConstPtr&, int, g2o::SparseOptimizer*);
    
    /// Use this method to implement perception algorithms
    void selfMeasurementDataCallback(const utiasdata_to_rosbags::MCLAMMeasurementData::ConstPtr&, int, g2o::SparseOptimizer*);
    
    /// Use this method to implement perception algorithms
    void selfGTCallback(const geometry_msgs::PoseStamped::ConstPtr&, int, g2o::SparseOptimizer*);
    
    /// Solve the sliding widnow graph
    void solveSlidingWindowGraph(g2o::SparseOptimizer*);
    
    /// Use this method to ipublish states
    void publishState(g2o::SparseOptimizer*);  
    
    /// Use this method to generate the sigmas or its inverse
    void makeOdomSigmas(double, double, double);
    
    Eigen::Isometry2d curPose;
    Eigen::Isometry2d curGTPose;
    Time curTime;
    Time prevTime;  
    
};


class TeammateRobot
{
  NodeHandle *nh;
  //One subscriber per sensor in the robot
  Subscriber sOdom_;
  Subscriber sMeasurements_;
  Subscriber sRobotsGT_;
  
  bool notInitialized;  

  Eigen::Isometry2d initPose; // x y theta;
  Eigen::Isometry2d prevPose;
  int SE2vertexID_prev;
  int SE2vertexID; // current vertex id holder for the robot state
  int *targetVertexID;  //current vertex id holder for the ball's state
  int vertextCounter_; // counter of vertices for the individual robots
  int *totalVertextCounter;
  //Publisher robotState_publisher; // has no target state publisher... only selfRobot publishes target state
  bool *ifRobotIsStarted;
  
  vector<Target*> targetsToTrack;
  vector<int>* subject_barcode;
  vector<Eigen::Isometry2d>* teamGTPose;
  
  int *currentPoseVertexIDs;  
  
  int *dynamic_window_size;
  
  double invSigmaX,invSigmaY,invSigmaTheta;
  
  public:
    TeammateRobot(NodeHandle *nh, g2o::SparseOptimizer* graph, int robotNumber, int startCounter, int* totVertCount, int* tgtVertexID, Eigen::Isometry2d _initPose, vector<Target*> _targetsToTrack, int* _curPosVerID, bool *_ifRobotIsStarted, vector<int>* _subject_barcode, vector<Eigen::Isometry2d>* _teamGTPose, int* _dynamic_window_size): curPose(_initPose), vertextCounter_(startCounter), totalVertextCounter(totVertCount), SE2vertexID_prev(0), SE2vertexID(0), initPose(_initPose), targetVertexID(tgtVertexID), targetsToTrack(_targetsToTrack),currentPoseVertexIDs(_curPosVerID),ifRobotIsStarted(_ifRobotIsStarted), subject_barcode(_subject_barcode),teamGTPose(_teamGTPose),dynamic_window_size(_dynamic_window_size)
    {
      ifRobotIsStarted[robotNumber] = false;
    
      sOdom_ = nh->subscribe<nav_msgs::Odometry>("/Robot_"+boost::lexical_cast<string>(robotNumber+1)+"/odometry", 10, boost::bind(&TeammateRobot::teammateOdometryCallback,this, _1,robotNumber+1,graph));
      
      sMeasurements_ = nh->subscribe<utiasdata_to_rosbags::MCLAMMeasurementData>("/Robot_"+boost::lexical_cast<string>(robotNumber+1)+"/measurements", 10, boost::bind(&TeammateRobot::teammateMeasurementDataCallback,this, _1,robotNumber+1,graph));
      
      sRobotsGT_ = nh->subscribe<geometry_msgs::PoseStamped>("/Robot_"+boost::lexical_cast<string>(robotNumber+1)+"/GTpose", 10, boost::bind(&TeammateRobot::teammateGTCallback,this, _1,robotNumber+1,graph));      
      
      ROS_INFO(" constructing TeammateRobot object and called sensor subscribers for robot %d",robotNumber+1);
      
      notInitialized = true;
      dynamic_window_size[robotNumber] = WINDOW_SIZE;
    }

    /// Use this method to implement perception algorithms
    void teammateOdometryCallback(const nav_msgs::Odometry::ConstPtr&, int, g2o::SparseOptimizer*);
    
    /// Use this method to implement perception algorithms
    void teammateMeasurementDataCallback(const utiasdata_to_rosbags::MCLAMMeasurementData::ConstPtr&, int, g2o::SparseOptimizer*);
    
    /// Use this method to implement perception algorithms
    void teammateGTCallback(const geometry_msgs::PoseStamped::ConstPtr&, int, g2o::SparseOptimizer*);
    
    /// Use this method to generate the sigmas or its inverse
    void makeOdomSigmas(double, double, double);    
  

    Eigen::Isometry2d curPose;
    Eigen::Isometry2d curGTPose;
    Time curTime;
    Time prevTime;
    
};



class ReadRobotMessages
{
  NodeHandle* nh_;
  Rate loop_rate_;
  Subscriber subfixedLandmarkGTs;
  
  SelfRobot* robot_;
  vector<TeammateRobot*> teammateRobots_;
  vector<Target*> targets_;
  
  g2o::SparseOptimizer* graph_;
  
  vector<int> subject_barcode_;
  vector<Eigen::Isometry2d> teamGTPose_;
  
  int totalVertextCounter_; // counter of vertices for the full graph (including all robots and the targets)
  int targetVertexID_;  //current vertex id holder for the ball's state  
  int *currentPoseVertexID; // we have to do it to nitialize all with 0 
  int *dynamic_window_sizes;
  bool *robotStarted; // to indicaate whether a robot has started or not..
 
 
  public:
    ReadRobotMessages(g2o::SparseOptimizer* _graph, NodeHandle* _nh ): graph_(_graph), nh_(_nh), loop_rate_(30),totalVertextCounter_(0)
    { 
      
      currentPoseVertexID = new int[MAX_ROBOTS]; // we have to do it to nitialize all with 0 
      dynamic_window_sizes= new int[NUM_ROBOTS];
      robotStarted= new bool[NUM_ROBOTS]; // to indicaate whether a robot has started or not..
    
      Eigen::Isometry2d initialRobotPose;
      Eigen::Vector3d initialTargetPosition;      
      
      teammateRobots_.reserve(NUM_ROBOTS);
      targets_.reserve(NUM_TARGETS);
      subject_barcode_.reserve(20);
      teamGTPose_.reserve(5);
      for(int rob=0;rob<MAX_ROBOTS;rob++)
	{
	  currentPoseVertexID[rob] = 0;
	}
	
      for(int j=0;j<NUM_TARGETS;j++)
      {
	initialTargetPosition = Eigen::Vector3d(0,0,0);
	Target *tempTarget = new Target(initialTargetPosition);
	targets_.push_back(tempTarget);
      }      
      
      initialRobotPose = Eigen::Rotation2Dd(-M_PI).toRotationMatrix();
      initialRobotPose.translation() = Eigen::Vector2d(0,0);
	
      robot_ = new SelfRobot(nh_, graph_, 
			     MY_ID-1,      0,&totalVertextCounter_,&targetVertexID_,initialRobotPose,targets_,&currentPoseVertexID[0],&robotStarted[0],&subject_barcode_,&teamGTPose_,&dynamic_window_sizes[0]);	
      
      int robotsInitiated = 1; // 1 robot is already initiated
      int i = 0;
      while(robotsInitiated<NUM_ROBOTS)
      {
	if(i+1 != MY_ID)
	{
	  TeammateRobot *tempRobot = new TeammateRobot(nh_, graph_,i,0,&totalVertextCounter_,&targetVertexID_,initialRobotPose, targets_,&currentPoseVertexID[0],&robotStarted[0],&subject_barcode_,&teamGTPose_,&dynamic_window_sizes[0]);
	  teammateRobots_.push_back(tempRobot);
	  robotsInitiated++;
	}
	i++;
      }
      
      subfixedLandmarkGTs = nh_->subscribe<utiasdata_to_rosbags::MCLAM_landmark_GTData>("all_landmark_gtdata", 1000,boost::bind(&ReadRobotMessages::initializeFixedLandmarks,this,_1,graph_));
    }
    
    void initializeFixedLandmarks(const utiasdata_to_rosbags::MCLAM_landmark_GTData::ConstPtr&, g2o::SparseOptimizer*);
    
};







































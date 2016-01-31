
#include "mhls_utias_dataset.h"



////////////////////////// METHOD DEFINITIONS OF THE ReadRobotMessages CLASS //////////////////////


void SelfRobot::makeOdomSigmas(double x, double y, double theta)
{
  //invSigmaX = 5000.0; invSigmaY = 5000000.0; invSigmaTheta = 5000.0;

  //if(x>0.0001)
   invSigmaX = 1000;//1/(100*x);
  
  //if(y>0.0001)
   invSigmaY = 1000;//10/(y);    
  
  //if(theta>0.0001)
   invSigmaTheta = 1000;//1/(100*theta);  
   
}

void TeammateRobot::makeOdomSigmas(double x, double y, double theta)
{
  //invSigmaX = 5000.0; invSigmaY = 5000000.0; invSigmaTheta = 5000.0;

  //if(x>0.0001)
   invSigmaX = 1000;//1/(100*x);
  
  //if(y>0.0001)
   invSigmaY = 1000;//10/(y);    
  
  //if(theta>0.0001)
   invSigmaTheta = 1000;//1/(100*theta);  
   
}


inline double normalizeAngle(double x)
{
    x = fmod(x + M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}


inline void addLandmark(int vertexId, double x, double y, g2o::SparseOptimizer* graph_ptr)
{
  ROS_INFO("A fixed landmark with ID %d at position x=%f, y=%f is created",vertexId,x,y);

     Eigen::Vector2d landmark;
     VertexPointXY * v = new VertexPointXY();
     
     if (SLAM)
     {
      if(vertexId==5)
	{
	  v->setFixed(true);
	  landmark =  Eigen::Vector2d(x, y);
	  v->setId(vertexId);  
	  v->setEstimate(landmark);	
	}
	else
	{
	  landmark =  Eigen::Vector2d(0, 0);
	  v->setId(vertexId);  
	  v->setEstimate(landmark);
	  v->setFixed(false);
	}
     }
     else
     {
	  v->setFixed(true);
	  landmark =  Eigen::Vector2d(x, y);
	  v->setId(vertexId);  
	  v->setEstimate(landmark);	       
     }
       
      graph_ptr->addVertex(v);  
  
      //printf(" Added landmark numer %d at x = %f, y = %f\n",vertexId,x,y);
}


inline void addRobotRobotEdge( int observedSubjectCode, float range, float bearing, int fromVertexID, int toVertexID,  g2o::SparseOptimizer* graph_ptr)
{
    //ROS_WARN(" -------------------------------------------------------- Teammate Robot with subject code %d  Detected -------- adding Robot-Robot Edge",observedSubjectCode);
    double localX = range*cos(bearing);
    double localY = range*sin(bearing);
    
    if(fromVertexID>0 && toVertexID>0)
    {
      
    double covDD = K1;//K1*(range*range);
    double covPhiPhi = K2;//K2*range;
    double covXX = pow(cos(bearing),2) * covDD + pow(sin(bearing),2) * ( pow(range,2) * covPhiPhi + covDD * covPhiPhi );
    double covYY = pow(sin(bearing),2) * covDD + pow(cos(bearing),2) * ( pow(range,2) * covPhiPhi + covDD * covPhiPhi );       
    
    Eigen::Isometry2d measurement; 
    measurement = Eigen::Rotation2Dd(0).toRotationMatrix();
    measurement.translation() = Eigen::Vector2d(localX,localY);      
      
      EdgeSE2 * e = new EdgeSE2;
      // retrieve vertex pointers from graph with id's
      g2o::OptimizableGraph::Vertex * pose_a_vertex
	      = dynamic_cast<g2o::OptimizableGraph::Vertex*>
		(graph_ptr->vertices()[fromVertexID]);
      g2o::OptimizableGraph::Vertex * pose_b_vertex
	      = dynamic_cast<g2o::OptimizableGraph::Vertex*>
		(graph_ptr->vertices()[toVertexID]);			
      
      if(pose_a_vertex!=NULL && pose_a_vertex->dimension() == 3)		
	e->vertices()[0] = pose_a_vertex;
      else
      {
	ROS_WARN(" From Robot pose does not exist at %d ??? ", fromVertexID);
	ROS_WARN("To Robot pose may or may not exist at %d ??? ", toVertexID);
	return;
      }   	      
      
      if(pose_b_vertex!=NULL && pose_b_vertex->dimension() == 3)
	e->vertices()[1] = pose_b_vertex;
      else
      {
	ROS_WARN(" From Robot pose is there at %d ??? ", fromVertexID);
	ROS_WARN("To Robot pose does not exist at %d ??? ", toVertexID);
	return;
      }           
	    
      // add information matrix
      Eigen::Matrix<double, 3, 3> Lambda;      
      Lambda<<1/covXX, 0.0, 0.0, 0.0,1/covYY, 0.0,0.0,0.0, 0.0001; // very low information on the angular measurement (which does not exist).      
      
      // set the observation and imformation matrix
      const SE2 a(measurement);
      e->setMeasurement(a);
      e->information() = Lambda;      
      
      // finally add the edge to the graph
      if(!graph_ptr->addEdge(e))
      {
	assert(false);
      }
    }    
  
}


void ReadRobotMessages::initializeFixedLandmarks(const utiasdata_to_rosbags::MCLAM_landmark_GTData::ConstPtr& data, g2o::SparseOptimizer* graph_ptr)
{  
    //vertexID of the landmark should correspond to the landmark subject number in the dataset!
    for(int i=0;i<20;i++)
    {
      subject_barcode_.push_back(data->barcode[i]);
      //only subjects 5 to 19 are landmarks!
      if(i>=5)
	addLandmark(i, data->landmarkGTpose[i].pose.position.x,data->landmarkGTpose[i].pose.position.y,graph_ptr);
    }
    //addLandmark(1,  6.0, -4.5);

/*  for (auto j = subject_barcode_.begin(); j != subject_barcode_.end(); ++j)
    std::cout << *j << ' ';*/  

  //After adding the landmarks once, we unsubscribe from this topic because all messages are the same!
  subfixedLandmarkGTs.shutdown();
  // graph 

}


void Target::addNewTGTVertexAndEdgesAround(int tgtID, double tgtX, double tgtY ,int RobotID, int robotVertexID, Time curObsTime, g2o::SparseOptimizer* graph_ptr)
{
   //If it is the self robot we will obviously add a new vertex and a new tgt-tgt edge
   //If it is the teammate observing it, we need to check if the teammates observation is too new from the previous target node 

   targetID = tgtID;
   
   bool makeNewTargetNode = false;
   if(/*tgtVertexCounter==0 || */RobotID==MY_ID)
	makeNewTargetNode = true;
   
   if(tgtVertexCounter==0)
   {
      curObservationTimestamp = (unsigned long long) 1000000*curObsTime.sec + curObsTime.nsec/1000;
   }
   else
   {
      prevObservationTimestamp = curObservationTimestamp;
      curObservationTimestamp = (unsigned long long) 1000000*curObsTime.sec + curObsTime.nsec/1000;     
   }
   
   //time difference in sec in previous and current observation of the target. A new node is made only if the delta T is larger than the time period (1/frequency) of observation of a single robot
   float deltaT = ((float)(abs(curObservationTimestamp-prevObservationTimestamp)))/1000000;
   if(deltaT>0.030) // 30 millisecond is the observation frequnecy inverse.
     makeNewTargetNode = true;
   
   //ROS_INFO(" delta T in observation  !!! %f\n", deltaT);
   
   
   if(makeNewTargetNode)
   {
      if(tgtVertexCounter==0 )
	tgtVertexID = (NUM_ROBOTS+(targetID+1))*MAX_INDIVIDUAL_STATES;
      else
	tgtVertexID++;
      //   cout<<" ball seq value is "<<seq<<endl; 
      
      //vertex id for the target start after all the robot's vertices hence below (NUM_ROBOTS+1) (below the first robot's id the ids are reserved for the landmarks/environmental features ... approximately equal to a value of MAX_INDIVIDUAL_STATES)
      //moreover, the target vertices are created only for and when the robot running the algorithm receives the odometry signal (not the ball detection signal because it is assumed to have a lower frequency)
      //create a ball/target vertex
      
      //Robot pose node's pose is:
      double initX,initY;
      if(tgtVertexCounter==0)
      {
	VertexSE2* mostRecetRobotPoseVertex = dynamic_cast<VertexSE2*>(graph_ptr->vertices()[robotVertexID]);
	Isometry2d cuRobPose = mostRecetRobotPoseVertex->estimate().toIsometry();
	
	//First initialize the target node vertex
	Eigen::Isometry2d targetNodeInitiPoint; 
	targetNodeInitiPoint = Eigen::Rotation2Dd(0);
	targetNodeInitiPoint.translation() = Eigen::Vector2d(tgtX, tgtY);  
	targetNodeInitiPoint = targetNodeInitiPoint*cuRobPose;
	initX = targetNodeInitiPoint.translation().x();
	initY = targetNodeInitiPoint.translation().y();
      }
      else
      {
	VertexXY_VXVY* olderTargetNode = dynamic_cast<VertexXY_VXVY*>(graph_ptr->vertices()[tgtVertexID-1]);
	initX = olderTargetNode->estimate().x();
	initY = olderTargetNode->estimate().y();
      }
      
      
      Eigen::Vector4d curBallPosition = Eigen::Vector4d(initX,initY,0.0,0.0);      
      //tgtVertexID = (NUM_ROBOTS+1)*MAX_INDIVIDUAL_STATES + seq;
      VertexXY_VXVY * v_target = new VertexXY_VXVY();
      v_target->setId(tgtVertexID);  
      v_target->setEstimate(curBallPosition);
      v_target->isOptimizedAtLeastOnce = false;
      v_target->ballZCoordinate = 0.0;
      v_target->timestamp =  (unsigned long long) 1000000*curObsTime.sec + curObsTime.nsec/1000;
      //if(vertextCounter_ == 0)
      //v_target->setFixed(true);
      v_target->setFixed(false);
      graph_ptr->addVertex(v_target);
      
	if(tgtVertexCounter>0) //start adding edges if seq is greater than 0
	{  
	 prevNodeTimestamp = curNodeTimestamp;
	 curNodeTimestamp = (unsigned long long) 1000000*curObsTime.sec + curObsTime.nsec/1000;
	//target to target edge goes here... just like the se2 to se2 edge and like the vertex of the target
	    
	  //targetVertexID_prev = (NUM_ROBOTS+1)*MAX_INDIVIDUAL_STATES + seq - 1;
	  tgtVertexID_prev = tgtVertexID - 1;
	  //Now add edge from the current node to the previous node
	  EdgeXY_VXVY * e_target = new EdgeXY_VXVY;
	  // retrieve vertex pointers from graph with id's
	  g2o::OptimizableGraph::Vertex * pose_a_vertex_tgt
		  = dynamic_cast<g2o::OptimizableGraph::Vertex*>
		    (graph_ptr->vertices()[tgtVertexID_prev]);
	  g2o::OptimizableGraph::Vertex * pose_b_vertex_tgt
		  = dynamic_cast<g2o::OptimizableGraph::Vertex*>
		    (graph_ptr->vertices()[tgtVertexID]);			
		    
		    
	  // error check vertices and associate them with the edge
	  assert(pose_a_vertex_tgt!=NULL);
	  assert(pose_a_vertex_tgt->dimension() == 4);
	  e_target->vertices()[0] = pose_a_vertex_tgt;

	  assert(pose_b_vertex_tgt!=NULL);
	  assert(pose_b_vertex_tgt->dimension() == 4);
	  e_target->vertices()[1] = pose_b_vertex_tgt;	  
		
	  // add information matrix
	  ///@TODO What is this magic number? FIX it!
	  float invQ = 100000000; //Just for testing acceleration noise variance = (0.0001)^2 m^2/s^4
	  // now finding the information matrix components which form the edge!
	  float T = ((float)(abs(curNodeTimestamp-prevNodeTimestamp)))/1000000;//delta T between the two states in seconds (division by 1000000 since timestamps are in microseconds)
	  if(T == 0)
	  {
	    ROS_WARN(" delta T = 0 !!! ");
	  }
	  
	  ///@HACK 
	  ///@TODO after a certain gap in observation, we consider the motion model is no more valid therefore we attenuate the T so that it does not go up to infinity
	  if(T>0.1)
	    T = 0.1;
	    
	  float T4by2 = invQ*powf(T,4)/2, T3by2 = invQ*powf(T,3)/2, T2 = invQ*powf(T,2); 

		  
	  Eigen::Matrix<double, 4, 4> Lambda_tgt;      
	  Lambda_tgt<<T4by2, T4by2, T3by2, T3by2, 
		  T4by2, T4by2, T3by2, T3by2,
		  T3by2, T3by2, T2, T2,
		  T3by2, T3by2, T2, T2;
	  //Lambda.setIdentity();
	  
	  // set the observation and imformation matrix
	  //const SE2 a(odom);
	  Eigen::Vector4d diff = Eigen::Vector4d(0.0,0.0,0.0,0.0);    
	  e_target->setMeasurement(diff);
	  e_target->information() = Lambda_tgt;      
	  
	  // finally add the edge to the graph
	  if(!graph_ptr->addEdge(e_target))
	  {
	    assert(false);
	  }	  
	  
	  
	}
	else///First node is not set as a fixed node.. A prior node with 0 mean and high covariance is created and attached to the firt node with a very loose edge
	{
	  curNodeTimestamp = (unsigned long long) 1000000*curObsTime.sec + curObsTime.nsec/1000;
	  
	  v_target->setFixed(false); 
      
	  Eigen::Vector4d priorTargetPosition;
	  VertexXY_VXVY* firstTargetnode = dynamic_cast<VertexXY_VXVY*>(graph_ptr->vertices()[tgtVertexID]);
	  priorTargetPosition = Eigen::Vector4d(0,0,0.0,0.0);
	  //Create the Prior SE2 Node now. This node will keep track of the past information.
	  VertexXY_VXVY * v_target_prior = new VertexXY_VXVY();
	  v_target_prior->setId((NUM_ROBOTS+(targetID+1))*MAX_INDIVIDUAL_STATES - 1); //The ID is one before the first node position of this target  
	  v_target_prior->setEstimate(priorTargetPosition);
	  v_target_prior->timestamp =  firstTargetnode->timestamp;
	  v_target_prior->setFixed(true);
	  graph_ptr->addVertex(v_target_prior);
	  
	  //Now connect the fixed prior node to the first node through a very "loose" edge! An edge that means we have poorinformation regarding the prior
	  EdgeXY_VXVY * edgeFirst_to_Prior = new EdgeXY_VXVY;	
	  edgeFirst_to_Prior->vertices()[0] = v_target_prior;
	  edgeFirst_to_Prior->vertices()[1] = firstTargetnode;	  

	  // add information matrix and set the observation and imformation matrix
	  Eigen::Matrix<double, 4, 4> Lambda_tgt_loose;      
	  Lambda_tgt_loose<<
		  0.001, 0, 0, 0, 
		  0, 0.001, 0, 0,
		  0, 0, 0.0001, 0,
		  0, 0, 0, 0.0001;
	  
	  Eigen::Vector4d diff = Eigen::Vector4d(0.0,0.0,0.0,0.0);  
	  edgeFirst_to_Prior->setMeasurement(diff);
	  edgeFirst_to_Prior->information() = Lambda_tgt_loose;      
	  // finally add the edge to the graph
	  if(!graph_ptr->addEdge(edgeFirst_to_Prior))
	  {
	    assert(false);
	  } 
	  
	}

      tgtVertexCounter++;   
      
      //Remove old nodes and edges in the spirit of dynamic window approach
	// Start fixing old target nodes for the Sliding window optimization approach!
      if(tgtVertexCounter>WINDOW_SIZE_TGT)
	{
	    Eigen::Vector4d priorTargetPosition;// = Eigen::Vector4d(0,0,0.0,0.0);
      
	    // Prior (Mean) will be set to the vertex pose that is going to be deleted in the sliding window case 
	    VertexXY_VXVY* vertex_to_become_prior = dynamic_cast<VertexXY_VXVY*>(graph_ptr->vertices()[tgtVertexID-WINDOW_SIZE_TGT]);
	    priorTargetPosition = Eigen::Vector4d(vertex_to_become_prior->estimate().x(),vertex_to_become_prior->estimate().y(),vertex_to_become_prior->estimate().z(),vertex_to_become_prior->estimate().w());
	    
	    //We now have to remove the last added prior node and a new prior node that contains the updated past. One may simply update the previous prior node but seems like it is not so straightforward to do so using g2o. Therefore I am copying the prious prior, deleting it, updating locally and then adding a new prior node which is the updated prior node
	    VertexXY_VXVY* v_prior_old = dynamic_cast<VertexXY_VXVY*>(graph_ptr->vertices()[(NUM_ROBOTS+(targetID+1))*MAX_INDIVIDUAL_STATES - 1]);
	    graph_ptr->removeVertex(v_prior_old,/*bool detach=*/false);
	    
	    VertexXY_VXVY * v_target_prior = new VertexXY_VXVY();
	    v_target_prior->setId((NUM_ROBOTS+(targetID+1))*MAX_INDIVIDUAL_STATES - 1); //The ID is one before the first node pose of this robot, same as the old prior
	    v_target_prior->setEstimate(priorTargetPosition);
	    v_target_prior->timestamp =  vertex_to_become_prior->timestamp;
	    v_target_prior->setFixed(true);
	    graph_ptr->addVertex(v_target_prior);
	    
	
	    //Now remove the oldest node (vertex_to_become_prior), which has already been used to update the new prior node to maintain the sliding window. However, grab the last hessian block of this vertex. This hessian will become the information contained by the prior node
	    
	    //cout<<"vertex_to_become_prior->hessian is = "<<endl<<vertex_to_become_prior->hessian(0,0)<<" "<<vertex_to_become_prior->hessian(0,1)<<"  "<<vertex_to_become_prior->hessian(0,2)<<endl;
	    //cout<<vertex_to_become_prior->hessian(1,0)<<"  "<<vertex_to_become_prior->hessian(1,1)<<"  "<<vertex_to_become_prior->hessian(1,2)<<endl;
	    //cout<<vertex_to_become_prior->hessian(2,0)<<"  "<<vertex_to_become_prior->hessian(2,1)<<"  "<<vertex_to_become_prior->hessian(2,2)<<endl;	
	    Eigen::Matrix<double, 4, 4> Hessian;      
	    if(vertex_to_become_prior->isOptimizedAtLeastOnce)
	    {	
	      Hessian<<
	      vertex_to_become_prior->hessian(0,0), vertex_to_become_prior->hessian(0,1), vertex_to_become_prior->hessian(0,2),	vertex_to_become_prior->hessian(0,3),
	      vertex_to_become_prior->hessian(1,0),vertex_to_become_prior->hessian(1,1), vertex_to_become_prior->hessian(1,2),vertex_to_become_prior->hessian(1,3),
	      vertex_to_become_prior->hessian(2,0),vertex_to_become_prior->hessian(2,1), vertex_to_become_prior->hessian(2,2),vertex_to_become_prior->hessian(2,3),
	      vertex_to_become_prior->hessian(3,0),vertex_to_become_prior->hessian(3,1), vertex_to_become_prior->hessian(3,2),vertex_to_become_prior->hessian(3,3);
	      //Hessian<<50000, 0.0, 0.0, 0.0,50000, 0.0,0.0,0.0, 500000; 
	      //New hessian is an exponentially decayed version of the original
	      //double decayCoeff = 0.1;
	      Hessian = Hessian*(exp(-decayCoeff*WINDOW_SIZE_TGT)); //decay is exponentially with time.
	    }
	    else
	    {
	      Hessian<<
		  0.001, 0, 0, 0, 
		  0, 0.001, 0, 0,
		  0, 0, 0.0001, 0,
		  0, 0, 0, 0.0001;
	    }
	    
	    graph_ptr->removeVertex(vertex_to_become_prior,/*bool detach=*/false);
	    
	    //Now connect the fixed prior node to the first node of the rest of the seequence through a very "loose" edge! An edge that means we have poorinformation regarding the prior
		    
	    EdgeXY_VXVY * edgeFirst_to_Prior = new EdgeXY_VXVY;
	    VertexXY_VXVY* firstTargetnode = dynamic_cast<VertexXY_VXVY*>(graph_ptr->vertices()[tgtVertexID-WINDOW_SIZE_TGT+1]); //remember it is the next node after the removed one
	    
	    edgeFirst_to_Prior->vertices()[0] = v_target_prior;
	    edgeFirst_to_Prior->vertices()[1] = firstTargetnode;	  

	    // set the observation and imformation matrix
	    Eigen::Vector4d diff = Eigen::Vector4d(0.0,0.0,0.0,0.0);  //measurement to connect the first node to prior node   
	    edgeFirst_to_Prior->setMeasurement(diff);
	    edgeFirst_to_Prior->information() = Hessian;      
	    // finally add the edge to the graph
	    if(!graph_ptr->addEdge(edgeFirst_to_Prior))
	    {
	      assert(false);
	    } 
	    
	    //Calculating Arrival Cost at T-N
	    
	    //cout<<"Hessian is = "<<endl<<
		//  Hessian(0,0)<<"  "<<Hessian(0,1)<<"  "<<Hessian(0,2)<<"  "<<Hessian(0,3)<<endl;
	    //cout<<Hessian(1,0)<<"  "<<Hessian(1,1)<<"  "<<Hessian(1,2)<<"  "<<Hessian(0,3)<<endl;
	    //cout<<Hessian(2,0)<<"  "<<Hessian(2,1)<<"  "<<Hessian(2,2)<<"  "<<Hessian(0,3)<<endl;	
	    //cout<<Hessian(3,0)<<"  "<<Hessian(3,1)<<"  "<<Hessian(3,2)<<"  "<<Hessian(0,3)<<endl;		    
	    //cout<<"error.transpose()*Hessian*error = "<<error.transpose()*Hessian*error<<endl;
	    //cout<<"Hessian.determinant() = "<<Hessian.determinant()<<endl;
	} 

   }  
      
    // Now add the robot state node to target state node observation edge. Here it will connect to the latest target node if it is made as per above part of the method
    Eigen::Vector2d tempBallObsVec = Eigen::Vector2d(tgtX,tgtY);
    // add information matrix
    Eigen::Matrix<double, 2, 2> tempInformationOnTarget;
    
    double range = tempBallObsVec.norm(),
	   bearing = atan2(tgtY,tgtX);
    //here we calculate the information matrix.

    
    double covDD = K3*(range*range);
    double covPhiPhi = K4*(1/(range+1));
    double covXX = pow(cos(bearing),2) * covDD + pow(sin(bearing),2) * ( pow(range,2) * covPhiPhi + covDD * covPhiPhi );
    double covYY = pow(sin(bearing),2) * covDD + pow(cos(bearing),2) * ( pow(range,2) * covPhiPhi + covDD * covPhiPhi );

    
    
//     double d = tempBallObsVec.norm(),
// 	    phi = atan2(tgtY,tgtX);
//     
//     double covDD = (double)(1/tgtMismatchFac)*(K3*d + K4*(d*d));
//     double covPhiPhi = K5*(1/(d+1));
//     
//     double covXX = pow(cos(phi),2) * covDD 
// 				+ pow(sin(phi),2) * ( pow(d,2) * covPhiPhi + covDD * covPhiPhi );
//     double covYY = pow(sin(phi),2) * covDD 
// 				+ pow(cos(phi),2) * ( pow(d,2) * covPhiPhi + covDD * covPhiPhi );
    
				

    if(covXX<=0||covYY<=0)
      ROS_WARN(" covariance negative!!! ");
    tempInformationOnTarget<<1/covXX,0,
				0,1/covYY;
    
    EdgeSE2_XY_VXVY * e = new  EdgeSE2_XY_VXVY();
    // retrieve vertex pointers from graph with id's
    g2o::OptimizableGraph::Vertex * pose_a_vertex
	    = dynamic_cast<g2o::OptimizableGraph::Vertex*>
	      (graph_ptr->vertices()[robotVertexID]);//current robot pose vertex
    g2o::OptimizableGraph::Vertex * pose_b_vertex
	    = dynamic_cast<g2o::OptimizableGraph::Vertex*>
	      (graph_ptr->vertices()[tgtVertexID]);//landmark Vertex	
    
    if(pose_a_vertex!=NULL && pose_a_vertex->dimension() == 3)		
    {
      e->vertices()[0] = pose_a_vertex;
    }
    else
    {
      ROS_WARN(" Robot pose does not exist at target observation edge");
  // 	break;
    }   	
      
    if(pose_b_vertex!=NULL && pose_b_vertex->dimension() == 4)
    {
      e->vertices()[1] = pose_b_vertex;
    }
    else
    {
      ROS_WARN(" Ball position does not exist ??? ");
  // 	break;
    }   	
	      
    
    e->setMeasurement(tempBallObsVec);
    e->information() = tempInformationOnTarget;   
    if(!graph_ptr->addEdge(e))
    {
      ROS_WARN(" Edge not added because it may already exist ");
    }    
    
       
      
}



////////////////////////// METHOD DEFINITIONS OF THE SELFROBOT CLASS //////////////////////

void SelfRobot::selfOdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry, int RobotNumber, g2o::SparseOptimizer* graph_ptr)
{
  //boost::mutex::scoped_lock lock(graph_ptr);
  
  uint seq = odometry->header.seq;
  prevTime = curTime;
  curTime = odometry->header.stamp;
  
  Eigen::Vector2d t;
  t = curPose.translation();
  Eigen::Matrix<double,2,2> rotation = curPose.linear();
  double curAngle = acos(rotation(0,0));  
  curAngle = normalizeAngle(curAngle);
  
  //ROS_INFO(" got odometry from self-robot (ID=%d) at time %d\n",RobotNumber, odometry->header.stamp.sec);  

  //Below is an example how to extract the twist variables from the odometry message and use it to propagate the robot state by using a simple motion model without noise.
  
  double linearVel = odometry->twist.twist.linear.x;
  double angularVel = odometry->twist.twist.angular.x;
  double deltaX=0,deltaY=0,deltaTheta=0;
  
  Duration delta = curTime - prevTime;
  double deltaSecs = delta.toSec();
  //if(deltaSecs<0)
    //cout<<"Loooooookkkkkkkkkk.....deltaSecs = "<<deltaSecs<<endl;
  
    deltaX = linearVel*deltaSecs*cos(angularVel*deltaSecs);
    deltaY = linearVel*deltaSecs*sin(angularVel*deltaSecs);
    deltaTheta = angularVel*deltaSecs;  
  
//   if(angularVel == 0)
//   {
//     deltaX = linearVel*deltaSecs;
//     deltaY=0;deltaTheta=0;
//   }
//   else
//   {
//     double r = linearVel/angularVel;
//     
//     deltaX = (-r)*sin(curAngle) + (r)*sin(curAngle+angularVel*deltaSecs);
//     deltaY = (r)*cos(curAngle)  + (-r)*cos(curAngle+angularVel*deltaSecs);
//     deltaTheta = angularVel*deltaSecs;
//   }

//   double odomVector[3] = {deltaX, deltaY, deltaTheta};
//   
//   Eigen::Isometry2d odom; 
//   odom = Eigen::Rotation2Dd(odomVector[2]).toRotationMatrix();
//   odom.translation() = Eigen::Vector2d(odomVector[0], odomVector[1]);  
// 
//   if(seq == 0)
//     curPose = initPose;
//   else
//   {
//     prevPose = curPose;
//     curPose = prevPose*odom;
//   }
   
//   Eigen::Vector2d posterior;
//   posterior = curPose.translation();
//   Eigen::Matrix<double,2,2> posteriorRot = curPose.linear();
//   double posteriorAngle = acos(posteriorRot(0,0));
//   
//   publishState(posterior(0),posterior(1),posteriorAngle); 
//   
//   ROS_INFO("Odometry propagated self robot state is x=%f, y=%f, theta=%f",posterior(0),posterior(1),posteriorAngle);

    if((*totalVertextCounter)<MAX_VERTEX_COUNT)
      {

	double odomVector[3] = {deltaX, deltaY, deltaTheta};
	//ROS_INFO("Odometry is dx=%f, dy=%f, dTheta=%f",deltaX, deltaY, deltaTheta);	
	
	Eigen::Isometry2d odom; 
	odom = Eigen::Rotation2Dd(odomVector[2]).toRotationMatrix();
	odom.translation() = Eigen::Vector2d(odomVector[0], odomVector[1]);    
	
	if(vertextCounter_ == 0)
	{
	  curPose = initPose;
	  avgComputationTime = 0.0;
	}
	else
	{
	  prevPose = curPose;
	  curPose = prevPose*odom;
	}
	
	
	//Create Self Robot Pose Vertex
	// vertext id for the robots start at MAX_INDIVIDUAL_STATES times the robot number
	if(SE2vertexID == 0)
	  SE2vertexID = (RobotNumber)*MAX_INDIVIDUAL_STATES;
	else
	  SE2vertexID++;
	
	VertexSE2 * v = new VertexSE2();
	v->setId(SE2vertexID);  
	v->setEstimate(curPose);
	v->timestamp =  (unsigned long long) 1000000*curTime.sec + curTime.nsec/1000;
	v->agentID = RobotNumber;
	v->isOptimizedAtLeastOnce = false;
	v->odometryReading = odom;
	//if(vertextCounter_ == 0)
	//v->setFixed(true);
	graph_ptr->addVertex(v);
	currentPoseVertexIDs[RobotNumber-1] = SE2vertexID;
	//cout<<"currentPoseVertexIDs[RobotNumber] = "<<currentPoseVertexIDs[RobotNumber-1]<<endl;
	
	//ROS_INFO("Breaking here");
	
	//start adding self-robot pose-pose edges if seq is greater than 0
	if(vertextCounter_>0) 
	{
	  SE2vertexID_prev = SE2vertexID - 1;
	  v->setFixed(false);
	  
	  //Now add edge from the current node to the previous node
	  
	  //std::cout <<"SE2 Edge from to \n";		
	  //std::cout <<SE2vertexID_prev<<"\n"; 		
	  //std::cout <<SE2vertexID<<"\n"; 
	  
	  EdgeSE2 * e = new EdgeSE2;
	  // retrieve vertex pointers from graph with id's
	  g2o::OptimizableGraph::Vertex * pose_a_vertex
		  = dynamic_cast<g2o::OptimizableGraph::Vertex*>
		    (graph_ptr->vertices()[SE2vertexID_prev]);
	  g2o::OptimizableGraph::Vertex * pose_b_vertex
		  = dynamic_cast<g2o::OptimizableGraph::Vertex*>
		    (graph_ptr->vertices()[SE2vertexID]);			
		    			
	  // error check vertices and associate them with the edge
	  assert(pose_a_vertex!=NULL);
	  assert(pose_a_vertex->dimension() == 3);
	  e->vertices()[0] = pose_a_vertex;

	  assert(pose_b_vertex!=NULL);
	  assert(pose_b_vertex->dimension() == 3);
	  e->vertices()[1] = pose_b_vertex;	  
		
	  //cout<<"odometry values = "<<odomVector[0]<<" "<<odomVector[1]<<" "<<odomVector[2]<<endl;
	  
	  
	  
	  
	  // add information matrix
	  Eigen::Matrix<double, 3, 3> Lambda;      
	  //Lambda<<500, 0.0, 0.0, 0.0,500, 0.0,0.0,0.0, 50;      
	  makeOdomSigmas(odomVector[0], odomVector[1],odomVector[2]);
	  //cout<<"invSigmas = "<<invSigmaX<<" "<<invSigmaY<<" "<<invSigmaTheta<<endl;
	  Lambda<<invSigmaX, 0.0, 0.0, 0.0,invSigmaY, 0.0,0.0,0.0, invSigmaTheta;
	  
	  // set the observation and imformation matrix
	  const SE2 a(odom);
	  e->setMeasurement(a);
	  e->information() = Lambda;      
	  
	  // finally add the edge to the graph
	  if(!graph_ptr->addEdge(e))
	  {
	    assert(false);
	  } 
	  
	}
	else ///First node is not set as a fixed node.. A prior node with 0 mean and high covariance is created and attached to the first node with a very loose edge
	{
	  v->setFixed(false); 
	  
	  Eigen::Isometry2d priorPose;
	  VertexSE2* firstRobotnode = dynamic_cast<VertexSE2*>(graph_ptr->vertices()[SE2vertexID]);
	  priorPose = Eigen::Rotation2Dd(0);
	  priorPose.translation() = Eigen::Vector2d(0,0); 
	  
	  priorPose = initPose;
	  
	  //Create the Prior SE2 Node now. This node will keep track of the past information.
	  VertexSE2 * v_prior = new VertexSE2();
	  v_prior->setId((RobotNumber)*MAX_INDIVIDUAL_STATES - 1); //The ID is one before the first node pose of this robot  
	  v_prior->setEstimate(priorPose);
	  v_prior->timestamp =  firstRobotnode->timestamp;
	  v_prior->agentID = RobotNumber;
	  v_prior->setFixed(true);
	  graph_ptr->addVertex(v_prior);
	  
	  //Now connect the fixed prior node to the first node through a very "loose" edge! An edge that means we have poorinformation regarding the prior
	  EdgeSE2 * edgeFirst_to_Prior = new EdgeSE2;	
	  edgeFirst_to_Prior->vertices()[0] = v_prior;
	  edgeFirst_to_Prior->vertices()[1] = firstRobotnode;	  

	  // add information matrix
	  Eigen::Matrix<double, 3, 3> Lambda;      
	  Lambda<<0.001, 0.0, 0.0, 0.0,0.001, 0.0,0.0,0.0, 0.0001;
	  //Lambda<<500000, 0.0, 0.0, 0.0,500000, 0.0,0.0,0.0, 5000000;
	  
	  // set the observation and imformation matrix
	  Eigen::Isometry2d tempMean; tempMean = Eigen::Rotation2Dd(0); tempMean.translation() = Eigen::Vector2d(0,0);   
	  edgeFirst_to_Prior->setMeasurement(tempMean);
	  edgeFirst_to_Prior->information() = Lambda;      
	  // finally add the edge to the graph
	  if(!graph_ptr->addEdge(edgeFirst_to_Prior))
	  {
	    assert(false);
	  } 
	  
	  //std::cout <<"First SE2 Edge from to \n";		
	  //std::cout <<(RobotNumber)*MAX_INDIVIDUAL_STATES - 1<<"\n"; 		
	  //std::cout <<SE2vertexID<<"\n"; 	  
	  
	  //U_prior = Lambda.determinant();
	  
	}    

      vertextCounter_++; 
      (*totalVertextCounter)++;
      
      ///... here is where the sliding window solver is invoked..
      
      cout<<"Total robot(s) state vertices = "<< (*totalVertextCounter) <<endl;
      //cout<<"Total target state vertices = "<< targetsToTrack[0]->tgtVertexCounter<<endl;
      //cout<<"Target state current ID = "<< targetsToTrack[0]->tgtVertexID <<endl;
      
      
      if(vertextCounter_ > WINDOW_SIZE)
      {
	  if(vertextCounter_ == MAX_VERTEX_COUNT-5)
	  {
	    cout<<"avgComputationTime = "<<avgComputationTime<<endl;
	    cout<<"windowSolverInvokeCount = "<<windowSolverInvokeCount<<endl;	
	    //compute average time of solver per iteration of the estimator
	    avgComputationTime = avgComputationTime/windowSolverInvokeCount;
	    printf("\n\nverage time taken by the solver per iteration of the estimator = %f\n\n\n",avgComputationTime);
	  }     
	  //Now solve the resulting graph
	  timespec ts1,ts2;
	  clock_gettime(CLOCK_REALTIME, &ts1);
	  //cout<<"some time =  "<< ts.tv_nsec <<" (nano)seconds"<<'\n';

	  solveSlidingWindowGraph(graph_ptr);
	  

	  clock_gettime(CLOCK_REALTIME, &ts2);
	  float diff_millisec = fabs(((float)ts2.tv_nsec - (float)ts1.tv_nsec)/1000000);
	  //cout<<"time taken =  "<< diff_millisec <<" milli seconds"<<'\n';
	  
	  computationTime[vertextCounter_] = diff_millisec;
	  
	  if(diff_millisec<30) //removing outliers
	  {
	    avgComputationTime += diff_millisec;
	    windowSolverInvokeCount++;
	  }
	  
	  
	  //cout<<"avgComputationTime = "<<avgComputationTime<<endl;
	  //cout<<"windowSolverInvokeCount = "<<windowSolverInvokeCount<<endl;
	  
	  //cout<<"I am here"<<endl;
	  Eigen::Isometry2d priorPose;
	  
	  // Prior (Mean) will be set to the vertex pose that is going to be deleted in the sliding window case 
	  VertexSE2* vertex_to_become_prior = dynamic_cast<VertexSE2*>(graph_ptr->vertices()[SE2vertexID-WINDOW_SIZE]);
	  priorPose = vertex_to_become_prior->estimate().toIsometry();
	  
	  //We now have to remove the last added prior node and a new prior node that contains the updated past. One may simply update the previous prior node but seems like it is not so straightforward to do so using g2o. Therefore I am copying the prious prior, deleting it, updating locally and then adding a new prior node which is the updated prior node
	  VertexSE2* v_prior_old = dynamic_cast<VertexSE2*>(graph_ptr->vertices()[(RobotNumber)*MAX_INDIVIDUAL_STATES - 1]);
	  graph_ptr->removeVertex(v_prior_old,/*bool detach=*/false);
	  
	  VertexSE2 * v_prior = new VertexSE2();
	  v_prior->setId((RobotNumber)*MAX_INDIVIDUAL_STATES - 1); //The ID is one before the first node pose of this robot  
	  v_prior->setEstimate(priorPose);
	  v_prior->timestamp =  vertex_to_become_prior->timestamp;
	  v_prior->agentID = RobotNumber;
	  v_prior->setFixed(true);
	  graph_ptr->addVertex(v_prior);
	  
      
	  //Now remove the oldest node (vertex_to_become_prior), which has already been used to update the new prior node to maintain the sliding window. However, grab the last hessian block of this vertex. This hessian will become the information contained by the prior node
	  
	  //cout<<"vertex_to_become_prior->hessian is = "<<endl<<vertex_to_become_prior->hessian(0,0)<<" "<<vertex_to_become_prior->hessian(0,1)<<"  "<<vertex_to_become_prior->hessian(0,2)<<endl;
	  //cout<<vertex_to_become_prior->hessian(1,0)<<"  "<<vertex_to_become_prior->hessian(1,1)<<"  "<<vertex_to_become_prior->hessian(1,2)<<endl;
	  //cout<<vertex_to_become_prior->hessian(2,0)<<"  "<<vertex_to_become_prior->hessian(2,1)<<"  "<<vertex_to_become_prior->hessian(2,2)<<endl;	
	  Eigen::Matrix<double, 3, 3> Hessian;      
	  Hessian<<vertex_to_become_prior->hessian(0,0), vertex_to_become_prior->hessian(0,1), vertex_to_become_prior->hessian(0,2), vertex_to_become_prior->hessian(1,0),vertex_to_become_prior->hessian(1,1), vertex_to_become_prior->hessian(1,2),vertex_to_become_prior->hessian(2,0),vertex_to_become_prior->hessian(2,1), vertex_to_become_prior->hessian(2,2);
	  //Hessian<<50000, 0.0, 0.0, 0.0,50000, 0.0,0.0,0.0, 500000; 
	  //New hessian is an exponentially decayed version of the old
	  //decayCoeff = 0.1;
	  
	  double actualDecayCoeff = (-1.0)*decayCoeff*DECAY_LAMBDA;
	  //cout<<" actualDecayCoeff = "<<actualDecayCoeff<<endl;
	  
	  Hessian = Hessian*(exp(actualDecayCoeff)); //decay is exponentially with time.
	  
	  graph_ptr->removeVertex(vertex_to_become_prior,/*bool detach=*/false);
	  
	  //Now connect the fixed prior node to the first node of the rest of the seequence through a very "loose" edge! An edge that means we have poorinformation regarding the prior
		  
	  EdgeSE2 * edgeFirst_to_Prior = new EdgeSE2;
	  VertexSE2* firstVertex = dynamic_cast<VertexSE2*>(graph_ptr->vertices()[SE2vertexID-WINDOW_SIZE+1]); //remember it is the next node after the removed one
	  
	  edgeFirst_to_Prior->vertices()[0] = v_prior;
	  edgeFirst_to_Prior->vertices()[1] = firstVertex;	  

	  // set the observation and imformation matrix
	  Eigen::Isometry2d odomFromFirstVertex = firstVertex->odometryReading; //measurement to connect the first node to prior node   
	  //cout<<"odomFromFirstVertex = "<<odomFromFirstVertex.translation().x()<<" , "<<odomFromFirstVertex.translation().y()<<" agent ID = "<<firstVertex->agentID<<endl;
	  edgeFirst_to_Prior->setMeasurement(odomFromFirstVertex);
	  edgeFirst_to_Prior->information() = Hessian;      
	  // finally add the edge to the graph
	  if(!graph_ptr->addEdge(edgeFirst_to_Prior))
	  {
	    assert(false);
	  } 
	  
	  //Calculating Arrival Cost at T-N
	  double x_t = v_prior->estimate().translation().x();
	  double y_t = v_prior->estimate().translation().y();
	  double theta_t = v_prior->estimate().rotation().angle();
	  
	  double x_t_1 = firstVertex->estimate().translation().x();
	  double y_t_1 = firstVertex->estimate().translation().y();
	  double theta_t_1 = firstVertex->estimate().rotation().angle();
	  
	  Eigen::Vector3d delta = Eigen::Vector3d(odomVector[0], odomVector[1], odomVector[2]);
	  
	  Eigen::Vector3d h_t_t_1 = Eigen::Vector3d((x_t_1-x_t)*cos(theta_t) + (y_t_1-y_t)*sin(theta_t),
	  -(x_t_1-x_t)*sin(theta_t) + (y_t_1-y_t)*cos(theta_t),
	    normalizeAngle(theta_t_1-theta_t)
	  );
	  
	  Eigen::Vector3d error = Eigen::Vector3d((delta(0)-h_t_t_1(0))*cos(h_t_t_1(2)) + (delta(1)-h_t_t_1(1))*sin(h_t_t_1(2)),
	  -(delta(0)-h_t_t_1(0))*sin(h_t_t_1(2)) + (delta(1)-h_t_t_1(1))*cos(h_t_t_1(2)),
	    normalizeAngle((delta(2)-h_t_t_1(2)))
	  );
	  
	  //       cout<<"Hessian is = "<<endl<<
	  //             Hessian(0,0)<<"  "<<Hessian(0,1)<<"  "<<Hessian(0,2)<<endl;
	  //       cout<<Hessian(1,0)<<"  "<<Hessian(1,1)<<"  "<<Hessian(1,2)<<endl;
	  //       cout<<Hessian(2,0)<<"  "<<Hessian(2,1)<<"  "<<Hessian(2,2)<<endl;	
	  //cout<<"error.transpose()*Hessian*error = "<<error.transpose()*Hessian*error<<endl;
	  //cout<<"Hessian.determinant() = "<<Hessian.determinant()<<endl;
	  //double beta_t_n = U[vertextCounter_-1-WINDOW_SIZE]/();
      }
      else
	if(vertextCounter_ >1)
	{
	    //simply solve it
	    solveSlidingWindowGraph(graph_ptr);
	    //U[vertextCounter_-1] = graph_ptr->chi2() + U_prior;
	  
	}   
      
      }
      
      else
      {
	
	//Solve once more before exiting
	fclose(mhls_utias_g2o);
	fclose(resultsFile);
	//solveSlidingWindowGraph(graph_ptr);
	(*totalVertextCounter) = 0;
	//std::cout << "Saving original graph to temp_dwa_graph.g2o...\n";
	cout << "Experiment finished...\n\n\n";
	//graph_ptr->save("utias_dwa_graph.g2o");  
	graph_ptr->clear();
	ros::shutdown();
      }
  
  
}


void SelfRobot::selfMeasurementDataCallback(const utiasdata_to_rosbags::MCLAMMeasurementData::ConstPtr& data, int RobotNumber, g2o::SparseOptimizer* graph_ptr)
{
  //ROS_INFO("Got ball data from self robot %d",RobotNumber);  
  Time curObservationTime = data->header.stamp;
  
  bool subjectIsLandmark = false;
  bool subjectIsTeammate = false;
  int subject = 99;// we have to figure this out from the barcode so we initialize it with a non-exixting subject, e.g., 99
  int subjectBarcode = data->subjectBarcode;
  double range = data->range - 0.08;
  double bearing = data->bearing;
  
  if(range<=0)
    range = data->range;
  
  
  for(int i=0;i<20;i++)
  {
    //printf("checking with barcode %d \t",(*subject_barcode)[i]);
    if((*subject_barcode)[i]==subjectBarcode && i>4)
    {
      subject=i;
      subjectIsLandmark = true;
    }
    
    if((*subject_barcode)[i]==subjectBarcode &&  i<=4)
    {
      subject=i;
      subjectIsTeammate = true;
    }    
  }
  
  //cout<<"subjectBarcode found, id = "<<subjectBarcode<<endl;
  
  if(subjectIsTeammate && currentPoseVertexIDs[subject]>0 && COP_LOC) //check if bar code belongs to a teammate robot which is activated
  {
   addRobotRobotEdge(subjectBarcode, range, bearing, SE2vertexID, currentPoseVertexIDs[subject] ,graph_ptr);
  }    
  
  //If the subject is not a landmark but the target robot
  if(subjectBarcode==23 && NUM_TARGETS>0) //check the header file for fixed subjectBarcodes for the target
  {
   
    //ROS_WARN("                                             Target Robot 5 Detected ");
    double localX = range*cos(bearing);
    double localY = range*sin(bearing);
    
    targetsToTrack[0]->addNewTGTVertexAndEdgesAround(0 /*targetID*/ , localX,localY,RobotNumber, SE2vertexID, curObservationTime, graph_ptr);    
    
  }
  
  if(subjectBarcode==32 && NUM_TARGETS>1) //check the header file for fixed subjectBarcodes for the target
  {
   
    //ROS_WARN("                                             Target Robot 4 Detected ");
    double localX = range*cos(bearing);
    double localY = range*sin(bearing);
    
    targetsToTrack[1]->addNewTGTVertexAndEdgesAround(1 /*targetID*/,localX,localY,RobotNumber, SE2vertexID, curObservationTime, graph_ptr);    
    
  } 
  
  if(subjectBarcode==41 && NUM_TARGETS>2) //check the header file for fixed subjectBarcodes for the target
  {
   
    //ROS_WARN("                                             Target Robot 3 Detected ");
    double localX = range*cos(bearing);
    double localY = range*sin(bearing);
    
    targetsToTrack[2]->addNewTGTVertexAndEdgesAround(2 /*targetID*/,localX,localY,RobotNumber, SE2vertexID, curObservationTime, graph_ptr);    
    
  }    
  
  //If the subject is a landmark and not another robot
  if(subjectIsLandmark && USE_LANDMARKS)
  {
    double localX = range*cos(bearing);
    double localY = range*sin(bearing);
    
    Eigen::Vector2d tempLandmarkObsVec = Eigen::Vector2d(localX,localY);
    Eigen::Matrix<double, 2, 2> tempInformationOnLandmark;
    
    double covDD = K1;//K1*(range*range);
    double covPhiPhi = K2;//K2*range;
    double covXX = pow(cos(bearing),2) * covDD + pow(sin(bearing),2) * ( pow(range,2) * covPhiPhi + covDD * covPhiPhi );
    double covYY = pow(sin(bearing),2) * covDD + pow(cos(bearing),2) * ( pow(range,2) * covPhiPhi + covDD * covPhiPhi );    
    
    if(covXX<0||covYY<0)
	ROS_WARN(" covariance negative!!! ");
      
    tempInformationOnLandmark<<1/covXX,0,
				 0,1/covYY;    
    
    //cout<<"covddphiphi"<<endl;
    //cout<<covDD<<"\t"<<0<<endl<<0<<"\t"<<covPhiPhi<<endl<<endl; 				 
    //cout<<"covxy"<<endl;
    //cout<<covXX<<"\t"<<0<<endl<<0<<"\t"<<covYY<<endl<<endl; 				 
    
    EdgeSE2PointXY * e = new  EdgeSE2PointXY();
    // retrieve vertex pointers from graph with id's
    g2o::OptimizableGraph::Vertex * pose_a_vertex
	    = dynamic_cast<g2o::OptimizableGraph::Vertex*>
	      (graph_ptr->vertices()[SE2vertexID]);//current robot pose vertex
    g2o::OptimizableGraph::Vertex * pose_b_vertex
	    = dynamic_cast<g2o::OptimizableGraph::Vertex*>
	      (graph_ptr->vertices()[subject]);//landmark Vertex is the subject number identified using the subjectBarcode earlier in this function	
    
    if(pose_a_vertex!=NULL && pose_a_vertex->dimension() == 3)		
      e->vertices()[0] = pose_a_vertex;
    else
    {
      ROS_WARN(" Robot pose does not exist ");
      return;
    }   	
      
    if(pose_b_vertex!=NULL && pose_b_vertex->dimension() == 2)
      e->vertices()[1] = pose_b_vertex;
    else
    {
      ROS_WARN(" Landmark does not exist ??? ");
      return;
    }   	
	      
    
    e->setMeasurement(tempLandmarkObsVec);
    e->information() = tempInformationOnLandmark;   
    if(!graph_ptr->addEdge(e))
    {
 	ROS_WARN(" Edge not added because it may already exist ");
    }       
  }
  
}


void SelfRobot::selfGTCallback(const geometry_msgs::PoseStamped::ConstPtr& data, int RobotNumber, g2o::SparseOptimizer* graph_ptr)
{
  //ROS_INFO(" got landmark data from self robot (ID=%d)",RobotNumber);  
  
  uint seq = data->header.seq;
  
  if(notInitialized)
  {
    initPose = Eigen::Rotation2Dd(normalizeAngle(tf::getYaw(data->pose.orientation))).toRotationMatrix();
    initPose.translation() = Eigen::Vector2d(data->pose.position.x,data->pose.position.y); 

    notInitialized = false;
  }
  
    curGTPose = Eigen::Rotation2Dd(normalizeAngle(tf::getYaw(data->pose.orientation))).toRotationMatrix();
    curGTPose.translation() = Eigen::Vector2d(data->pose.position.x,data->pose.position.y);  
    
    (*teamGTPose)[RobotNumber-1] =  curGTPose;
  
//   ROS_INFO("Odometry propagated self robot state is x=%f, y=%f, theta=%f",posterior(0),posterior(1),posteriorAngle);  
  
//   ROS_INFO("Ground Truth self robot state is x=%f, y=%f, theta=%f",truePose(0),truePose(1),truerAngle);    
  
}


void SelfRobot::solveSlidingWindowGraph(g2o::SparseOptimizer* graph_ptr)
{
      /// start Optimizatin here
     
      char filename[100];
      sprintf(filename, "original_dwa_graph_%d.g2o", solverStep);
#ifdef  SAVE_GRAPHFILES
      graph_ptr->save(filename);
#endif
      std::string strSolver = "lm_var_cholmod";
      g2o::OptimizationAlgorithmFactory* solverFactory = g2o::OptimizationAlgorithmFactory::instance();
      g2o::OptimizationAlgorithmProperty solverProperty;
      //solverFactory->listSolvers(cerr);
      graph_ptr->setAlgorithm(solverFactory->construct(strSolver, solverProperty));    
      
      string strRobustKernel = "Cauchy";
      AbstractRobustKernelCreator* creator = RobustKernelFactory::instance()->creator(strRobustKernel);
      for (SparseOptimizer::EdgeSet::const_iterator it = graph_ptr->edges().begin(); it != graph_ptr->edges().end(); ++it) 
      {
	OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
	  e->setRobustKernel(creator->construct());
	  e->robustKernel()->setDelta(1.0);
      }    
      
      graph_ptr->setVerbose(false);
      bool flagtostop = 0;
      graph_ptr->setForceStopFlag(&flagtostop);
      int maxIterations = MAX_ITERATIONS;  

      if (! graph_ptr->solver()) {
	std::cout << "Error allocating solver. Allocating \"" << strSolver << "\" failed!" << std::endl;
      }  

      graph_ptr->initMultiThreading;
      graph_ptr->initializeOptimization();
      graph_ptr->computeActiveErrors();

      int result = graph_ptr->optimize(maxIterations);
      if (maxIterations > 0 && result==g2o::OptimizationAlgorithm::Fail){
	std::cout << "Optimization failed at iteration = " << result<<", result might be invalid" << std::endl;
      }
      else
      {
	double finalChi=graph_ptr->chi2();
	//std::cout << "Optimization complete with final ChiSquare value = "<<finalChi<<" and iterations = "<< result<<std::endl; 
      }
      
      sprintf(filename, "solved_dwa_graph_%d.g2o", solverStep);
#ifdef  SAVE_GRAPHFILES
      graph_ptr->save(filename);
#endif 
      
      
      VertexSE2* mostRecetPoseVertex = dynamic_cast<VertexSE2*>(graph_ptr->vertices()[SE2vertexID]);
      
      // extracting most recent target position
      if(NUM_TARGETS>0)
      {
	if((targetsToTrack[0]->tgtVertexCounter>0 && graph_ptr->vertices()[targetsToTrack[0]->tgtVertexID]))
	{
	  VertexXY_VXVY * mostRecetTargetVertex = dynamic_cast<VertexXY_VXVY*>(graph_ptr->vertices()[targetsToTrack[0]->tgtVertexID]);
	  mostRecetTargetVertex->isOptimizedAtLeastOnce=true;
	  //cout<<"Most recent Ball Vertex is "<<targetsToTrack[0]->tgtVertexID<<endl;
	  
	  targetsToTrack[0]->prevPose(0) = targetsToTrack[0]->curPose(0);
	  targetsToTrack[0]->prevPose(1) = targetsToTrack[0]->curPose(1); 
	  targetsToTrack[0]->prevPose(2) = targetsToTrack[0]->curPose(2);
	  
	  targetsToTrack[0]->curPose(0) = ((VertexXY_VXVY*)mostRecetTargetVertex)->estimate().x();
	  targetsToTrack[0]->curPose(1) = ((VertexXY_VXVY*)mostRecetTargetVertex)->estimate().y();
	  targetsToTrack[0]->curPose(2) = ((VertexXY_VXVY*)mostRecetTargetVertex)->ballZCoordinate;
	  //cout<<"Ball is at "<<targetsToTrack[0]->curPose(0)<<" and "<< targetsToTrack[0]->curPose(1)<<endl;
	  
	}
	else
	{
	  targetsToTrack[0]->curPose(0) = targetsToTrack[0]->prevPose(0);
	  targetsToTrack[0]->curPose(1) = targetsToTrack[0]->prevPose(1); 
	}
      }      
      
      // extracting most recent self pose
      //cout<<"Most recent Pose theta is = "<<mostRecetPoseVertex->estimate().rotation().angle()<<endl;
      curPose = mostRecetPoseVertex->estimate().toIsometry();      
      
      Eigen::Vector2d truePose;
      truePose = curGTPose.translation();
      Eigen::Matrix<double,2,2> trueRot = curGTPose.linear();
      double trueAngle = acos(trueRot(0,0));
      
      
      Eigen::Vector2d posterior;
      posterior = curPose.translation();
      Eigen::Matrix<double,2,2> posteriorRot = curPose.linear();
      double posteriorAngle = acos(posteriorRot(0,0));  
      
      double errorPos = pow((posterior(0)-truePose(0))*(posterior(0)-truePose(0)) + (posterior(1)-truePose(1))*(posterior(1)-truePose(1)),0.5);
      double errorOrient = fabs(posteriorAngle - trueAngle);
      
      publishState(graph_ptr); 
      
      //ROS_INFO("Estimated self robot state is x=%f, y=%f, theta=%f",posterior(0),posterior(1),posteriorAngle);  
  
      //ROS_INFO("Ground Truth self robot state is x=%f, y=%f, theta=%f",truePose(0),truePose(1),trueAngle);
      
      //cout <<"RMS error in position estimate = "<<errorPos<<endl;
      //cout <<"RMS error in orientation estimate = "<<errorOrient<<endl;
      
      std_msgs::Float32 errorPos1, errorOrient1;
      errorPos1.data = errorPos;
      errorOrient1.data = errorOrient;
      
      diagnosticError1.publish(errorPos1);
      diagnosticError2.publish(errorOrient1);
      //cout<<"mostRecetPoseVertex->hessian is = "<<endl<<mostRecetPoseVertex->hessian(0,0)<<" "<<mostRecetPoseVertex->hessian(0,1)<<"  "<<mostRecetPoseVertex->hessian(0,2)<<endl;
      //cout<<mostRecetPoseVertex->hessian(1,0)<<"  "<<mostRecetPoseVertex->hessian(1,1)<<"  "<<mostRecetPoseVertex->hessian(1,2)<<endl;
      //cout<<mostRecetPoseVertex->hessian(2,0)<<"  "<<mostRecetPoseVertex->hessian(2,1)<<"  "<<mostRecetPoseVertex->hessian(2,2)<<endl;
      //cout<<"Eigen::Rotation2Dd(((VertexSE2*)mostRecetPoseVertex)->estimate().rotation().angle()).toRotationMatrix() = "<<endl<<Eigen::Rotation2Dd(((VertexSE2*)mostRecetPoseVertex)->estimate().rotation().angle()).toRotationMatrix()<<endl;   
//       initPose = curPose;
//       publishSelfState(graph_ptr);
      
      
      //fixing the initial nodes
      
//       graph_ptr->clear();
//       addFixedLandmarkNodes(graph_ptr);
//       SE2vertexID = 0;
//       vertextCounter_ = 0;
//       (*totalVertextCounter)= 0;   
//       targetsToTrack[0]->tgtVertexCounter=0;
   
      solverStep++;  
}



void SelfRobot::publishState(g2o::SparseOptimizer* graph_ptr)
{
    //using the first robot for the globaltime stamp of this message
    msg.header.stamp = curTime; //time of the self-robot must be in the full state
    
        if(*totalVertextCounter<100) // do not publish if we dont have all robots running
          return;
    
    msg.header.stamp = curTime; //time of the self-robot must be in the full state
    //estimateBallState.header.stamp = curTime;
    //receivedGTdata.header.stamp = curTime;
    
    for(int i=0;i<MAX_ROBOTS;i++)
    {
      if(currentPoseVertexIDs[i]>0)
      {
      
	int latestOptimizedRobPoseVer = currentPoseVertexIDs[i];
	bool poseExists = false;
	//cout<<"latestOptimizedRobPoseVer = "<<latestOptimizedRobPoseVer<<endl;
	//This loop is to check whether the latest pose exists or not
	for(int j=0; j<WINDOW_SIZE;j++)
	{
	  if((graph_ptr->vertices()[latestOptimizedRobPoseVer]))
	  {
	    j=WINDOW_SIZE+1; //break and come out of the loop
	    poseExists = true;
	  }
	  else
	    latestOptimizedRobPoseVer--; //go one ID step below
	}
      
	if(poseExists)
	{
	  //cout<<"I am here and latestOptimizedRobPoseVer = "<<latestOptimizedRobPoseVer<<endl;
	  VertexSE2* mostRecetPoseVertex = dynamic_cast<VertexSE2*>(graph_ptr->vertices()[latestOptimizedRobPoseVer]);
	  mostRecetPoseVertex->isOptimizedAtLeastOnce = true;
	  
	  msg.robotEstimatedpose[i].pose.position.x = mostRecetPoseVertex->estimate().translation().x();
	  msg.robotEstimatedpose[i].pose.position.y = mostRecetPoseVertex->estimate().translation().y();
	  msg.robotEstimatedpose[i].pose.position.z = ROB_HT; //fixed height aboveground
	
	  msg.robotEstimatedpose[i].pose.orientation.x = 0;
	  msg.robotEstimatedpose[i].pose.orientation.y = 0;
	  msg.robotEstimatedpose[i].pose.orientation.z = sin(mostRecetPoseVertex->estimate().rotation().angle()/2);
	  msg.robotEstimatedpose[i].pose.orientation.w = cos(mostRecetPoseVertex->estimate().rotation().angle()/2);
	  
	  //fprintf(mhls_utias_g2o,"VERTEX_SE2 %d %f %f %f %llu %d\n",latestOptimizedRobPoseVer,msg.robotEstimatedpose[i].pose.position.x,msg.robotEstimatedpose[i].pose.position.y,mostRecetPoseVertex->estimate().rotation().angle(),mostRecetPoseVertex->timestamp,i+1);
	  
	  Eigen::Vector2d truePose;
	  truePose = (*teamGTPose)[i].translation();
	  Eigen::Matrix<double,2,2> trueRot = (*teamGTPose)[i].linear();
	  double trueAngle = acos(trueRot(0,0));
	    
	    
	  msg.robotGTpose[i].pose.position.x = truePose(0);
	  msg.robotGTpose[i].pose.position.y = truePose(1);
	  msg.robotGTpose[i].pose.position.z = ROB_HT; //fixed height aboveground
	  
	  msg.robotGTpose[i].pose.orientation.x = 0;
	  msg.robotGTpose[i].pose.orientation.y = 0;
	  msg.robotGTpose[i].pose.orientation.z = sin(trueAngle/2);
	  msg.robotGTpose[i].pose.orientation.w = cos(trueAngle/2);

	  	  
	  fprintf(resultsFile,"%d %f %f %f %f %f %f %llu ",i+1,
	      msg.robotEstimatedpose[i].pose.position.x,
	      msg.robotEstimatedpose[i].pose.position.y,
	      mostRecetPoseVertex->estimate().rotation().angle(),
		  msg.robotGTpose[i].pose.position.x,
		  msg.robotGTpose[i].pose.position.y,
		  trueAngle,
		  mostRecetPoseVertex->timestamp);
	}
	else
	{
	  fprintf(resultsFile,"%d %f %f %f %f %f %f %llu ",i+1,
	      0.0,
	      0.0,
	      0.0,
		  0.0,
		  0.0,
		  0.0,
		  (unsigned long long) 1000000*curTime.sec + curTime.nsec/1000);	  
	}
      }
      else
      {
	fprintf(resultsFile,"%d %f %f %f %f %f %f %llu ",i+1,
	    0.0,
	    0.0,
	    0.0,
		0.0,
		0.0,
		0.0,
		(unsigned long long) 1000000*curTime.sec + curTime.nsec/1000);	  
      }      
    }
    fprintf(resultsFile,"\n");
    
    // The target's estimated Pose
    //     if(NUM_TARGETS>0)
    //     {
    //       msg.robotEstimatedpose[4].pose.position.x = targetsToTrack[0]->curPose[0];
    //       msg.robotEstimatedpose[4].pose.position.y = targetsToTrack[0]->curPose[1];
    //       msg.robotEstimatedpose[4].pose.position.z = ROB_HT; //fixed height aboveground    
    //     }
    //    
    
    
    State_publisher.publish(msg);    
}
 

////////////////////////// METHOD DEFINITIONS OF THE TEAMMATEROBOT CLASS //////////////////////

void TeammateRobot::teammateOdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry, int RobotNumber, g2o::SparseOptimizer* graph_ptr)
{
  //The robot has started
  
  if(notInitialized)
    return;
  
  ifRobotIsStarted[RobotNumber-1]=true;
  uint seq = odometry->header.seq;
  prevTime = curTime;
  curTime = odometry->header.stamp;
  
  //ROS_INFO(" got odometry from teammate-robot (ID=%d) at time %d\n",RobotNumber, odometry->header.stamp.sec);  

  //Below is an example how to extract the twist variables from the odometry message and use it to propagate the robot state by using a simple motion model without noise.
  
  double linearVel = odometry->twist.twist.linear.x;
  double angularVel = odometry->twist.twist.angular.x;
  double deltaX=0,deltaY=0,deltaTheta=0;
  
  Duration delta = curTime - prevTime;
  double deltaSecs = delta.toSec();
  //cout<<"deltaSecs = "<<deltaSecs<<endl;
  
  deltaX = linearVel*deltaSecs*cos(angularVel*deltaSecs);
  deltaY = linearVel*deltaSecs*sin(angularVel*deltaSecs);
  deltaTheta = angularVel*deltaSecs;   

  if((*totalVertextCounter)<MAX_VERTEX_COUNT)
  {
    double odomVector[3] = {deltaX, deltaY, deltaTheta};
    
    Eigen::Isometry2d odom; 
    odom = Eigen::Rotation2Dd(odomVector[2]).toRotationMatrix();
    odom.translation() = Eigen::Vector2d(odomVector[0], odomVector[1]);    
    
    if(vertextCounter_ == 0)
    {
      curPose = initPose;
    }
    else
    {
      //VertexSE2* previousNode =  dynamic_cast<VertexSE2*>(graph_ptr->vertices()[SE2vertexID-1]);
      prevPose = curPose;
      //curPose = (previousNode->estimate().toIsometry())*odom;
      curPose = prevPose*odom;
    }

 
    // vertext id for the robots start at MAX_INDIVIDUAL_STATES times the robot number
    if(SE2vertexID == 0)
      SE2vertexID = (RobotNumber)*MAX_INDIVIDUAL_STATES;
    else
      SE2vertexID++;
    VertexSE2 * v = new VertexSE2();
    v->setId(SE2vertexID);  
    v->setEstimate(curPose);
    v->timestamp =  (unsigned long long) 1000000*curTime.sec + curTime.nsec/1000;
    v->agentID = RobotNumber;
    v->odometryReading = odom;
    v->isOptimizedAtLeastOnce = false;
    //if(vertextCounter_ == 0)
     //v->setFixed(true);
    graph_ptr->addVertex(v);
    currentPoseVertexIDs[RobotNumber-1] = SE2vertexID;
    //cout<<"currentPoseVertexIDs"<<"["<<RobotNumber<<"] ="<<currentPoseVertexIDs[RobotNumber-1]<<endl;
    
      
    if(vertextCounter_>0) //start adding edges if seq is greater than 0
    {
      SE2vertexID_prev = SE2vertexID - 1;
      v->setFixed(false);
      
      //Now add edge from the current node to the previous node
      EdgeSE2 * e = new EdgeSE2;
      // retrieve vertex pointers from graph with id's
      g2o::OptimizableGraph::Vertex * pose_a_vertex
	      = dynamic_cast<g2o::OptimizableGraph::Vertex*>
		(graph_ptr->vertices()[SE2vertexID_prev]);
      g2o::OptimizableGraph::Vertex * pose_b_vertex
	      = dynamic_cast<g2o::OptimizableGraph::Vertex*>
		(graph_ptr->vertices()[SE2vertexID]);			
		
	  //std::cout <<"Teammate SE2 Edge from to \n";		
	  //std::cout <<SE2vertexID_prev<<"\n"; 		
	  //std::cout <<SE2vertexID<<"\n"; 
	  		
		
      // error check vertices and associate them with the edge
      assert(pose_a_vertex!=NULL);
      assert(pose_a_vertex->dimension() == 3);
      e->vertices()[0] = pose_a_vertex;

      assert(pose_b_vertex!=NULL);
      assert(pose_b_vertex->dimension() == 3);
      e->vertices()[1] = pose_b_vertex;	  
            
      // add information matrix
      Eigen::Matrix<double, 3, 3> Lambda;
      //Lambda<<500, 0.0, 0.0, 0.0,500, 0.0,0.0,0.0, 50;    
      makeOdomSigmas(odomVector[0], odomVector[1],odomVector[2]);
      //cout<<"invSigmas = "<<invSigmaX<<" "<<invSigmaY<<" "<<invSigmaTheta<<endl;
      Lambda<<invSigmaX, 0.0, 0.0, 0.0,invSigmaY, 0.0,0.0,0.0, invSigmaTheta;
      //Lambda<<500000, 0.0, 0.0, 0.0,500000, 0.0,0.0,0.0, 500000;
      //Lambda<<50000.0, 0.0, 0.0, 0.0,50000.0, 0.0,0.0,0.0, 500000.0;
      //Lambda.setIdentity();
      
      // set the observation and imformation matrix
      const SE2 a(odom);
      e->setMeasurement(a);
      e->information() = Lambda;      
      
      // finally add the edge to the graph
      if(!graph_ptr->addEdge(e))
      {
	assert(false);
      } 
      
    }
    else //////First node is not set as a fixed node.. A prior node with 0 mean and high covariance is created and attached to the firt node with a very loose edge
    {
      v->setFixed(false); 
      
      Eigen::Isometry2d priorPose;
      VertexSE2* firstRobotnode = dynamic_cast<VertexSE2*>(graph_ptr->vertices()[SE2vertexID]);
      priorPose = Eigen::Rotation2Dd(0);
      priorPose.translation() = Eigen::Vector2d(0,0); 
      
      priorPose = initPose;
      
      //Create the Prior SE2 Node now. This node will keep track of the past information.
      VertexSE2 * v_prior = new VertexSE2();
      v_prior->setId((RobotNumber)*MAX_INDIVIDUAL_STATES - 1); //The ID is one before the first node pose of this robot  
      v_prior->setEstimate(priorPose);
      v_prior->timestamp =  firstRobotnode->timestamp;
      v_prior->agentID = RobotNumber;
      v_prior->setFixed(true);
      graph_ptr->addVertex(v_prior);
      
      //Now connect the fixed prior node to the first node through a very "loose" edge! An edge that means we have poorinformation regarding the prior
      EdgeSE2 * edgeFirst_to_Prior = new EdgeSE2;	
      edgeFirst_to_Prior->vertices()[0] = v_prior;
      edgeFirst_to_Prior->vertices()[1] = firstRobotnode;	  

      // add information matrix
      Eigen::Matrix<double, 3, 3> Lambda;      
      Lambda<<0.001, 0.0, 0.0, 0.0,0.001, 0.0,0.0,0.0, 0.0001;      
      
      // set the observation and imformation matrix
      Eigen::Isometry2d tempMean; tempMean = Eigen::Rotation2Dd(0); tempMean.translation() = Eigen::Vector2d(0,0);   
      edgeFirst_to_Prior->setMeasurement(tempMean);
      edgeFirst_to_Prior->information() = Lambda;      
      // finally add the edge to the graph
      if(!graph_ptr->addEdge(edgeFirst_to_Prior))
      {
	assert(false);
      } 
    }
    
   vertextCounter_++; 
   (*totalVertextCounter)++;
   //cout<<" vertextCounter_ for the robot OMNI"<<RobotNumber<<" is = "<<vertextCounter_<<endl;
   
   if(vertextCounter_ > WINDOW_SIZE)
   {
      Eigen::Isometry2d priorPose;
      
      // Prior (Mean) will be set to the vertex pose that is going to be deleted in the sliding window case 
      VertexSE2* vertex_to_become_prior = dynamic_cast<VertexSE2*>(graph_ptr->vertices()[SE2vertexID-WINDOW_SIZE]);
      priorPose = vertex_to_become_prior->estimate().toIsometry();
      
      //We now have to remove the last added prior node and a new prior node that contains the updated past. One may simply update the previous prior node but seems like it is not so straightforward to do so using g2o. Therefore I am copying the prious prior, deleting it, updating locally and then adding a new prior node which is the updated prior node
      VertexSE2* v_prior_old = dynamic_cast<VertexSE2*>(graph_ptr->vertices()[(RobotNumber)*MAX_INDIVIDUAL_STATES - 1]);
      graph_ptr->removeVertex(v_prior_old,/*bool detach=*/false);
      
      VertexSE2 * v_prior = new VertexSE2();
      v_prior->setId((RobotNumber)*MAX_INDIVIDUAL_STATES - 1); //The ID is one before the first node pose of this robot  
      v_prior->setEstimate(priorPose);
      v_prior->timestamp =  vertex_to_become_prior->timestamp;
      v_prior->agentID = RobotNumber;
      v_prior->setFixed(true);
      graph_ptr->addVertex(v_prior);
      
  
      //Now remove the oldest node (vertex_to_become_prior), which has already been used to update the new prior node to maintain the sliding window. However, grab the last hessian block of this vertex. This hessian will become the information contained by the prior node
      
      //cout<<"vertex_to_become_prior->hessian is = "<<endl<<vertex_to_become_prior->hessian(0,0)<<" "<<vertex_to_become_prior->hessian(0,1)<<"  "<<vertex_to_become_prior->hessian(0,2)<<endl;
      //cout<<vertex_to_become_prior->hessian(1,0)<<"  "<<vertex_to_become_prior->hessian(1,1)<<"  "<<vertex_to_become_prior->hessian(1,2)<<endl;
      //cout<<vertex_to_become_prior->hessian(2,0)<<"  "<<vertex_to_become_prior->hessian(2,1)<<"  "<<vertex_to_become_prior->hessian(2,2)<<endl;	
      Eigen::Matrix<double, 3, 3> Hessian;
      if(vertex_to_become_prior->isOptimizedAtLeastOnce)
      {
	//cout<<"Hessian exists, using it"<<endl;
	Hessian<<vertex_to_become_prior->hessian(0,0), vertex_to_become_prior->hessian(0,1), vertex_to_become_prior->hessian(0,2), vertex_to_become_prior->hessian(1,0),vertex_to_become_prior->hessian(1,1), vertex_to_become_prior->hessian(1,2),vertex_to_become_prior->hessian(2,0),vertex_to_become_prior->hessian(2,1), vertex_to_become_prior->hessian(2,2);
	//New hessian is an exponentially decayed version of the old
	//decayCoeff = 0.1;
	
	  double actualDecayCoeff = (-1.0)*decayCoeff*DECAY_LAMBDA;
	  //cout<<" actualDecayCoeff = "<<actualDecayCoeff<<endl;
	  
	  Hessian = Hessian*(exp(actualDecayCoeff)); //decay is exponentially with time.
      }
      else
      {
	Hessian<<5000, 0.0, 0.0, 0.0,5000, 0.0,0.0,0.0, 5000;
	//Hessian<<1, 0.0, 0.0, 0.0,1, 0.0,0.0,0.0, 1.0; 
	//cout<<"Vertex not optimized, hessian missing so assuming very low information"<<endl;
      }
	
      
      

      
      graph_ptr->removeVertex(vertex_to_become_prior,/*bool detach=*/false);
      
      //Now connect the fixed prior node to the first node of the rest of the seequence through a very "loose" edge! An edge that means we have poorinformation regarding the prior
	      
      EdgeSE2 * edgeFirst_to_Prior = new EdgeSE2;
      VertexSE2* firstVertex = dynamic_cast<VertexSE2*>(graph_ptr->vertices()[SE2vertexID-WINDOW_SIZE+1]); //remember it is the next node after the removed one
      
      edgeFirst_to_Prior->vertices()[0] = v_prior;
      edgeFirst_to_Prior->vertices()[1] = firstVertex;	  

      // set the observation and imformation matrix
      Eigen::Isometry2d odomFromFirstVertex = firstVertex->odometryReading; //measurement to connect the first node to prior node   
      //cout<<"Teammate odomFromFirstVertex = "<<odomFromFirstVertex.translation().x()<<" , "<<odomFromFirstVertex.translation().y()<<" agent ID = "<<firstVertex->agentID<<endl;
      edgeFirst_to_Prior->setMeasurement(odomFromFirstVertex);
      edgeFirst_to_Prior->information() = Hessian;      
      // finally add the edge to the graph
      if(!graph_ptr->addEdge(edgeFirst_to_Prior))
      {
	assert(false);
      } 
      
      //Calculating Arrival Cost at T-N
      double x_t = v_prior->estimate().translation().x();
      double y_t = v_prior->estimate().translation().y();
      double theta_t = v_prior->estimate().rotation().angle();
      
      double x_t_1 = firstVertex->estimate().translation().x();
      double y_t_1 = firstVertex->estimate().translation().y();
      double theta_t_1 = firstVertex->estimate().rotation().angle();
      
      Eigen::Vector3d delta = Eigen::Vector3d(odomVector[0], odomVector[1], odomVector[2]);
      
      Eigen::Vector3d h_t_t_1 = Eigen::Vector3d((x_t_1-x_t)*cos(theta_t) + (y_t_1-y_t)*sin(theta_t),
      -(x_t_1-x_t)*sin(theta_t) + (y_t_1-y_t)*cos(theta_t),
	normalizeAngle(theta_t_1-theta_t)
      );
      
      Eigen::Vector3d error = Eigen::Vector3d((delta(0)-h_t_t_1(0))*cos(h_t_t_1(2)) + (delta(1)-h_t_t_1(1))*sin(h_t_t_1(2)),
      -(delta(0)-h_t_t_1(0))*sin(h_t_t_1(2)) + (delta(1)-h_t_t_1(1))*cos(h_t_t_1(2)),
	normalizeAngle((delta(2)-h_t_t_1(2)))
      );
      
      //cout<<"Hessian is = "<<endl<<
      //      Hessian(0,0)<<"  "<<Hessian(0,1)<<"  "<<Hessian(0,2)<<endl;
      //cout<<Hessian(1,0)<<"  "<<Hessian(1,1)<<"  "<<Hessian(1,2)<<endl;
      //cout<<Hessian(2,0)<<"  "<<Hessian(2,1)<<"  "<<Hessian(2,2)<<endl;	
      //cout<<"error.transpose()*Hessian*error = "<<error.transpose()*Hessian*error<<endl;
      //cout<<"Hessian.determinant() = "<<Hessian.determinant()<<endl;
      //double beta_t_n = U[vertextCounter_-1-WINDOW_SIZE]/();
      
      //Now do not solve the resulting graph here... it is solved only in the self-robot class
      //solveSlidingWindowGraph(graph_ptr);
   }
     
  }

  
}


void TeammateRobot::teammateMeasurementDataCallback(const utiasdata_to_rosbags::MCLAMMeasurementData::ConstPtr& data, int RobotNumber, g2o::SparseOptimizer* graph_ptr)
{
  
  // self robot has not started yet
  
  if(notInitialized || SE2vertexID<=0)
    return;
      
  //ROS_INFO("Got measurement data from teammate robot %d",RobotNumber);  
  Time curObservationTime = data->header.stamp;

  bool subjectIsLandmark = false;
  bool subjectIsTeammate = false;
  int subject = 99;// we have to figure this out from the barcode so we initialize it with a non-exixting subject, e.g., 99
  int subjectBarcode = data->subjectBarcode;
  double range = data->range - 0.08;
  double bearing = data->bearing;
  
  if(range<=0)
    range = data->range;
  
  for(int i=0;i<20;i++)
  {
    //printf("checking with barcode %d \t",(*subject_barcode)[i]);
    if((*subject_barcode)[i]==subjectBarcode && i>4)
    {
      subject=i;
      subjectIsLandmark = true;
    }
    
    if((*subject_barcode)[i]==subjectBarcode &&  i<=4)
    {
      subject=i;
      subjectIsTeammate = true;
    }    
  }
  
  //cout<<"subjectBarcode found, barcode = "<<subjectBarcode<<endl;
  //cout<<"subject  id = "<<subject<<endl;
  
  if(subjectIsTeammate && currentPoseVertexIDs[subject]>0 && COP_LOC) //check if bar code belongs to a teammate robot which is activated
  {
   addRobotRobotEdge(subjectBarcode, range, bearing, SE2vertexID, currentPoseVertexIDs[subject] ,graph_ptr);
  }    
  
  
  //If the subject is not a landmark but the target robot
  if(subjectBarcode==23 && NUM_TARGETS>0) //check the header file for fixed subjectBarcodes for the target
  {
   
    //ROS_WARN("                                             Target Robot 5 Detected ");
    double localX = range*cos(bearing);
    double localY = range*sin(bearing);
    
    targetsToTrack[0]->addNewTGTVertexAndEdgesAround(0 /*targetID*/ ,localX,localY,RobotNumber, SE2vertexID, curObservationTime, graph_ptr);    
    
  }  
  
  if(subjectBarcode==32 && NUM_TARGETS>1) //check the header file for fixed subjectBarcodes for the target
  {
   
    //ROS_WARN("                                             Target Robot 4 Detected ");
    double localX = range*cos(bearing);
    double localY = range*sin(bearing);
    
    targetsToTrack[1]->addNewTGTVertexAndEdgesAround(1 /*targetID*/ ,localX,localY,RobotNumber, SE2vertexID, curObservationTime, graph_ptr);    
    
  }    
  
  if(subjectBarcode==41 && NUM_TARGETS>2) //check the header file for fixed subjectBarcodes for the target
  {
   
    //ROS_WARN("                                             Target Robot 3 Detected ");
    double localX = range*cos(bearing);
    double localY = range*sin(bearing);
    
    targetsToTrack[2]->addNewTGTVertexAndEdgesAround(2 /*targetID*/,localX,localY,RobotNumber, SE2vertexID, curObservationTime, graph_ptr);    
    
  }      
  
  //If the subject is a landmark and not another robot
  if(subjectIsLandmark && USE_LANDMARKS)
  {
    double localX = range*cos(bearing);
    double localY = range*sin(bearing);
    
    Eigen::Vector2d tempLandmarkObsVec = Eigen::Vector2d(localX,localY);
    Eigen::Matrix<double, 2, 2> tempInformationOnLandmark;
    
    double covDD = K1;//K1*(range*range);
    double covPhiPhi = K2;//K2*range;
    double covXX = pow(cos(bearing),2) * covDD + pow(sin(bearing),2) * ( pow(range,2) * covPhiPhi + covDD * covPhiPhi );
    double covYY = pow(sin(bearing),2) * covDD + pow(cos(bearing),2) * ( pow(range,2) * covPhiPhi + covDD * covPhiPhi );    
    
    if(covXX<0||covYY<0)
	ROS_WARN(" covariance negative!!! ");
      
    tempInformationOnLandmark<<1/covXX,0,
				 0,1/covYY;    
    
    //cout<<"covddphiphi"<<endl;
    //cout<<covDD<<"\t"<<0<<endl<<0<<"\t"<<covPhiPhi<<endl<<endl; 				 
    //cout<<"covxy"<<endl;
    //cout<<covXX<<"\t"<<0<<endl<<0<<"\t"<<covYY<<endl<<endl; 				 
    
    EdgeSE2PointXY * e = new  EdgeSE2PointXY();
    // retrieve vertex pointers from graph with id's
    g2o::OptimizableGraph::Vertex * pose_a_vertex
	    = dynamic_cast<g2o::OptimizableGraph::Vertex*>
	      (graph_ptr->vertices()[SE2vertexID]);//current robot pose vertex
    g2o::OptimizableGraph::Vertex * pose_b_vertex
	    = dynamic_cast<g2o::OptimizableGraph::Vertex*>
	      (graph_ptr->vertices()[subject]);//landmark Vertex is the subject number identified using the subjectBarcode earlier in this function	
    
    if(pose_a_vertex!=NULL && pose_a_vertex->dimension() == 3)		
      e->vertices()[0] = pose_a_vertex;
    else
    {
      ROS_WARN(" Robot pose does not exist at teammate %d ",RobotNumber);
      return;
    }   	
      
    if(pose_b_vertex!=NULL && pose_b_vertex->dimension() == 2)
      e->vertices()[1] = pose_b_vertex;
    else
    {
      ROS_WARN(" Robot pose does not exist at teammate %d ",RobotNumber);
      return;
    }   	
	      
    
    e->setMeasurement(tempLandmarkObsVec);
    e->information() = tempInformationOnLandmark;   
    if(!graph_ptr->addEdge(e))
    {
 	ROS_WARN(" Edge not added because it may already exist ");
    }       
    
    
  }  
}


void TeammateRobot::teammateGTCallback(const geometry_msgs::PoseStamped::ConstPtr& data, int RobotNumber, g2o::SparseOptimizer* graph_ptr)
{
  //ROS_INFO(" got landmark data from teammate robot (ID=%d)",RobotNumber);  
  
  if(*totalVertextCounter <= 0)
    return;  
  
  uint seq = data->header.seq;
  
  if(notInitialized)
  {
    initPose = Eigen::Rotation2Dd(tf::getYaw(data->pose.orientation)).toRotationMatrix();
    initPose.translation() = Eigen::Vector2d(data->pose.position.x,data->pose.position.y); 

    notInitialized = false;
  }
  
    curGTPose = Eigen::Rotation2Dd(tf::getYaw(data->pose.orientation)).toRotationMatrix();
    curGTPose.translation() = Eigen::Vector2d(data->pose.position.x,data->pose.position.y);  
    
    (*teamGTPose)[RobotNumber-1] =  curGTPose;

}


int main (int argc, char* argv[])
{
 ros::init(argc, argv, "mhls_utias_dataset");
  
 if (argc < 5)
  {
    ROS_WARN("WARNING: you should at least specify output file for storing results\n");
    return 1;
  }
  else
  {
    strcpy(resultsFileName,"/home/aamir/catkin_ws/src/mhls_utias_dataset/results/");
    strcat(resultsFileName,argv[1]);
    
    MY_ID = atoi(argv[2]);
    WINDOW_SIZE = atoi(argv[3]);
    DECAY_LAMBDA = atoi(argv[4]);
  }
  
  //registering all the types from the libraries
  g2o::DlWrapper dlTypesWrapper;
  g2o::loadStandardTypes(dlTypesWrapper, argc, argv);

  //register all the solvers
  g2o::DlWrapper dlSolverWrapper;
  g2o::loadStandardSolver(dlSolverWrapper, argc, argv);    
  
  g2o::SparseOptimizer graph;  
 
  NodeHandle nh;
  
  //nh.getParam("MY_ID", MY_ID); 
  //nh.getParam("WINDOW_SIZE", WINDOW_SIZE); 
  //nh.getParam("DECAY_LAMBDA", DECAY_LAMBDA); 
  
  nh.getParam("NUM_ROBOTS", NUM_ROBOTS);
  nh.getParam("MAX_ROBOTS", MAX_ROBOTS);
  nh.getParam("NUM_TARGETS", NUM_TARGETS);
  nh.getParam("WINDOW_SIZE_TGT", WINDOW_SIZE_TGT);
  nh.getParam("MAX_VERTEX_COUNT", MAX_VERTEX_COUNT);
  nh.getParam("MAX_INDIVIDUAL_STATES", MAX_INDIVIDUAL_STATES);
  nh.getParam("MAX_ITERATIONS", MAX_ITERATIONS); 
  nh.getParam("COP_LOC", COP_LOC);
  nh.getParam("USE_LANDMARKS", USE_LANDMARKS);
  nh.getParam("K1", K1);
  nh.getParam("K2", K2);
  nh.getParam("K3", K3);
  nh.getParam("K4", K4);      
  nh.getParam("K5", K5); 
  nh.getParam("decayCoeff", decayCoeff);
  nh.getParam("SLAM", SLAM);
  
  printf("--------MY_ID = %d and WINDOW_SIZE set to %d and DECAY_LAMBDA set to %d--------\n",MY_ID, WINDOW_SIZE,DECAY_LAMBDA);
  
  ReadRobotMessages node(&graph, &nh);

  
  //MultiThreadedSpinner spinner(0);
  //spinner.spin();
  spin();
  return 0;
}





























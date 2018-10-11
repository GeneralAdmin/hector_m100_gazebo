/** @file first_fly_hector.cpp
 *  @version 1.0
 *  @date Oct, 2017
 *
 *  @brief
 *  Voo simples para o sistema de controle autonamo de voo.
 *  Realiza comandos de velociade para subir dois metros, 
 *  andar dois metros para frente, voltar dois metros para trás e pousar.
 *
 *  @copyright 2017 CORO-UFMG. All rights reserved.
 *
 */

 #include "dji_m100/first_fly_hector.h"
 #include "geometry_msgs/Pose.h"
 #include "geometry_msgs/PoseStamped.h"
 #include "hector_uav_msgs/TakeoffAction.h"
 #include "hector_uav_msgs/LandingAction.h"
 #include "hector_uav_msgs/PoseAction.h"
 #include "actionlib/client/simple_action_client.h"
 #include <vector>
 
//const float deg2rad = C_PI/180.0;    
//const float rad2deg = 180.0/C_PI;

// Pose Goal message   - codigo para controle por waypoint
// ------------------------------------------------------------------------
geometry_msgs::PoseStamped current_goal;
std::vector<geometry_msgs::Pose> waypoints;

int wp_counter = 0;
double wp_radius = 0.005;
bool quit_loop = false;

typedef actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> PoseActionClient;
typedef actionlib::SimpleActionClient<hector_uav_msgs::TakeoffAction> TakeoffClient;


// Callback function for the subscribe() function
void position_cb( const geometry_msgs::PoseStamped::ConstPtr& current_pos ) {
	
	ROS_INFO("Current position: [%f,%f,%f]", current_pos->pose.position.x, current_pos->pose.position.y, current_pos->pose.position.z);
	ROS_INFO("Current goal: [%f,%f,%f]", current_goal.pose.position.x, current_goal.pose.position.y, current_goal.pose.position.z);
	ROS_INFO("abs( current_pos->pose.position.y - current_goal.pose.position.y ): %f", fabs( current_pos->pose.position.y -current_goal.pose.position.y ));
	
	//If the current position is close to the current goal for X, Y, & Z
	if( fabs( current_pos->pose.position.x - current_goal.pose.position.x ) < wp_radius ) 
	{
		if( fabs( current_pos->pose.position.y - current_goal.pose.position.y ) < wp_radius ) {
			if( fabs( current_pos->pose.position.z - current_goal.pose.position.z ) < wp_radius ) {
				//If there are more waypoints
				wp_counter++;	//Move to the next waypoint
				if( wp_counter < waypoints.size() ) {
					current_goal.pose = waypoints.at(wp_counter);
				} else {
					quit_loop = true;
					ROS_INFO( "Finished the waypoint path!" );
				}
			}
		}
	}
	
}



void generate_waypoints() {
	// Local variables
	geometry_msgs::Pose tmp_wp;

	// Processing
	//Waypoint 1
	tmp_wp.orientation.w = 1.0;	//Intitalize the quaternion (relying on x, y, and z to default to 0
	tmp_wp.position.z = 20.0;	//First waypoint is at [0, 0, 12]
	waypoints.push_back(tmp_wp);


	tmp_wp.orientation.w = 0.7071068;
	tmp_wp.orientation.z = -0.7071068;
	waypoints.push_back(tmp_wp);

	
	//Waypoint 2
	tmp_wp.position.x = -30.0;	//[5.0, 0, 12.0]
	waypoints.push_back(tmp_wp);

	
	//Waypoint 3
	tmp_wp.position.y = -22.0;	//[5.0, 5.0, 12.0]
	waypoints.push_back(tmp_wp);

	tmp_wp.orientation.w = 1;
	tmp_wp.orientation.z = 0;
	waypoints.push_back(tmp_wp);


	//Waypoint 4
	tmp_wp.position.x = 0.0;	//[0.0, 5.0, 12.0]
	waypoints.push_back(tmp_wp);

	tmp_wp.orientation.w = 0.7071068;
	tmp_wp.orientation.z = -0.7071068;
	waypoints.push_back(tmp_wp);
	
	//Waypoint 5
	//tmp_wp.position.x = -1.0;	//
	tmp_wp.position.y = 0.0; //[0, 0, 12.0]
	waypoints.push_back(tmp_wp);

	tmp_wp.position.x = -25.0;	//[5.0, 0, 12.0]
	waypoints.push_back(tmp_wp);
	
	tmp_wp.position.y = -20.0;	//[5.0, 5.0, 12.0]
	waypoints.push_back(tmp_wp);

	tmp_wp.position.y = -21.0;	//[5.0, 5.0, 12.0]
	waypoints.push_back(tmp_wp);
	

	tmp_wp.position.x = 0.0;	//[0.0, 5.0, 12.0]
	waypoints.push_back(tmp_wp);
	
	tmp_wp.position.y = 0.0; //[0, 0, 12.0]
	waypoints.push_back(tmp_wp);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "first_fly_hector_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate( 10 );
  
  // Carrega parametros
  double vel, t_wait, t_desloc;
  nh.param("/first_fly_node/vel", vel, 0.25);
  nh.param("/first_fly_node/t_wait", t_wait, 1.0);
  nh.param("/first_fly_node/t_desloc", t_desloc, 8.0);
  ROS_INFO("params: t_wait = %f, t_desloc = %f, vel = %f", t_wait, t_desloc, vel);
  
  
  //Local waypoint variables -------------------------------------------
  geometry_msgs::PoseStamped HBII0;		// Initial pose in inertial coords. Set to zero. 	
  ros::Subscriber pos_sub;
  hector_uav_msgs::PoseGoal poseGoal;		// PoseGoal object for simpleactionclient PoseActionClient
  hector_uav_msgs::TakeoffGoal goal;		// Goal (empty message) for TakeoffClient
  
  // Publish to /position_goal, and Listen to /ground_truth_to_tf/pose (current pose)
  pos_sub = nh.subscribe( "/ground_truth_to_tf/pose", 1000, position_cb );
  
  // Generate the waypoints
  generate_waypoints();
  
  //Format output message
  current_goal.header.frame_id = "world";
  current_goal.pose = waypoints.at(wp_counter); //initialize with wp 0
  
  //Write something so we know the node is running
  ROS_INFO( "Publishing position goal..." );
	
  // Initialise the PoseActionClient
  PoseActionClient poc(nh, "action/pose");
  poc.waitForServer();
  ROS_INFO("Pose client initialised.");
  
  //Local waypoint variables -------------------------------------------
  
  

  coro::M100_hector m100(nh);

  ROS_INFO("Selec (F) mode on RC to start mission.");
  ros::Rate loop_start(1);
  while (ros::ok())
  {
    if( m100.isAutonomous() )
    {
        ROS_INFO("F mode: ON.");
		
		
		 m100.initZeroPos();  
  
  		//bool obtain_control_result = obtain_control();
  		bool takeoff_result;
  		ROS_INFO("M100 taking off!");
  		//takeoff_result = m100.takeoff();
		TakeoffClient toc(nh, "action/takeoff");
		toc.waitForServer();
		ROS_INFO("Takeoff client initialised.");
	
		// Send take-off goal to toc
		toc.sendGoal(goal);
		
		ros::Time start_time = ros::Time::now();
		
		// Main while loop
		while ( ros::ok() && !quit_loop ) {
			//Update our message so the receiving node knows it is recent
			current_goal.header.stamp = ros::Time::now();
			current_goal.header.seq++;
			current_goal.header.frame_id = "world";

			// Send current goal to pose
			poseGoal.target_pose = current_goal; 
			poc.sendGoal(poseGoal);

			//Update subscribers and sleep
			ros::spinOnce();
			loop_rate.sleep();
			loop_rate.sleep();
			
			ros::Duration elapsed_time = ros::Time::now() - start_time;
			
			//Rotate UAV
			geometry_msgs::Twist tvel;
			tvel.linear.z = vel*C_PI;
			m100.setVel( tvel );
			
			if( elapsed_time > ros::Duration(t_desloc) )
        	{
				start_time = ros::Time::now();
			}
			
			
			ros::spinOnce();
			loop_rate.sleep();
		}
		
		
		
		
          m100.land(); // LAND          

          ROS_INFO("step %d: === Mission Finished ===", 0);
		
		
		
      //  first_fly( m100, vel, t_wait, t_desloc );
        ROS_INFO("Selec (P) mode on RC to END mission.");
        m100.waitNotAutonomousMode();
        ROS_INFO("Selec (F) mode on RC to START mission.");
    }
    ros::spinOnce();
    loop_start.sleep();
  }

  return 0;
}

// Controle autonamo do voo com a velocidade 'vel'. 
// Decola, sobe dois metros, anda dois metros para frente, anda dois metros para trás e pousa.
// Parametros de entrada:
//  vel      - velocidade [m/s]
//  t_wait   - tempo de espera [s]
//  t_desloc - tempo de deslocamento [s]
int first_fly( coro::M100_hector &m100, double vel, double t_wait, double t_desloc )
{
  
  sensor_msgs::NavSatFix m100_home_pos;
  
/* bool takeoff_result;

  if(!takeoff_result)
  {
    ROS_INFO("Take off fail!");
    return -1;
  }*/

  int step = 0;
  ROS_INFO("step %d: ROTATE.", step);
  ros::Time start_time = ros::Time::now();

  ros::spinOnce();
  
    //Verify 
   if (m100.isGpsOk())
   {
		ROS_INFO("GPS is OK");	   
		
		m100.printGPSPos();

		m100_home_pos = m100.getCurrGPSPos();
		
		ros::spinOnce();
   }
  
  
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    ros::Duration elapsed_time = ros::Time::now() - start_time;
    geometry_msgs::Twist tvel;
    switch(step)
    {
      case 0: // ROTATE
        tvel.angular.z = vel*C_PI;
        m100.setVel( tvel );	
        if( elapsed_time > ros::Duration(t_wait) )
        {
          step++;
          ROS_INFO("step %d: move FRONT. t = %f, vel = %f", step, t_desloc, vel);
          start_time = ros::Time::now();
        }      
        break;

      case 1: // MOVE front
        tvel.linear.x = vel;
        m100.setVel( tvel, false );
        if( elapsed_time > ros::Duration(t_desloc) )
        {
          step++;
          ROS_INFO("step %d: ROTATE. t = %f", step, t_wait);
          start_time = ros::Time::now();
        }
        break;

      case 2: // ROTATE
        tvel.linear.z = vel*C_PI;
        m100.setVel( tvel );
        if( elapsed_time > ros::Duration(t_wait) )
        {
          step++;
          ROS_INFO("step %d: move front. t = %f", step, t_desloc);
          start_time = ros::Time::now();
        }
        break;

      case 3: // MOVE FRONT
        tvel.linear.x = vel;
		tvel.linear.z = vel*C_PI/2;
        m100.setVel( tvel );
        if( elapsed_time > ros::Duration(t_desloc) )
        {
          step++;
          ROS_INFO("step %d: wait.", step);
          start_time = ros::Time::now();
        }
        break;

      case 4: // WAIT
        m100.stop();
        if( elapsed_time > ros::Duration(t_wait) )
        {
          step++;
          ROS_INFO("step %d: move rear.", step);
          start_time = ros::Time::now();
        }
        break;

      case 5: // MOVE REAR
        tvel.linear.x = -vel;
		tvel.linear.z = vel*C_PI/2;
        m100.setVel( tvel );
        if( elapsed_time > ros::Duration(t_desloc) )
        {
          step++;
          ROS_INFO("step %d: wait.", step);
          start_time = ros::Time::now();
        }
        break;

      case 6: // Land
        m100.stop();
        if( elapsed_time > ros::Duration(t_wait) )
        {
          step++;
          ROS_INFO("step %d: land.", step);
          start_time = ros::Time::now();

          m100.land(); // LAND          

          step++;
          ROS_INFO("step %d: === Mission Finished ===", step);
          step = -1;
          
        }
        break;
    }

    ros::spinOnce();
    loop_rate.sleep();
    ++count;

	//m100.printGPSPos();
	
    if(step < 0)
      break;
  }
  
  
  
  return 0;
}


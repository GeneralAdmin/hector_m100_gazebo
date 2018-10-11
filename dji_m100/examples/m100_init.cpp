/** @file first_fly.cpp
 *  @version 1.0
 *  @date Oct, 2017
 *
 *  @brief
 *  Inicializa o drone dji M100, espera o gps funcionar e exibe os dados de GPS.
 *
 *  @copyright 2017 CORO-UFMG. All rights reserved.
 *
 */

 #include "dji_m100/m100.h"
 #include <nav_msgs/Path.h>
 
//const float deg2rad = C_PI/180.0;    
//const float rad2deg = 180.0/C_PI;

coro::M100 *g_m100;
bool init_ok = false;
geometry_msgs::Vector3Stamped pose_path;

void vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  if( !init_ok )
    return;
  
  g_m100->setVel(*msg,false);
  
}


void pose_path_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  pose_path = *msg;
/*  if( !init_ok )
    return;

  msg->vector.x;
  
  
  g_m100->setVel(*msg,false);*/
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "m100_init");
  ros::NodeHandle nh;
  
  ROS_INFO("## m100_init START ##");
  nh.setParam("/m100/ukf_mode", true);
  nh.setParam("/m100/att_ctrl_gain_k", 0.2);
  
  coro::M100 m100(nh,false);
  g_m100 = &m100;
  
  
  ros::Rate loop_start(6);
  //ros::Publisher velPub = nh.advertise<geometry_msgs::Twist>("/dji_sdk/set_vel", 1);  
  ros::Subscriber velSub = nh.subscribe("dji_sdk/vel", 1, &vel_callback );
  ros::Subscriber poseSub = nh.subscribe("/pose_path", 1, &pose_path_callback );
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path> ( "/path_follow/path", 1 );
  
  ROS_INFO("Selec (F) mode on RC to start mission.");
  
  
  nav_msgs::Path drone_path;
  geometry_msgs::PoseStamped drone_pose;

  init_ok = false;
  while (ros::ok())
  {
    if( !init_ok )
    {
      // Espera inicializar posição do GPS
      
      if( m100.isAutonomous() )
      {      
	if( m100.isGpsOk() )
	{
	    init_ok = m100.initZeroPos();
	    
	    if(init_ok)
	      ROS_INFO("## M100 POSITION SUCCESSFUL INITIALIZED ##");
	}
      }
    }
    else  
    {
      // Exibe os dados de posição
      
      geometry_msgs::PointStamped local_position = m100.getCurrLocalPos();
      ROS_INFO("Local Position: \n x = %f \n y = %f \n z = %f", 
	      local_position.point.x, local_position.point.y, local_position.point.z );
      
      sensor_msgs::NavSatFix gps_position = m100.getCurrGPSPos();
      ROS_INFO("GPS Position: \n latitude = %f \n longitude = %f \n altitude = %f", 
	      gps_position.latitude, gps_position.longitude, gps_position.altitude );
      
      ROS_INFO("GPS Health = %d", m100.getGpsHealth());
      
      geometry_msgs::Twist vel = m100.getVel();
      ROS_INFO("Vel: \n x = %f \n y = %f \n z = %f", 
	      vel.linear.x, vel.linear.y, vel.linear.z );

      drone_path.poses.clear();
      drone_path.header.stamp=ros::Time::now();
      drone_path.header.frame_id="world";  
      for( int i = 0; i < 3; i++ )
      {
	drone_pose.pose.position.x = pose_path.vector.x + i;
	drone_pose.pose.position.y = pose_path.vector.y + i;
	drone_pose.pose.position.z = 3.0;
	drone_path.poses.push_back(drone_pose);
      }
      path_pub.publish(drone_path);
      
    }
    
    ros::spinOnce();
    loop_start.sleep();
  }

  ROS_INFO("## m100_init END ##");
  return 0;
}


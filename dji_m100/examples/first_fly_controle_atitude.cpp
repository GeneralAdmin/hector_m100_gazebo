/** @file first_fly.cpp
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

 #include "dji_m100/m100.h"
 #include "dji_m100/first_fly.h"
 
//const float deg2rad = C_PI/180.0;    
//const float rad2deg = 180.0/C_PI;





int main(int argc, char** argv)
{
  ros::init(argc, argv, "first_fly_node");
  ros::NodeHandle nh;
  
  // Carrega parametros
  double vel, t_wait, t_desloc;
  nh.param("/first_fly_node/vel", vel, 0.5);
  nh.param("/first_fly_node/t_wait", t_wait, 1.0);
  nh.param("/first_fly_node/t_desloc", t_desloc, 4.0);
  ROS_INFO("params: t_wait = %f, t_desloc = %f, vel = %f", t_wait, t_desloc, vel);

  //bool attCtrl = true;
  //nh.param("/first_fly_node/attidude_control", attCtrl, true);
  //ROS_INFO("attidude_control = %d", (int)attCtrl);
  
  //coro::M100 m100(nh,false,attCtrl);
  coro::M100 m100(nh,false);

  ROS_INFO("Selec (F) mode on RC to start mission.");
  ros::Rate loop_start(1);
  while (ros::ok())
  {
    if( m100.isAutonomous() )
    {
        ROS_INFO("F mode: ON.");
        first_fly( m100, vel, t_wait, t_desloc );
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
int first_fly( coro::M100 &m100, double vel, double t_wait, double t_desloc )
{
  //bool obtain_control_result = obtain_control();
  bool takeoff_result;
  ROS_INFO("M100 taking off!");
  
  takeoff_result = m100.takeoff();

  if(!takeoff_result)
  {
    ROS_INFO("Take off fail!");
    return -1;
  }

  int step = 0;
  ROS_INFO("step %d: wait.", step);
  ros::Time start_time = ros::Time::now();

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    ros::Duration elapsed_time = ros::Time::now() - start_time;
    geometry_msgs::Twist tvel;
    switch(step)
    {
      case 0: // WAIT
        if( elapsed_time > ros::Duration(0.1) )
        {
          step++;
          ROS_INFO("step %d: move up. t = %f, vel = %f", step, t_desloc, vel);
          start_time = ros::Time::now();
        }      
        break;

      case 1: // MOVE UP
        tvel.linear.z = vel;
        m100.setVel( tvel );
        if( elapsed_time > ros::Duration(t_desloc) )
        {
          step++;
          ROS_INFO("step %d: wait. t = %f", step, t_wait);
          start_time = ros::Time::now();
        }
        break;

      case 2: // WAIT
        m100.stop();
        if( elapsed_time > ros::Duration(t_wait) )
        {
          step++;
          ROS_INFO("step %d: move front. t = %f", step, t_desloc);
          start_time = ros::Time::now();
        }
        break;

      case 3: // MOVE FRONT
        tvel.linear.x = vel;
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
        m100.setVel( tvel );
        if( elapsed_time > ros::Duration(t_desloc) )
        {
          step++;
          ROS_INFO("step %d: wait.", step);
          start_time = ros::Time::now();
        }
        break;

      case 6: // WAIT
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

    if(step < 0)
      break;
  }
  
  return 0;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/
/*void
localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
}


geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}


bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_gps = *msg;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void rc_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  current_rc = *msg;
}*/


/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
/*bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(6))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  // ROS_INFO(" f_status = %d\n current_gps_z = %f\n home = %f", flight_status, current_gps.altitude, home_altitude);

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
      current_gps.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }

  return true;
}
*/


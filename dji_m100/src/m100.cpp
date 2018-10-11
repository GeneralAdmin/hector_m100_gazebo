#include "dji_m100/m100.h"

// ROS includes
#include <tf/tf.h>

// DJI SDK includes
#include "dji_sdk/dji_sdk.h"
#include <dji_sdk/dji_sdk_node.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/SetLocalPosRef.h>

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

using namespace coro;
//using namespace DJI::OSDK;

ros::ServiceClient waypoint_upload_service;
ros::ServiceClient waypoint_action_service;
ros::ServiceClient hotpoint_action_service;

M100::M100( ros::NodeHandle &nh, bool readOnly ):
m_nh( nh ), m_readOnly(readOnly), m_flight_status(255)
{
  m_nh.param("/m100/ukf_mode", m_ukf_mode, false);
  m_nh.param("/m100/debug", m_debug, false);
  m_nh.param("/m100/att_control_test", m_att_control_test, true);
  
  // Subscribe to messages from dji_sdk_node	
  m_attitudeSub       = m_nh.subscribe("dji_sdk/attitude", 1, &M100::attitude_callback, this);
  m_gpsSub            = m_nh.subscribe("dji_sdk/gps_position", 1, &M100::gps_callback, this);
  m_gpsHealthSub      = m_nh.subscribe("dji_sdk/gps_health", 1, &M100::gps_health_callback, this);
  m_flightStatusSub   = m_nh.subscribe("dji_sdk/flight_status", 1, &M100::flight_status_callback,this);
  m_rcSub             = m_nh.subscribe("dji_sdk/rc", 1, &M100::rc_callback, this);
  //Velocity in ENU ground frame, published at 50 Hz. The velocity is valid only when gps_health >= 3.
  m_velSub            = m_nh.subscribe("dji_sdk/velocity", 1, &M100::vel_callback, this);
  m_ukf_velSub        = m_nh.subscribe("ukf_vel", 1, &M100::ukf_vel_callback, this);
  m_ukf_odomSub        = m_nh.subscribe("ukf_odom", 1, &M100::ukf_odom_callback, this);
  // IMU data including raw gyro reading in FLU body frame,
  m_imuSub            = m_nh.subscribe("dji_sdk/imu", 1, &M100::imu_callback, this);
  m_localPositionSub  = nh.subscribe("dji_sdk/local_position", 1, &M100::local_position_callback, this);

  if(!m_readOnly)
  {
    // Publish the control signal
    m_ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 1);
    m_ctrlRpy 	   = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 1);
    
    // Basic services
    m_sdk_ctrl_authority_service = m_nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
    m_drone_task_service         = m_nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    m_set_local_pos_ref_service  = m_nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");
	
	//Serviços atualizados
	waypoint_upload_service = nh.serviceClient<dji_sdk::MissionWpUpload>(
    "dji_sdk/mission_waypoint_upload");
	waypoint_action_service = nh.serviceClient<dji_sdk::MissionWpAction>(
    "dji_sdk/mission_waypoint_action");
	 hotpoint_action_service = nh.serviceClient<dji_sdk::MissionHpAction>(
    "dji_sdk/mission_hotpoint_action");
		
	
  }
  
  /*
  // Activate
  if (activate().result)
  {
    ROS_INFO("Activated successfully");
  }
  else
  {
    ROS_WARN("Failed activation");
    return -1;
  }
  
  // Obtain Control Authority
  ServiceAck ack = obtainCtrlAuthority();
  if (ack.result)
  {
    ROS_INFO("Obtain SDK control Authority successfully");
  }
  else
  {
    if (ack.ack_data == 3 && ack.cmd_set == 1 && ack.cmd_id == 0)
    {
      ROS_INFO("Obtain SDK control Authority in progess, "
               "send the cmd again");
      obtainCtrlAuthority();
    }
    else
    {
      ROS_WARN("Failed Obtain SDK control Authority");
      return -1;

    }
  }

  
  */
  
  
  
  
  
  
  
    //this->initZeroPos();
  
}

void M100::attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  m_current_atti = msg->quaternion;
 // m_attitude_msgOk = true;
}

//retorna com a menssagem de gps 
void M100::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  m_current_gps = *msg;
  //m_gps_msgOk = true;
}

void M100::gps_health_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  m_gps_health = msg->data;
}

void M100::flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  m_flight_status = msg->data;
}

void M100::rc_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  m_current_rc = *msg;
}

void M100::vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  m_current_vel = *msg;
}

void M100::ukf_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  m_current_ukf_vel = *msg;
}

void M100::ukf_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  m_current_ukf_odom = *msg;
}

void M100::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  m_current_imu = *msg;
}

void M100::local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  m_local_position = *msg;
}

bool M100::takeoff()
{
  if(m_readOnly)
    return false;

  showStatus();
  if(m_flight_status == DJISDK::M100FlightStatus::M100_STATUS_IN_AIR)
  {
    return true; // Se já está no ar retorna 'true'
  }
 /* else if(m_flight_status != DJISDK::M100FlightStatus::M100_STATUS_ON_GROUND)
  {
    return false; // Caso contrário se não está no solo retorna 'false', pois não pode decolar.
  }*/

  ros::Time start_time = ros::Time::now();

  float home_altitude = m_current_gps.altitude;
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

  // ROS_INFO(" f_status = %d\n m_current_gps_z = %f\n home = %f", m_flight_status, m_current_gps.altitude, home_altitude);

  if(m_flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
      m_current_gps.altitude - home_altitude < 1.0)
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


/*!
 * Landing
 */
bool M100::land()
{
  if(m_readOnly)
    return false;
  
  if(!takeoff_land(6))
  {
    ROS_ERROR("Land fail!");
    return false;
  }

  return true;
}

// Retorna true se o F mode está ativado.
bool M100::isAutonomous()
{
  // Testa se o parametro do modo do controle está disponível.
  if(m_current_rc.axes.size() < 5)
  {
    
    //ROS_ERROR("(M100::isAutonomous): Sem mensagem do RC. HECTOR_SIMU está desativado?");
    return false;
  }

  //ROS_INFO("(M100::isAutonomous): Mode = %f",m_current_rc.axes[4]);
  if(m_current_rc.axes[4] != 8000) // Se o modo não for "F"
  {
    //ROS_INFO("(M100::isAutonomous): isAutonomous = false. HECTOR_SIMU está desativado?");
    return false;
  }
  //ROS_INFO("(M100::isAutonomous): isAutonomous = true.");

  return obtain_control();
}

bool M100::isGpsOk()
{
  bool isOk = (m_gps_health >= 3);
  if( !isOk )
  {
     ROS_ERROR("M100::isGpsOk(): GPS is not Ok - GPS Health = %d",m_gps_health);
  }

  return isOk;
}

void M100::setVel(geometry_msgs::Twist vel, bool isBodyReferece)
{
  if(m_readOnly)
    return;
  if( m_ukf_mode )
  {
    if( isBodyReferece == true )
      ROS_ERROR("M100::setVel(): isBodyReferece = 'true' but is expected to be 'false'. ");	
    else
    {
      double k = 1.0;
      m_nh.param ( "/m100/att_ctrl_gain_k", k, 1.0 );
      
      if(m_att_control_test)
	 // m_current_vel velocidades em relação ao mundo /dji_sdk/velocity graund_ENU
	this->ctrlVel( vel, m_current_vel, k, isBodyReferece);
      else
	this->ctrlVel( vel, m_current_ukf_vel, k, isBodyReferece);
    }
  
  }
  else
  {
    sensor_msgs::Joy controlVelYawRate;
    
    // teste de attitude
/*uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
		DJISDK::HORIZONTAL_ANGLE |
		DJISDK::YAW_RATE            |
		DJISDK::HORIZONTAL_GROUND   |
		DJISDK::STABLE_ENABLE);    
*/
    uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
		DJISDK::HORIZONTAL_VELOCITY |
		DJISDK::YAW_RATE            |
		DJISDK::HORIZONTAL_GROUND   |
		DJISDK::STABLE_ENABLE);

    if(isBodyReferece)
    {
      flag = (DJISDK::VERTICAL_VELOCITY   |
		    DJISDK::HORIZONTAL_VELOCITY |
		    DJISDK::YAW_RATE            |
		    DJISDK::HORIZONTAL_BODY   |
		    DJISDK::STABLE_ENABLE);
    }

    controlVelYawRate.axes.push_back(vel.linear.x);
    controlVelYawRate.axes.push_back(vel.linear.y);
    controlVelYawRate.axes.push_back(vel.linear.z);
    controlVelYawRate.axes.push_back(vel.angular.z);
    controlVelYawRate.axes.push_back(flag);

    m_ctrlBrakePub.publish(controlVelYawRate);
  }
}


// Controle Velocidade atuando na attitude para controlar o X e Y. Assim fica independente do GPS.
// vel_setPoint -> Velocidades desejadas.
// vel_mesure 	-> Entrada de velocidades medidas pelo sensor.
// k 		-> Ganho do controlador
// If 'isBodyReferece = true'' the velocity is relative to the drone frame.
// If not, the velocity is relative to the ENU ground frame (GPS).

void M100::ctrlVel(geometry_msgs::Twist vel_setPoint, geometry_msgs::Vector3Stamped vel_mesure,
		   double k, bool isBodyReferece)
{
  if(m_readOnly)
    return;
  
  
  double difX = 0;
  double difY = 0;

  // A velocidade medida pelo topico /dji_sdk/velocity está com o vector.x invertido com o vector.y
  //double vel_x = vel_mesure.vector.y;
  //double vel_y = vel_mesure.vector.x;

  // Apenas controla a velocidade se receber valores de medição válidos.
  if( vel_mesure.vector.x != 0.0 ||  m_att_control_test )
  {
    difX = vel_setPoint.linear.x - vel_mesure.vector.x;
    difY = vel_setPoint.linear.y - vel_mesure.vector.y;
  }
    
    
  
  double attN = k * difX; // Varia de -0,5235 a 0,5235 radianos
  double attE = k * difY; // k = 0.5
  
  // Debug
  if( m_debug )
  {
    ROS_INFO( "\n set_Vel = %f, %f \n get_Vel = %f, %f \n dif_Vel = %f, %f \n k = %f \n att(N,E) = %f, %f \n",
	      vel_setPoint.linear.x,
	      vel_setPoint.linear.y,
	      vel_mesure.vector.x,
	      vel_mesure.vector.y,
	      difX,
	      difY,
	      k,
	      attN * 180.0 / M_PI,
	      attE * 180.0 / M_PI );     

    
/*     ROS_INFO( "\n diff_x: %f = \n set_vx: %f - \n get_vx: %f ",  
	       difX,
	       vel_setPoint.linear.x,
	       vel_mesure.vector.x );

     ROS_INFO( "\n diff_y: %f = \n set_vy: %f - \n get_vy: %f ",  
	       difY,
	       vel_setPoint.linear.y,
	       vel_mesure.vector.y );
     
     ROS_INFO( "HORIZONTAL_ANGLE [graus], k = %f: \n att_x: %f \n att_y: %f ",  
	       k,
	       attX * 180.0 / M_PI,
	       attY * 180.0 / M_PI);  */   
  }

  sensor_msgs::Joy controlAtt;
  
  // Angulo X - Varia de -30 a 30 graus () - ROLL em em relação ao mundo NEU da DJI
  // valor positivo vai em direcao ao East
  controlAtt.axes.push_back(attE); 
  
  // Angulo Y - Varia de -30 a 30 graus - PITCH em em relação ao mundo NEU da DJI
  // valor positivo vai em direcao ao North
  controlAtt.axes.push_back(attN);

  // Velocidade Z - de -4 a 4 m/s
  controlAtt.axes.push_back(vel_setPoint.linear.z);
  
  // Velocidade angular Z - de -100 a 100 graus/s
  controlAtt.axes.push_back(vel_setPoint.angular.z);
  
  //controlAtt.buttons.push_back(0);
  //controlAtt.header.frame_id = "ctrl_rpz_yaw";

  uint8_t flag = (DJISDK::HORIZONTAL_ANGLE |
              DJISDK::VERTICAL_VELOCITY |
              DJISDK::YAW_RATE |
              (isBodyReferece ? DJISDK::HORIZONTAL_BODY : DJISDK::HORIZONTAL_GROUND) |
              DJISDK::STABLE_ENABLE);

  controlAtt.axes.push_back(flag);

  m_ctrlBrakePub.publish(controlAtt);
  
  
  
  //m_ctrlRpy.publish(controlAtt);
  //ros::spinOnce();
}

geometry_msgs::Twist M100::getVel()
{
  geometry_msgs::Twist vel;

  if( !isGpsOk() )
    ROS_ERROR("M100::getVel(): GPS is not Ok - GPS Health = %d",m_gps_health);

  vel.linear = m_current_vel.vector;
  vel.angular = m_current_imu.angular_velocity;
  return vel;
}

// Para o quadrirrotor
void M100::stop()
{
  geometry_msgs::Twist vel; // Construtor inicializa com zeros.
  setVel(vel);
}

// Espera até usuário desativar o F mode.
void M100::waitNotAutonomousMode()
{
  ros::Rate loop_rate(2);
  while (ros::ok())
  {

    if( !isAutonomous() )
    {
      ROS_INFO("F mode: OFF.");
      return;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

bool M100::takeoff_land(int task)
{
  if(m_readOnly)
    return false;
  
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  m_drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool M100::obtain_control()
{
  if(m_readOnly)
    return true;

  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  m_sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool M100::set_local_pos_ref()
{
  if(m_readOnly)
    return false;

  bool isOk = isGpsOk();
  if( !isOk )
    ROS_ERROR("M100::set_local_pos_ref(): GPS is not Ok - GPS Health = %d",m_gps_health);

  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  m_set_local_pos_ref_service.call(localPosReferenceSetter);

  return isOk;
}


void M100::flyToGPSPos(sensor_msgs::NavSatFix gps_pos)
{
  if(m_readOnly)
    return;

}

sensor_msgs::NavSatFix M100::getCurrGPSPos()
{
  if( !isGpsOk() )
    ROS_ERROR("M100::getCurrGPSPos(): GPS is not Ok - GPS Health = %d",m_gps_health);

  return m_current_gps;
}

geometry_msgs::PointStamped M100::getCurrLocalPos()
{
  if( !isGpsOk() )
    ROS_ERROR("M100::getCurrLocalPos(): GPS is not Ok - GPS Health = %d",m_gps_health);

  return m_local_position;
}

void M100::flyToPos(geometry_msgs::Pose pos)
{
}

geometry_msgs::Pose M100::getCurrPos()
{

  geometry_msgs::Pose pos;
  
  ////////////////////////////////////
  //          DEBUG HENRIQUE
  ////////////////////////////////////
  if( m_ukf_mode && !m_att_control_test ) 
  {
    pos = m_current_ukf_odom.pose.pose;
  }
  else
  {
    geometry_msgs::Point localOffset;
    sensor_msgs::NavSatFix currPose = getCurrGPSPos();
    localOffsetFromGpsOffset(localOffset, currPose, m_start_gps_location);
    
    pos.position = localOffset;
    
    /*if( m_ukf_mode ) // Gabiarra Henrique - Não sei porque os eixos x e y trocam quando está no ukf_mode
    {
      pos.position = localOffset;
      pos.position.x = localOffset.y;
      pos.position.y = localOffset.x;
    }
    else
    {
      pos.position = localOffset;
    }*/
    
    
    pos.orientation = m_current_atti;
  }
  
  
  //tf::Quaternion init_orient, curr_orient;

 /* ROS_INFO( "initial_orient: x = %f, x = %f, x = %f, x = %f",  
     m_initial_orientation.x,
     m_initial_orientation.y,
     m_initial_orientation.z,
     m_initial_orientation.w );*/

  //tf::quaternionMsgToTF( m_initial_orientation, init_orient );
  /*ROS_INFO( "current_atti: x = %f, x = %f, x = %f, x = %f",  
     m_current_atti.x,
     m_current_atti.y,
     m_current_atti.z,
     m_current_atti.w );*/

  //tf::quaternionMsgToTF( m_current_atti, curr_orient );

  //tf::quaternionTFToMsg( curr_orient * init_orient.inverse(), pos.orientation );

 /* ROS_INFO( "pos.orient: x = %f, x = %f, x = %f, x = %f",  
     pos.orientation.x,
     pos.orientation.y,
     pos.orientation.z,
     pos.orientation.w );*/


//return m_initial_orientation * m_current_atti.inverse();

/*  pos.x = localOffset.x;
  pos.y = localOffset.y;
  pos.z = localOffset.z;
  pos.yaw = this->toEulerAngle(m_current_atti).z - m_initial_orientation;*/
  return pos;
}

nav_msgs::Odometry M100::getOdometry()
{
  nav_msgs::Odometry odom;
  odom.pose.pose = getCurrPos();
  odom.twist.twist = getVel();
  return odom;
}

bool M100::initZeroPos()
{
  ROS_INFO("initZeroPos(): INICIO ");
  if( !isGpsOk() )
  {
    return false;
  }


  set_local_pos_ref();

  m_start_gps_location = getCurrGPSPos();
  double sumGps = 
     m_start_gps_location.latitude +
     m_start_gps_location.longitude +
     m_start_gps_location.altitude ;
  if( sumGps == 0 )
  {
    ROS_ERROR( "M100::initZeroPos(): GPS Not Properly initialized.");
    return false;
  }

  /*ROS_INFO( "initZeroPos(): start_gps_location: x = %f, x = %f, x = %f",  
     m_start_gps_location.latitude,
     m_start_gps_location.longitude,
     m_start_gps_location.altitude);*/

  //initZeroHeading();

  /*ROS_INFO( "initZeroPos(): initial_orient: x = %f, x = %f, x = %f, x = %f",  
     m_initial_orientation.x,
     m_initial_orientation.y,
     m_initial_orientation.z,
     m_initial_orientation.w );*/

  return true;
}

bool M100::initZeroHeading()
{
  double sumAtt = m_current_atti.x + m_current_atti.y + m_current_atti.z + m_current_atti.w;
  if( sumAtt == 0 )
  {
    ROS_ERROR( "M100::initZeroHeading(): Quaternion Not Properly Normalized.");
    return false;
  }
 
  m_initial_orientation = m_current_atti;
  //m_initial_orientation = this->toEulerAngle(m_current_atti).z; 
  return true;
}


int M100::getStatus()
{
  return m_flight_status;
}

int M100::showStatus()
{
  switch(getStatus())
  {
    case DJISDK::M100FlightStatus::M100_STATUS_ON_GROUND:
      ROS_INFO("M100_STATUS_ON_GROUND");
      break;
    case DJISDK::M100FlightStatus::M100_STATUS_TAKINGOFF:
      ROS_INFO("M100_STATUS_TAKINGOFF");
      break;
    case DJISDK::M100FlightStatus::M100_STATUS_IN_AIR:
      ROS_INFO("M100_STATUS_IN_AIR");
      break;
    case DJISDK::M100FlightStatus::M100_STATUS_LANDING:
      ROS_INFO("M100_STATUS_LANDING");
      break;
    case DJISDK::M100FlightStatus::M100_STATUS_FINISHED_LANDING:
      ROS_INFO("M100_STATUS_FINISHED_LANDING");
      break;
    default:
      ROS_ERROR("Unexpected status. M100_STATUS = %d", m_flight_status);
  }
  return m_flight_status;
}

bool M100::isFlying()
{
  return( m_flight_status == DJISDK::M100FlightStatus::M100_STATUS_IN_AIR );
}

bool M100::isTakingOff()
{
  return( m_flight_status == DJISDK::M100FlightStatus::M100_STATUS_TAKINGOFF );
}

bool M100::isLanding()
{
  return( m_flight_status == DJISDK::M100FlightStatus::M100_STATUS_LANDING );
}


// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/
// Ex.: localOffsetFromGpsOffset(localOffset, current_gps, start_gps_location);
void M100::localOffsetFromGpsOffset(geometry_msgs::Point&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
}

geometry_msgs::Point M100::toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Point ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

//-----------------------------------------------------------------------
//Novas funções
void M100::setWaypointDefaults(DJI::OSDK::WayPointSettings* wp)
{
  wp->damping         = 0;
  wp->yaw             = 0;
  wp->gimbalPitch     = 0;
  wp->turnMode        = 0;
  wp->hasAction       = 0;
  wp->actionTimeLimit = 100;
  wp->actionNumber    = 0;
  wp->actionRepeat    = 0;
  for (int i = 0; i < 16; ++i)
  {
    wp->commandList[i]      = 0;
    wp->commandParameter[i] = 0;
  }
}

std::vector<DJI::OSDK::WayPointSettings>
M100::generateWaypointsPolygon(DJI::OSDK::WayPointSettings* start_data, float64_t increment, int num_wp)
{
  // Let's create a vector to store our waypoints in.
  std::vector<DJI::OSDK::WayPointSettings> wp_list;

  // Some calculation for the polygon
  float64_t extAngle = 2 * M_PI / num_wp;

  // First waypoint
  start_data->index = 0;
  wp_list.push_back(*start_data);

  // Iterative algorithm
  for (int i = 1; i < num_wp; i++)
  {
    DJI::OSDK::WayPointSettings wp;
    setWaypointDefaults(&wp);
	
    DJI::OSDK::WayPointSettings* prevWp = &wp_list[i - 1];
	
	wp.index     = i;
    wp.latitude  = (prevWp->latitude + (increment * cos(i * extAngle)));
    wp.longitude = (prevWp->longitude + (increment * sin(i * extAngle)));
    wp.altitude  = (prevWp->altitude + 1);
    wp_list.push_back(wp);
  }

  // Come back home
  start_data->index = num_wp;
  wp_list.push_back(*start_data);

  return wp_list;
}


std::vector<DJI::OSDK::WayPointSettings> M100::createWaypoints(int numWaypoints, float64_t distanceIncrement, float32_t start_alt)
{
  // Create Start Waypoint
  DJI::OSDK::WayPointSettings start_wp;
  setWaypointDefaults(&start_wp);
  start_wp.latitude  = gps_pos.latitude;
  start_wp.longitude = gps_pos.longitude;
  start_wp.altitude  = start_alt;
  ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\n", gps_pos.latitude,
           gps_pos.longitude, start_alt);

  std::vector<DJI::OSDK::WayPointSettings> wpVector =
    M100::generateWaypointsPolygon(&start_wp, distanceIncrement, numWaypoints);
  return wpVector;
}


void M100::uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& wp_list, int responseTimeout, dji_sdk::MissionWaypointTask& waypointTask)
{
  dji_sdk::MissionWaypoint waypoint;
  for (std::vector<WayPointSettings>::iterator wp = wp_list.begin();
       wp != wp_list.end(); ++wp)
  {
    ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\n ", wp->latitude,
             wp->longitude, wp->altitude);
    waypoint.latitude            = wp->latitude;
    waypoint.longitude           = wp->longitude;
    waypoint.altitude            = wp->altitude;
    waypoint.damping_distance    = 0;
    waypoint.target_yaw          = 0;
    waypoint.target_gimbal_pitch = 0;
    waypoint.turn_mode           = 0;
    waypoint.has_action          = 0;
    waypointTask.mission_waypoint.push_back(waypoint);
  }
}


ServiceAck M100::initWaypointMission(dji_sdk::MissionWaypointTask& waypointTask)
{
  dji_sdk::MissionWpUpload missionWpUpload;
  missionWpUpload.request.waypoint_task = waypointTask;
  waypoint_upload_service.call(missionWpUpload);
  if (!missionWpUpload.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", missionWpUpload.response.cmd_set,
             missionWpUpload.response.cmd_id);
    ROS_WARN("ack.data: %i", missionWpUpload.response.ack_data);
  }
  return ServiceAck(
    missionWpUpload.response.result, missionWpUpload.response.cmd_set,
    missionWpUpload.response.cmd_id, missionWpUpload.response.ack_data);
}

ServiceAck M100::missionAction(DJI::OSDK::DJI_MISSION_TYPE type, DJI::OSDK::MISSION_ACTION   action)
{
  dji_sdk::MissionWpAction missionWpAction;
  dji_sdk::MissionHpAction missionHpAction;
  switch (type)
  {
    case DJI::OSDK::WAYPOINT:
      missionWpAction.request.action = action;
      waypoint_action_service.call(missionWpAction);
      if (!missionWpAction.response.result)
      {
        ROS_WARN("ack.info: set = %i id = %i", missionWpAction.response.cmd_set,
                 missionWpAction.response.cmd_id);
        ROS_WARN("ack.data: %i", missionWpAction.response.ack_data);
      }
      return { missionWpAction.response.result,
               missionWpAction.response.cmd_set,
               missionWpAction.response.cmd_id,
               missionWpAction.response.ack_data };
    case DJI::OSDK::HOTPOINT:
      missionHpAction.request.action = action;
      hotpoint_action_service.call(missionHpAction);
      if (!missionHpAction.response.result)
      {
        ROS_WARN("ack.info: set = %i id = %i", missionHpAction.response.cmd_set,
                 missionHpAction.response.cmd_id);
        ROS_WARN("ack.data: %i", missionHpAction.response.ack_data);
      }
      return ServiceAck(
        missionHpAction.response.result, missionHpAction.response.cmd_set,
        missionHpAction.response.cmd_id, missionHpAction.response.ack_data);
  }
}


bool M100::runWaypointMission(uint8_t numWaypoints, int responseTimeout)
{
  ros::spinOnce();

  // Waypoint Mission : Initialization
  dji_sdk::MissionWaypointTask waypointTask;
  M100::setWaypointInitDefaults(waypointTask);

  // Waypoint Mission: Create Waypoints
  float64_t increment = 0.000001 / C_PI * 180;
  float32_t start_alt = 10;
  ROS_INFO("Creating Waypoints..\n");
  std::vector<WayPointSettings> generatedWaypts =
    createWaypoints(numWaypoints, increment, start_alt);

  // Waypoint Mission: Upload the waypoints
  ROS_INFO("Uploading Waypoints..\n");
  uploadWaypoints(generatedWaypts, responseTimeout, waypointTask);

  // Waypoint Mission: Init mission
  ROS_INFO("Initializing Waypoint Mission..\n");
  if (initWaypointMission(waypointTask).result)
  {
    ROS_INFO("Waypoint upload command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending waypoint upload command");
    return false;
  }

  // Waypoint Mission: Start
  if (missionAction(DJI_MISSION_TYPE::WAYPOINT,MISSION_ACTION::START).result)
  {
    ROS_INFO("Mission start command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending mission start command");
    return false;
  }

  return true;
}



void M100::setWaypointInitDefaults(dji_sdk::MissionWaypointTask& waypointTask)
{
  waypointTask.velocity_range     = 10;
  waypointTask.idle_velocity      = 5;
  waypointTask.action_on_finish   = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
  waypointTask.mission_exec_times = 1;
  waypointTask.yaw_mode           = dji_sdk::MissionWaypointTask::YAW_MODE_AUTO;
  waypointTask.trace_mode         = dji_sdk::MissionWaypointTask::TRACE_POINT;
  waypointTask.action_on_rc_lost  = dji_sdk::MissionWaypointTask::ACTION_AUTO;
  waypointTask.gimbal_pitch_mode  = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;
}




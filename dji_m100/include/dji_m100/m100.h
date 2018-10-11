/** @file m100.h
 *  @version 1.0
 *  @date Oct, 2017
 *
 *  @brief
 *  Class to control DJI M100 quadrotor.
 *
 *  @copyright 2017 CORO-UFMG. All rights reserved.
 *
 */

#ifndef M100_H
#define M100_H

// ROS includes
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <dji_sdk/dji_sdk_node.h>


#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
//#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

//using namespace DJI::OSDK;

 typedef struct ServiceAck
	{
		  bool         result;
		  int          cmd_set;
		  int          cmd_id;
		  unsigned int ack_data;
		  ServiceAck(bool res, int set, int id, unsigned int ack)
			: result(res)
			, cmd_set(set)
			, cmd_id(id)
			, ack_data(ack)
		  {
		  }
		  ServiceAck()
		  {
		  }
	} ServiceAck;

namespace coro
{

class M100{
	
	

   private:
    ros::NodeHandle m_nh;
    bool m_readOnly;
    bool m_ukf_mode;
    bool m_debug;
    bool m_att_control_test;

    sensor_msgs::NavSatFix      m_start_gps_location;
    geometry_msgs::Quaternion   m_initial_orientation;
	sensor_msgs::NavSatFix gps_pos;

    //services and callbacks
    uint8_t                     m_flight_status;
    sensor_msgs::NavSatFix      m_current_gps;
    uint8_t                     m_gps_health; // Is ok if value >= 3. 
    geometry_msgs::Quaternion   m_current_atti;
    sensor_msgs::Joy            m_current_rc; 
    geometry_msgs::Vector3Stamped m_current_vel; 
    geometry_msgs::Vector3Stamped m_current_ukf_vel; 
    nav_msgs::Odometry		m_current_ukf_odom; 
    sensor_msgs::Imu            m_current_imu;
    geometry_msgs::PointStamped m_local_position;
    
    

    ros::Subscriber m_attitudeSub;
    ros::Subscriber m_gpsSub;
    ros::Subscriber m_gpsHealthSub;
    ros::Subscriber m_flightStatusSub;
    ros::Subscriber m_rcSub;
    ros::Subscriber m_velSub;
    ros::Subscriber m_ukf_velSub;
    ros::Subscriber m_ukf_odomSub;
    ros::Subscriber m_imuSub;
    ros::Subscriber m_localPositionSub;

    ros::Publisher              m_ctrlBrakePub;
    ros::Publisher              m_ctrlRpy;
    ros::ServiceClient          m_sdk_ctrl_authority_service;
    ros::ServiceClient          m_drone_task_service;
    ros::ServiceClient          m_set_local_pos_ref_service;
    
    void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg);
    void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);
    void rc_callback(const sensor_msgs::Joy::ConstPtr& msg); 
    void vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
    void ukf_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
    void ukf_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
    void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);

    bool takeoff_land(int task);
    bool obtain_control();
    bool set_local_pos_ref();
    int getStatus();


    static void localOffsetFromGpsOffset(geometry_msgs::Point&  deltaNed,
                            sensor_msgs::NavSatFix& target,
                            sensor_msgs::NavSatFix& origin);


   public:
   

    M100( ros::NodeHandle &nh, bool readOnly = false );

    bool takeoff();
    bool land();
    bool isAutonomous(); // if is in remote controler F and got authorization
    bool isGpsOk(); //Have to be 'true' to getVel(), getCurrPos() and getOdometry() work properly.
    void waitNotAutonomousMode();
	
	
    
    // Set drone velocity.
    // If 'isBodyReferece = true'' the velocity is relative to the drone frame.
    // If not, the velocity is relative to the ENU ground frame (GPS).
    void setVel(geometry_msgs::Twist vel, bool isBodyReferece = true);
    
    // Return the linear and angular velocities.
    // Linear Velocity (from GPS). in ENU ground frame.
    // The velocity is valid only when gps_health >= 3 (isGpsOk() = 'true').
    // Angular Velocity (from IMU). IMU data including raw gyro reading in FLU body frame.
    geometry_msgs::Twist getVel();

    // Set all velocities to zero.
    void stop();
    
    

    // Controle Velocidade atuando na attitude para controlar as velocidades X e Y.
    // Assim fica independente do GPS.
    // vel_setPoint -> Velocidades desejadas.
    // vel_mesure   -> Entrada de velocidades medidas pelo sensor.
    // k            -> Ganho do controlador
    // If 'isBodyReferece = true'' the velocity is relative to the drone frame.
    // If not, the velocity is relative to the ENU ground frame (GPS).
    void ctrlVel(geometry_msgs::Twist vel_setPoint, geometry_msgs::Vector3Stamped vel_mesure, double k, bool isBodyReferece);
    
    
    void flyToGPSPos(sensor_msgs::NavSatFix gps_pos); // no blocking function
    //Position in ENU ground frame (GPS). Is valid only when isGpsOk() = 'true'.
    sensor_msgs::NavSatFix getCurrGPSPos(); 
    
    void flyToPos(geometry_msgs::Pose pos); // in relation to where the drone was turned-on

    // Return the drone position and orientation 
    // in relation to where the drone was turned-on or
    // when called the functions initZeroPos or initZeroHeading.
    geometry_msgs::Pose getCurrPos(); 

    // Return the drone position in relation to where the drone was turned-on or
    // when called the functions initZeroPos.
    // Only the main object whit 'readOnly = false' can reset the zeroPos.
    geometry_msgs::PointStamped getCurrLocalPos();

    // Linear and angular velocities and position in ENU ground frame.
    nav_msgs::Odometry getOdometry();
    
    int getGpsHealth() { return( (int)m_gps_health ); }

    bool initZeroPos(); // make the current position to be (0,0,0,0)
    bool initZeroHeading(); // make the current heading to be 0 


    // Show the drone fly state: 
    // ON_GROUND, TAKINGOFF, IN_AIR, LANDING or FINISHED_LANDING.
    int showStatus();

    bool isFlying();
    bool isTakingOff();
    bool isLanding();
    
    static geometry_msgs::Point toEulerAngle(geometry_msgs::Quaternion quat);
	
	
	//Added
	void setWaypointDefaults(WayPointSettings* wp);
	void setWaypointInitDefaults(dji_sdk::MissionWaypointTask& waypointTask);
	
	std::vector<DJI::OSDK::WayPointSettings> createWaypoints( int numWaypoints, DJI::OSDK::float64_t distanceIncrement, DJI::OSDK::float32_t start_alt);
	
	std::vector<DJI::OSDK::WayPointSettings> generateWaypointsPolygon(DJI::OSDK::WayPointSettings* start_data, float64_t increment, int num_wp);
	
	void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& wp_list, int responseTimeout, dji_sdk::MissionWaypointTask& waypointTask);
	
	ServiceAck initWaypointMission(dji_sdk::MissionWaypointTask& waypointTask);
	
	ServiceAck missionAction(DJI::OSDK::DJI_MISSION_TYPE type, DJI::OSDK::MISSION_ACTION action);
	
	bool runWaypointMission(uint8_t numWaypoints, int responseTimeout);	
	
	
};

} // coro 

#endif // M100_H

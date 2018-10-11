/** @file landing_deck.cpp
 *  @version 1.0
 *  @date Oct, 2017
 *
 *  @brief
 *  Voo simples para o sistema de controle autonamo pouso em um alvo.
 *
 *  @copyright 2017 CORO-UFMG. All rights reserved.
 *
 */

#include "dji_m100/landing_deck.h"
 
//const float deg2rad = C_PI/180.0;    
//const float rad2deg = 180.0/C_PI;


nav_msgs::Odometry curr_drone_pose;
bool estimated_pose = false;
void GetDronePose(const nav_msgs::Odometry::ConstPtr &msg) {
  curr_drone_pose = *msg;
  estimated_pose = true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "landing_deck_node");
  ros::NodeHandle nh;
  
  // Carrega parametros
  double vel, t_up, t_wait, t_desloc, t_est;
  nh.param("/landing_deck_node/vel", vel, 1.0);
  nh.param("/landing_deck_node/t_up", t_up, 7.0);
  nh.param("/landing_deck_node/t_wait", t_wait, 1.0);
  nh.param("/landing_deck_node/t_desloc", t_desloc, 10.0);
  nh.param("/landing_deck_node/t_desloc", t_est, 3.0);
  ROS_INFO("params: t_wait = %f, t_desloc = %f, vel = %f", t_wait, t_desloc, vel);

  coro::M100 m100(nh);

  ros::Subscriber drone_pos_sub = nh.subscribe<nav_msgs::Odometry>("/dronePose/odom", 1, GetDronePose);

  ROS_INFO("Selec (F) mode on RC to start mission.");
  ros::Rate loop_start(1);
  while (ros::ok())
  {
    if( m100.isAutonomous() )
    {
        ROS_INFO("F mode: ON.");
        landing_deck( m100, vel, t_up, t_wait, t_desloc, t_est);
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
int landing_deck( coro::M100 &m100, double vel, double t_up, double t_wait, double t_desloc, double t_est)
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
  geometry_msgs::Twist last_tvel;
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
        if (estimated_pose)
        {
          step++;
          ROS_INFO("step %d: wait.", step);
          start_time = ros::Time::now();
        }else if( elapsed_time > ros::Duration(t_desloc) )
        {
          step = 6;
          ROS_INFO("step %d: wait.", step);
          start_time = ros::Time::now();
        }
        break;

      case 4: // WAIT
        m100.stop();
        if( elapsed_time > ros::Duration(t_wait/2) )
        {
          step++;
          ROS_INFO("step %d: land on deck.", step);
          start_time = ros::Time::now();
        }
        break;


      case 5: // LAND ON DECK
          {
            ROS_INFO("State 5");     
            // alpha=2, beta = 1.5, k=2
            double alpha = 0.4;
            double beta = 0.3;
            double k = vel;

            if (estimated_pose)
            {
              estimated_pose = false;
              start_time = ros::Time::now();

              double x = curr_drone_pose.pose.pose.position.x;
              double y = curr_drone_pose.pose.pose.position.y;
              double z = curr_drone_pose.pose.pose.position.z;

              ROS_INFO("x=%f, y=%f, z=%f", x, y, z);

              tvel.linear.x = x * (alpha + beta * z);
              tvel.linear.y = y * (alpha + beta * z);
              tvel.linear.z = z;

              double norm = sqrt(tvel.linear.x * tvel.linear.x +
                                tvel.linear.y * tvel.linear.y +
                                tvel.linear.z * tvel.linear.z);

              tvel.linear.x = k * tvel.linear.x / norm;
              tvel.linear.y = k * tvel.linear.y / norm;
              tvel.linear.z = k * tvel.linear.z / norm;
              last_tvel = tvel;

              m100.setVel( tvel );
            }else if (elapsed_time < ros::Duration(t_est))
            {
              double norm = sqrt(last_tvel.linear.x * last_tvel.linear.x +
                                last_tvel.linear.y * last_tvel.linear.y +
                                last_tvel.linear.z * last_tvel.linear.z);
              double x = curr_drone_pose.pose.pose.position.x;
              double y = curr_drone_pose.pose.pose.position.y;
              double z = curr_drone_pose.pose.pose.position.z;
              
              double last_dist = sqrt(pow(x,2) + pow(y,2) + pow(z,2));

              double aten = 1;


              double vref = std::min(1.0,k);

              if (fabs(z) < 2)
              {
                if (norm > 0.80 * vref)
                {
                  aten = 0.88;
                  std::cout <<"\ncase 1 : "<< norm;
                }else if (norm > 0.50 * vref)
                {
                  aten = 0.93;
                  std::cout <<"\ncase 2 : "<< norm;
                }else{
                  aten = 0.98;
                  std::cout <<"\ncase 3 : "<< norm;
                }
              }
              tvel.linear.x = aten * last_tvel.linear.x;
              tvel.linear.y = aten * last_tvel.linear.y;
              tvel.linear.z = aten * last_tvel.linear.z;

              last_tvel = tvel;
              m100.setVel( tvel );

            } else {
              step++;
              ROS_INFO("step %d: WAIT.", step);
              start_time = ros::Time::now();
            }
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
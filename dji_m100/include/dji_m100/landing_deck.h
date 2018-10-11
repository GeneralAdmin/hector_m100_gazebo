/** @file landing_deck.H
 *  @version 1.0
 *  @date Oct, 2017
 *
 *  @brief
 *  Voo simples para o sistema de controle autonamo de pouso.
 *
 *  @copyright 2017 CORO-UFMG. All rights reserved.
 *
 */

#ifndef landing_deck_H
#define landing_deck_H

#include "dji_m100/m100.h"

//#define C_EARTH (double)6378137.0
//#define C_PI (double)3.141592653589793
//#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

/*void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin);
geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);
*/

int landing_deck( coro::M100 &m100, double vel, double t_up, double t_wait, double t_desloc, double t_est );

#endif // landing_deck_H

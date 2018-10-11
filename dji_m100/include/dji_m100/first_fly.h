/** @file first_fly.H
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

#ifndef first_fly_H
#define first_fly_H

#include "dji_m100/m100.h"

//#define C_EARTH (double)6378137.0
//#define C_PI (double)3.141592653589793
//#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

/*void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin);
geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);
*/

int first_fly( coro::M100 &m100, double vel, double t_wait, double t_desloc );

#endif // first_fly_H

/*
 * PI.h
 *
 *  Created on: 10 Mar 2017
 *      Author: dinhngock6
 */

#ifndef _PI_H_
#define _PI_H_

typedef struct
{
   float Ts;
   float Kp;
   float Ki;
   float IN_max;
   float OUT_max;
   float heso_P;
   float heso_I;
   float REF;
   float INPUT;
   float ERROR;
   float OUT_P;
   float OUT_I;
   float OUT;
}PI_NORMAL_CLA;

#define PI_NORMAL_INT(v)            \
        v.heso_P = 0;                   \
        v.heso_I = 0;                   \
        v.REF = 0;                      \
        v.INPUT = 0;                    \
        v.ERROR = 0;                    \
        v.OUT_P = 0;                    \
        v.OUT_I = 0;                    \
        v.OUT = 0;


#define PI_NORMAL_MACRO(v)                                      \
        v.heso_P   = v.Kp*v.IN_max/v.OUT_max;                   \
        v.heso_I   = v.Ki*v.Ts*v.IN_max/v.OUT_max;              \
        v.OUT_P    = v.heso_P*v.ERROR;                          \
        v.OUT_I    = v.OUT_I + v.heso_I*v.ERROR;                \
        v.OUT      = v.OUT_P + v.OUT_I;                         \
        if(v.OUT > (1.0)) v.OUT = (1.0);                        \
        if(v.OUT < (-1.0)) v.OUT = (-1.0);

#define PI_NORMAL_V2_MACRO(v)                                   \
        v.heso_P   = v.Kp*v.IN_max/v.OUT_max;                   \
        v.heso_I   = v.Ki*v.Ts*v.IN_max/v.OUT_max;              \
        v.OUT_P    = v.heso_P*v.ERROR;                          \
        if(v.OUT >= 1.0 ||v.OUT <= -1.0 ) v.OUT_I = v.OUT_I;    \
        else v.OUT_I    = v.OUT_I + v.heso_I*v.ERROR;           \
        v.OUT      = v.OUT_P + v.OUT_I;                         \
        if(v.OUT > (1.0)) v.OUT = (1.0);                        \
        if(v.OUT < (-1.0)) v.OUT = (-1.0);

#define PI_VOLTAGE_MACRO(v)                                             \
        v.heso_P   = v.Kp*v.IN_max/v.OUT_max;                           \
        v.heso_I   = v.Ki*v.Ts*v.IN_max/v.OUT_max;                      \
        v.OUT_P    = v.heso_P*v.ERROR;                                  \
        v.OUT_I    = v.OUT_I + v.heso_I*v.ERROR;                        \
        v.OUT      = v.OUT_P + v.OUT_I;                                 \
        if(v.OUT > (1.5*Idm/Is_max)) v.OUT = (1.5*Idm/Is_max);          \
        if(v.OUT < (-1.5*Idm/Is_max)) v.OUT = (-1.5*Idm/Is_max);


#endif /* 2_LIBRUARY_INCLUDE_CONTROLLER_PI_H_ */

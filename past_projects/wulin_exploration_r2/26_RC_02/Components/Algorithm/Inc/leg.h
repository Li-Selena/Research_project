/* Legacy planar two-link utility; not used by R2_climb.
 * Local x/y are coordinates in the linkage plane, not chassis X/Y. */
#ifndef __LEG_H
#define __LEG_H

#include "struct_typedef.h"

#define LEG_REMOTE_X_MIN_MM  -180.0f
#define LEG_REMOTE_X_MAX_MM   180.0f
#define LEG_REMOTE_Y_MIN_MM  -180.0f
#define LEG_REMOTE_Y_MAX_MM   180.0f
#define LEG_REMOTE_H_MIN_MM  -55.0f
#define LEG_REMOTE_H_MAX_MM   55.0f

#define LEGINIT_OFFSET        0.30543261f
typedef struct{
	float theta1;
	float theta2;
}leg;
// extern float legInit_offest;
extern leg lf_leg;
extern leg rf_leg;
extern float  lf_last_theta1;
extern float  lf_last_theta2;
extern float  rf_last_theta1;
extern float  rf_last_theta2;
extern float lb_leg;
extern float rb_leg;

extern int8_t LEG_Cmd ;
extern float bleg_theta ;
extern float fleg_high ;

float Q_rsqrt(float number);
int InverseKinematics_Continuous(float x, float y,
                                 float last_theta1, float last_theta2,
                                 float *theta1, float *theta2);
void backLeg_Left(float h,float*theta);
void backLeg_Right(float h,float*theta);


#endif




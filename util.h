/*
 *A common include defining useful macros and definitions
 *
 */



#ifndef _UTIL_H
#define _UTIL_H

#define PI 3.14159265
#define toRad(deg) (deg*PI/180.0)

#define MAX_RANGE 5.0

#define SCALE 1


#define UNEXPLORED 1

#define X_RES	0.066
#define Y_RES	0.066

#define WIN_X 2000/SCALE
#define WIN_Y 700/SCALE

#define WIN_X_METERS (WIN_X *SCALE * X_RES)
#define WIN_Y_METERS (WIN_Y *SCALE * Y_RES)

#define XOFFSET WIN_X_METERS/2 //set origin in the middle
#define YOFFSET WIN_Y_METERS/2

#define windowX(x) ((x+XOFFSET)/X_RES/SCALE)
#define windowY(y) ((y+YOFFSET)/Y_RES/SCALE)
inline double wrap(double angle){
	
	double ret = angle;
	while(ret < -PI) ret += 2*PI;
	while(ret > PI) ret -= 2*PI;
	return ret;

}

#endif

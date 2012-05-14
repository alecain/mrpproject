

#ifndef _UTIL_H
#define _UTIL_H

#define PI 3.14159265
#define toRad(deg) (deg*PI/180.0)

inline double wrap(double angle){
	double ret = angle;
	while (ret<-PI) ret+= 2*PI;
	while (ret>PI) ret+= -2*PI;
	return ret;

}

#endif


/*
 * particle for localization 
 *
 *	Author: Andrew LeCain
 */
#ifndef _PARTICLE_H
#define _PARTICLE_H

#include "Vector2d.h"
#include <vector>
#include "map.h"
#include "Scan.h"
using namespace std;



class Particle{
	public:
		Particle(double x, double y, double theta);
		vector<Particle> update(double dlinear, double dtheta, int toSpawn);
		Particle clone(void);
		void draw(void);
		double score(Map *map, Scan *scan);

		Vector2d origin;
		double theta;
		static double thetaThetaCov;
		static double thetaLinearCov;
		static double linearLinearCov;
		static double linearThetaCov;

		static void setTTCovariance(double thetaThetaCov); 
		static void setTLCovariance(double thetaLinearCov); 
		static void setLLCOvariance(double linearLinearCov); 
		static void setLTCovariance(double linearThetaCov); 
		double scoreVal;	
};
#endif

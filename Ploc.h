
/*
 * particle based localization node 
 *
 *	Author: Andrew LeCain
 */

#ifndef _PLOC_H
#define _PLOC_H


#include <list>

#include "util.h"
#include "Particle.h"
#include "Vector2d.h"
#include "Scan.h"
#include "map.h"

class Pose{
	public:
	double x,y,theta;
	double sigx,sigy,sigtheta;//standard deviations
	void draw(void);
};

class Ploc{
	public:
		Ploc(int minParticles, int maxParticles, Map* map);
		void updateParticles(double dlinear, double dtheta); 
		void replenishParticles(void);
		void scoreParticles(Scan *scans);
		void pruneParticles();
		Pose getPose(int toAverage=25);
		list<Particle> particles;
	private:
		Map *map;
		int minParticles;
		int maxParticles;
		double avgscore;
		double bestScore;
};
#endif

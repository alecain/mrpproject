
/*
 * particle based localization node
 *
 *	Author: Andrew LeCain
 */

#ifndef _PLOC_H
#define _PLOC_H

using namespace PlayerCc;
using namespace std;

#include <list>
#include <libplayerc++/playerc++.h>

#include "util.h"
#include "Particle.h"
#include "Vector2d.h"
#include "Scan.h"
#include "map.h"

class Pose{
	public:
	Vector2d origin;
	Vector2d odomTransform;
	double theta;
	double thetaodom;
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
		Pose getPose(Position2dProxy *pp, int toAverage=25);
		list<Particle> particles;
	private:
		Map *map;
		int minParticles;
		int maxParticles;
		double avgscore;
		double bestScore;
};
#endif

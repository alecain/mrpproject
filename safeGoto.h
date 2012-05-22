

#ifndef _SAFEGOTO_H
#define _SAFEGOTO_H

#define SCANS_TO_PROCESS 50
#define GOAL_TOLERANCE	.1
#define SEEK_FORCE	.5
#define MAX_OBS_FORCE	SEEK_FORCE 

#include "util.h"
#include "Scan.h"
#include "Ploc.h"
#include <libplayerc++/playerc++.h>

using namespace PlayerCc;
/**
 ****************************************************************************
 *Constants
 ****************************************************************************
 */

class SafeGoTo{
	public:
		SafeGoTo(void); //spawns a new thread that computes velocity to apply
		int update(Scan *scan, Pose *pose); //updates the scan to be used by the navigator
		void goTo(Vector2d goal); //updates the current goal of the goto function
		void applyVelocity(Position2dProxy *pp); //applies the current velocity to the scan
		void drawVelocity(Pose *pose); //applies the current velocity to the scan
		
		static const double dt;//time quantum
		static const double maxSpeed;
		static const double maxTwist;

		static const double laserRes;
		static const double laserMin;
		static const double laserMax;

	private:
		Vector2d avoid(Vector2d position, Vector2d toAvoid);
		Vector2d Velocity;
		Vector2d goal;



};
#endif

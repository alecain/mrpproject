/*
 * safeGoto modified from
 * Homework #2
 *
 *Andrew LeCain
 *
 */


#include <iostream>
#include <iomanip>
#include <math.h>
#include <ostream>
#include <libplayerc++/playerc++.h>
#include <vector>

#include "util.h"
#include "safeGoto.h"
#include "Vector2d.h"

using namespace PlayerCc;
/**
 ****************************************************************************
 *Globals
 ****************************************************************************
 */


using namespace PlayerCc;

/*
 ****************************************************************************
 *Navigation
 ****************************************************************************
 */

const double SafeGoTo::dt = 0.1;//time quantum
const double SafeGoTo::maxSpeed = .75;// m/s
const double SafeGoTo::maxTwist = .6;// radians per second (.157 per tick)

const double SafeGoTo::laserRes=0.00613593; //Is this right?
const double SafeGoTo::laserMin = -1.932817852;
const double SafeGoTo::laserMax = 1.932817852;

SafeGoTo::SafeGoTo(void){
		Velocity= Vector2d(0,0);
		goal= Vector2d(0,0);
}
//Produces a Vector2d away from the given obstacle
Vector2d SafeGoTo::avoid(Vector2d position, Vector2d toAvoid){

	Vector2d ret=position.minus(toAvoid);
	ret.norm();
	double dist = position.distance(toAvoid);
	ret=ret.divide(dist*dist);

	return ret;
}

void SafeGoTo::goTo(Vector2d g){
	goal=g;
}


int SafeGoTo::update(Scan *scan, Pose *pose){//updates the scan to be used by the navigator
	Velocity = Velocity.minus(Velocity);
	double distance;
	if( (distance = goal.distance(pose->origin)) > GOAL_TOLERANCE){

		//Vector2d2d representing our current heading
		Vector2d heading(1,0);
		heading = heading.rotateAbs(pose->theta);

		Vector2d goalForce;
		goalForce = goal.minus(pose->origin);
		goalForce = goalForce.norm().times(SEEK_FORCE); //normalize the force, then scale by seek weight
		goalForce = goalForce.rotate(-pose->theta); //rotate the force to be relative to the robot


		int toSkip = scan->scans.size()/SCANS_TO_PROCESS;

		if(distance > .5){ //only avoid obstacles if we're far from our goal.. not ideal, but easier than changing weights.
			//for each scan in scan
			for (vector<ScanNode>::iterator it = scan->scans.begin(); it < scan->scans.end(); it+=toSkip){
				double dist =it->range;
				if (dist < 1 && distance > .01 ){
					Vector2d position(0,0); //our position
					//generate a Vector2d pointing towards the object (from us as origin)
					Vector2d obs = it->getObstacle();

					//produce a Vector2d pointing away inversely proportional to ditance
					Vector2d force = avoid(position, obs);

					//scale vector so it isn't a huge sum
					force = force.divide(SCANS_TO_PROCESS);

					//pay more attention to the vectors near our current heading
					force = force.times(cos((obs.getAngle())));

					//cap force to maxForce
					if (force.len() > MAX_OBS_FORCE) force = force.norm().times(MAX_OBS_FORCE);

					if (force.dot(Vector2d(0,1)) > 0 ){
						//avoid each of these sensor readings.
						Velocity = Velocity.plus(force);
					}
			}

		}
	} else{
		//goalForce = goalForce.divide(10);
	}

	//add a constant attractive force towards our goal
	Velocity = Velocity.plus(goalForce);

	//finally, divide by two to average with the previous run.
}else{
}
}

void SafeGoTo::applyVelocity(Position2dProxy* pp ){
	double turnRate = (Velocity.getAngle() ) *.9;
	if (turnRate > this->maxTwist) turnRate=this->maxTwist;
	if (turnRate < -this->maxTwist) turnRate=-this->maxTwist;

	double speed = Velocity.len() * .9;
	if (speed>maxSpeed) speed = maxSpeed;

	if(Velocity.getAngle() > PI*3/4 ||  Velocity.getAngle() < -PI*3/4){
		pp->SetSpeed(-4*speed,-turnRate);
	}
	if(Velocity.getAngle() > PI/2 ||  Velocity.getAngle() < -PI/2){
		pp->SetSpeed(0,turnRate);

	}else if(Velocity.getAngle() > PI/3 ||  Velocity.getAngle() < -PI/3){
		pp->SetSpeed(speed*.3,turnRate);

	}else if(Velocity.getAngle() > .1 ||  Velocity.getAngle() < -.1){
		//angle is not small. turn and drive slowly
		pp->SetSpeed(speed * .9,turnRate);
	}else{
		pp->SetSpeed(speed , turnRate);
	}
}

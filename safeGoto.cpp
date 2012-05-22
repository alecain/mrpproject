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
const double SafeGoTo::maxSpeed = .5;// m/s
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
	if (dist != 0){
		ret=ret.divide(dist*dist);
	}

	return ret;
}

void SafeGoTo::goTo(Vector2d g){
	goal=g;
}


int SafeGoTo::update(Scan *scan, Pose *pose){//updates the scan to be used by the navigator
	double distance;
	Vector2d goalForce;
	goalForce = goal.minus(pose->origin);
	goalForce = goalForce.norm().times(SEEK_FORCE); //normalize the force, then scale by seek weight
	goalForce = goalForce.rotate(-pose->theta); //rotate the force to be relative to the robot

	int toSkip = scan->scans.size()/SCANS_TO_PROCESS;
	if(!toSkip) toSkip = 1;

	if(distance > .5){ //only avoid obstacles if we're far from our goal.. not ideal, but easier than changing weights.
		//for each scan in scan
		for (vector<ScanNode>::iterator it = scan->scans.begin(); it < scan->scans.end(); it+=toSkip){
			double dist =it->range;

			if (it->angle <= toRad(120) && it->angle >= toRad(-120) ){
				Vector2d position(0,0); //our position
				//generate a Vector2d pointing towards the object (from us as origin)
				Vector2d obs = it->getObstacle();

				//produce a Vector2d pointing away inversely proportional to distance
				Vector2d force = avoid(position, obs);

				//scale vector so it isn't a huge sum
				force = force.divide(SCANS_TO_PROCESS);

				//pay more attention to the vectors near our current heading
				force = force.times(cos(it->angle)*2);
				Velocity = Velocity.plus(force);
			}
		}
	} else{
	}

	//add a constant attractive force towards our goal
	Velocity = Velocity.plus(goalForce);
	if (Velocity.len() > maxSpeed){
		Velocity = Velocity.norm().times(maxSpeed);
	}
	//finally, divide by two to average with the previous run.
}

void SafeGoTo::applyVelocity(Position2dProxy* pp ){
	double turnRate = (Velocity.getAngle() ) *.9;
	if (turnRate > this->maxTwist) turnRate=this->maxTwist;
	if (turnRate < -this->maxTwist) turnRate=-this->maxTwist;

	double speed = Velocity.len();
	if (speed>maxSpeed) speed = maxSpeed;

	if(Velocity.getAngle() > PI/4 ||  Velocity.getAngle() < -PI/4){
		pp->SetSpeed(0,turnRate);
	}else if(Velocity.getAngle() > .1 ||  Velocity.getAngle() < -.1){
		//angle is not small. turn and drive slowly
		pp->SetSpeed(speed * .9,turnRate*.7);
	}else{
		pp->SetSpeed(speed , turnRate*.5);
	}
}

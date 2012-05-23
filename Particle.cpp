/*
 * particle for localization 
 *
 *	Author: Andrew LeCain
 */
#include "Vector2d.h"
#include "Particle.h"
#include <vector>
#include <iostream>
#include <cmath>
#include <time.h>
#include <stdlib.h>
#include <GL/glut.h>
#include "util.h"

#define SCANS_TO_SCORE 70

double Particle::thetaThetaCov=0.08;
double Particle::thetaLinearCov=0.4;
double Particle::linearLinearCov=0.6;
double Particle::linearThetaCov=0.05;

double gaussian(){
	double sum=0;
	//compute an approximate gaussian
	for(int j=0;j<12;j++){
		sum += ((double) rand())/RAND_MAX;
	}

	return (sum-6.0);
}

Particle::Particle(double x, double y, double theta){
	this->origin.x=x;
	this->origin.y=y;
	this->theta=theta;
	this->scoreVal=0;
	this->creation= clock();//set the creation time
}
Particle Particle::clone(){
	Particle clone(*this);
	clone.origin.x = this->origin.x + gaussian() * linearLinearCov *.05;
	clone.origin.y = this->origin.y +gaussian() * linearLinearCov *.05;
	clone.theta = this->theta + gaussian() * thetaThetaCov*.05;
	clone.scoreVal = 0;
	clone.creation = clock();

	return clone;
}
vector<Particle> Particle::update(double dlinear,double dtheta, int toSpawn){

	this->scoreVal=0;
	vector<Particle> ret;


	for(int i=0;i<toSpawn;i++){
		double newTheta = this->theta;
		Vector2d newOrigin= this->origin;
		newTheta += dtheta + dtheta * thetaThetaCov * gaussian() + dlinear * thetaLinearCov * gaussian();
		newOrigin += Vector2d(dlinear + dtheta * linearThetaCov * gaussian() + dlinear * linearLinearCov * gaussian(),0).rotate(newTheta);
		ret.push_back(Particle(newOrigin.x, newOrigin.y, wrap(newTheta)));
	}

	this->theta += dtheta + dtheta* thetaThetaCov* gaussian()+dlinear*thetaLinearCov*gaussian();
	this->theta=wrap(this->theta);
	this->origin += Vector2d(dlinear + dtheta * linearThetaCov * gaussian() + dlinear * linearLinearCov * gaussian(),0).rotate(this->theta);
	return ret;
}

double Particle::score(Map *map, Scan *scan){
	if (scoreVal != 0){
		return scoreVal;
	}
	double cost=0;
	int toSkip = scan->scans.size()/SCANS_TO_SCORE;
	if(!toSkip) toSkip = 1;

	for (vector<ScanNode>::iterator it= scan->scans.begin(); it < scan->scans.end(); it+= toSkip){
		double mapRange= 0xFFFF;
		for(double i =-it->width/2; i < it->width/2; i+= toRad(1)){
			double ray = map->raytrace(it->origin.x + this->origin.x,
					it->origin.y+this->origin.y,
					wrap(i+it->angle+this->theta),
					MAX_RANGE);
			mapRange=min(mapRange,ray);
		}
		if (mapRange > MAX_RANGE){
			mapRange=MAX_RANGE;
		}
		cost += it->Score(mapRange);
	}
	scoreVal=cost;
	time_t temp;
	temp = clock();

	double age = (temp-creation)/CLOCKS_PER_SEC; //now minus then
	if ( age > 2 ) age = 2;
	scoreVal -=age;

	return scoreVal;
}

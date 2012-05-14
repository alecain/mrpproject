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
#include <stdlib.h>
#include <GL/glut.h>
#include "util.h"

double Particle::thetaThetaCov=0.01;
double Particle::thetaLinearCov=0.01;
double Particle::linearLinearCov=0.01;
double Particle::linearThetaCov=0.01;

Particle::Particle(double x, double y, double theta){
	this->origin.x=x;
	this->origin.y=y;
	this->theta=theta;
	this->scoreVal=0;
}
double gaussian(){
	double sum=0;
	//compute an approximate gaussian
	for(int j=0;j<12;j++){
		sum += ((double) rand())/RAND_MAX;
	}

	return sum-6.0;
}

vector<Particle> Particle::update(Vector2d odometry,double dtheta, int toSpawn){

	this->scoreVal=0;
	vector<Particle> ret;
	double dlinear=odometry.len(); 
	
	if(fabs(wrap(odometry.getAngle()-this->theta)) > PI/2){
		dlinear *= -1;
	}

	for(int i=0;i<toSpawn;i++){
		double newTheta = this->theta;
		Vector2d newOrigin= this->origin;
		newTheta += dtheta + dtheta * thetaThetaCov * gaussian() + dlinear * thetaLinearCov * gaussian();
		newOrigin += Vector2d(dlinear + dtheta * linearThetaCov * gaussian() + dlinear * linearLinearCov * gaussian(),0).rotate(newTheta);
		ret.push_back(Particle(newOrigin.x, newOrigin.y, wrap(newTheta)));

	}



	this->theta += dtheta + dtheta* thetaThetaCov*gaussian()+dlinear*thetaLinearCov*gaussian();
	this->theta=wrap(this->theta);	
	//this->origin += odometry.rotate(this->theta);
	this->origin += Vector2d(dlinear + dtheta * linearThetaCov * gaussian() + dlinear * linearLinearCov * gaussian(),0).rotate(this->theta);
	return ret;
}

double Particle::score(Map *map, Scan *scan){
	if (scoreVal != 0){
		return scoreVal;
	}
	double cost= 0;
	for (vector<ScanNode>::iterator it= scan->scans.begin(); it < scan->scans.end(); it++){
		double mapRange = map->raytrace(it->origin.x + this->origin.x, it->origin.y+this->origin.y, wrap(it->angle+this->theta));
		if (mapRange > 5.0){
			mapRange=5.0;
		}
		cost += abs(it->range - mapRange);
	}
	scoreVal=cost;
	return cost;
}

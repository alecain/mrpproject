/*
 * particle for localization 
 *
 *	Author: Andrew LeCain
 */
#include "Vector2d.h"
#include "Particle.h"
#include <vector>
#include <stdlib.h>

double Particle::thetaThetaCov=0.01;
double Particle::thetaLinearCov=0.01;
double Particle::linearLinearCov=0.01;
double Particle::linearThetaCov=0.01;

Particle::Particle(double x, double y, double theta){
	this->origin.x=x;
	this->origin.y=y;
	this->theta=theta;

}
double gaussian(){
	double sum=0;
	//compute an approximate gaussian
	for(int j=0;j<12;j++){
		sum += rand();
	}
	return sum-6.0;
}

vector<Particle> Particle::update(Vector2d odometry, int toSpawn){

	vector<Particle> ret;
	double dlinear= odometry.len(), dtheta=odometry.getAngle();

	for(int i=0;i<toSpawn;i++){
		double newTheta = this->theta;
		Vector2d newOrigin= this->origin;
		newTheta += dtheta + dtheta * thetaThetaCov * gaussian() + dlinear * thetaLinearCov * gaussian();
		newOrigin += Vector2d(dlinear + dtheta * linearThetaCov* gaussian() + dlinear * linearLinearCov * gaussian(),0).rotate(newTheta);
		ret.push_back(Particle(newOrigin.x, newOrigin.y, newTheta));

	}

	this->origin+= odometry;
	this->theta += odometry.getAngle();		
}


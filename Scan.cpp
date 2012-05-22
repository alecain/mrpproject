/*
 * Scan abstraction class
 *
 *
 *	Author: Andrew LeCain
 */


#include "util.h"
#include "Scan.h"
#include "Vector2d.h"
#include <vector>
#include <cmath>



const double pose[8][4] = {
	{  0.075, 0.130, toRad(90)},
	{  0.115, 0.115, toRad(50)},
	{  0.150, 0.080, toRad(30)},
	{  0.170, 0.025, toRad(10)},
	{  0.170, -0.025, toRad(-10)},
	{  0.150, -0.080, toRad(-30)},
	{  0.115, -0.115, toRad(-50)},
	{  0.075, -0.130, toRad(-90)}};


ScanNode::ScanNode(Vector2d origin, double angle, double range , double width){
	this->origin=origin;
	this->angle=angle;
	this->range=range;
	this->width=width;

}

//function to score a scan compared to an ideal range that we think we should get. lower is better
double ScanNode::Score(double ideal){
	double ret=0;
	if (range > MAX_RANGE){
		range = MAX_RANGE;
	}

	double delta = pow(range - ideal,2);

	return delta;
	if (range > MAX_RANGE * .9 && ideal < MAX_RANGE){ // could be a missed scan.
		ret = .4;
	}else if(range < ideal * .9){ //range < ideal.. maybe we hit some obstacle on the map. Still not good.
		ret = 2*delta;
	} else if ( range < ideal * 1.1){ //range ~ ideal. Good!
		ret=.1*delta;
	} else{ //scan is between ideal and max. This basically means we missed;
		ret = 5*delta;
	}

	return ret;
}

Scan::Scan(Vector2d origin, ScanType type){
	this->origin = origin;
	this->type=type;
}

void Scan::addScan(double angle, double len){
	double width;
	if (type== SONAR){
		width=toRad(20);
		if (len == 0){ //forgot this about sonars...
			len = MAX_RANGE;
		}
	}else{
		width=toRad(0);
	}
	ScanNode scan(this->origin, angle,len, width);
	this->scans.push_back(scan);	
}
void Scan::addScan(int index, double len){
	double width;
	if (type== SONAR){
		width=toRad(20);
	}else{
		width=toRad(2.844);
	}
	ScanNode scan(Vector2d(this->sensorX(index), this->sensorY(index)), this->indexToAngle(index), len,width);
	this->scans.push_back(scan);
}
int Scan::len(){
	return scans.size(); 
}
double Scan::indexToAngle(int index){
	if (this->type ==SONAR)
		return (pose[index][2]);
	return (index-341)*toRad(.351);
}
int Scan::angleToIndex(double angle){
	if (this->type ==SONAR){
		for(int i =0; i< 8 ;i++){
			if (pose[i][3]==angle){
				return i;
			}	
		}
	}
	return (angle/toRad(.351))+341;
}

double Scan::sensorX(int index){
	if (this->type ==SONAR)
		return (pose[index][0]);
	return 0;
}
double Scan::sensorY(int index){
	if (this->type ==SONAR)
		return (pose[index][1]);
	return 0; 
}

Vector2d Scan::getObstacle(int index){
	Vector2d scan = this->scans[index].origin;
	scan+= Vector2d(this->scans[index].range,0).rotateAbs(this->scans[index].angle);
	return scan;
}

//gives a vector to the obstacle relative to the robot
Vector2d ScanNode::getObstacle(){
	Vector2d scan = this->origin;
	scan+= Vector2d(this->range,0).rotateAbs(this->angle);
	return scan;
}

double Scan::getRange(int index){
	return scans[index].range;
}


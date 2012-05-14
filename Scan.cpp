/*
 * Scan abstraction class
 *
 *
 *	Author: Andrew LeCain
 */


#include "Scan.h"
#include "Vector2d.h"
#include <vector>

#define PI 3.14159
#define toRad(deg) (deg*PI/180.0)


const double pose[8][4] = {
	{  0.075, 0.130, toRad(90)},
	{  0.115, 0.115, toRad(50)},
	{  0.150, 0.080, toRad(30)},
	{  0.170, 0.025, toRad(10)},
	{  0.170, -0.025, toRad(-10)},
	{  0.150, -0.080, toRad(-30)},
	{  0.115, -0.115, toRad(-50)},
	{  0.075, -0.130, toRad(-90)}};


ScanNode::ScanNode(Vector2d origin, double angle, double range){
	this->origin=origin;
	this->angle=angle;
	this->range=range;
}


Scan::Scan(Vector2d origin, ScanType type){
	this->origin = origin;
	this->type=type;
}

void Scan::addScan(double angle, double len){
	ScanNode scan(this->origin, angle,len);
	this->scans.push_back(scan);	
}
void Scan::addScan(int index, double len){
	ScanNode scan(Vector2d(this->sensorX(index), this->sensorY(index)), this->indexToAngle(index), len);
	this->scans.push_back(scan);	
}
int Scan::len(){
	return scans.size(); 
}
double Scan::indexToAngle(int index){
	if (this->type ==SONAR)
		return (pose[index][2]);
	return toRad((index-341)*2.844);
}
int Scan::angleToIndex(double angle){
	if (this->type ==SONAR){
		for(int i =0; i< 8 ;i++){
			if (pose[i][3]==angle){
				return i;
			}	
		}
	}
	return (angle/0.05)+341;
}

double Scan::sensorX(int index){
	if (this->type ==SONAR)
		return (pose[index][0]);
	return this->origin.x;
}
double Scan::sensorY(int index){
	if (this->type ==SONAR)
		return (pose[index][1] );
	return this->origin.y;
}

Vector2d Scan::getObstacle(int index){
	Vector2d scan = this->scans[index].origin;
	scan+= Vector2d(this->scans[index].range,0).rotateAbs(this->scans[index].angle);	
}

double Scan::getRange(int index){
	return scans[index].range;
}


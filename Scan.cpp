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


Scan::Scan(Vector2d origin, ScanType type){
	this->origin = origin;
}

void Scan::addScan(double angle, double len){
	int index=this->angleToIndex(angle);
	this->addScan(index, len);
}
void Scan::addScan(int index, double len){
	Vector2d scan(len,0);
	scan.rotate(pose[index][2]);
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
		return (pose[index][0] + this->origin.x);
	return this->origin.x;
}
double Scan::sensorY(int index){
	if (this->type ==SONAR)
		return (pose[index][1] + this->origin.y);
	return this->origin.y;
}

Vector2d Scan::getObstacle(int index){
	Vector2d scan = this->scans[index];
	scan+= this->origin;	
}

double Scan::getRange(int index){
	return scans[index].len();
}


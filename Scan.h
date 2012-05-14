/*
 * Scan abstraction class
 *
 *
 *	Author: Andrew LeCain
 */
#ifndef _SCAN_H
#define _SCAN_H

#include "Vector2d.h"
#include <vector>

using namespace std;

typedef enum{
	SONAR,
	LASER

}ScanType;

class ScanNode{
	public:
		ScanNode(Vector2d origin, double angle, double range);
		Vector2d origin;
		double angle;
		double range;
};


class Scan{
	public:
		Scan(Vector2d origin, ScanType type = SONAR);
		void addScan(int index, double len);
		void addScan(double angle, double len);
		double indexToAngle(int index);
		int angleToIndex(double angle);
		int len();
		double sensorX(int index);
		double sensorY(int index);
		Vector2d getObstacle(int index);
		double getRange(int index);

		ScanType type;
		Vector2d origin;
		vector<ScanNode> scans;
};

#endif

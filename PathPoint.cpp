#include "PathPoint.h"

PathPoint::PathPoint(double x, double y) : _x(x), _y(y) {

}

void PathPoint::connect(PathPoint *other) {
	_connections.push_back(other);
}

double PathPoint::getX() {
	return _x;
}

double PathPoint::getY() {
	return _y;
}


#include <float.h>
#include "PathPoint.h"

using namespace std;

PathPoint::PathPoint(double x, double y) : _x(x), _y(y) {
	_cost = DBL_MAX;
}

void PathPoint::connect(PathPoint *other, double cost) {
	_connections.push_back(pair<PathPoint *, double>(other, cost));
}

double PathPoint::getX() {
	return _x;
}

double PathPoint::getY() {
	return _y;
}

void PathPoint::setCost(double cost) {
	_cost = cost;
}

double PathPoint::getCost() {
	return _cost;
}

vector< pair<PathPoint *, double> >::const_iterator PathPoint::connectionsBegin() {
	return _connections.begin();
}

vector< pair<PathPoint *, double> >::const_iterator PathPoint::connectionsEnd() {
	return _connections.end();
}


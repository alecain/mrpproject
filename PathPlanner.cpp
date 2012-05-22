#include <stdlib.h>
#include <map>
#include <algorithm>
#include <math.h>
#include <float.h>
#include <stdio.h>
#include "PathPlanner.h"

#define NUM_CONNECTIONS 5

using namespace std;

PathPlanner::PathPlanner(Map *map) : _map(map) {

}

void PathPlanner::generatePaths(int count) {
	double x;
	double y;

	// Generate 'count' random points and make sure they are in the configuration space
	for (int i = 0; i < count; i++) {
		while (x = _map->getPixelWidth() * rand() / RAND_MAX,
		       y = _map->getPixelHeight() * rand() / RAND_MAX,
		       _map->getPixel(x, y) != 255);

		_points.push_back(new PathPoint(x, y));
	}

	clock_t start = clock();
	// Connect each of the points to their closests points to create the graph
	for (int i = 0; i < _points.size(); i++) {
		map<double, PathPoint *> closest;
		vector<double> dists;
		for (int j = 0; j < _points.size(); j++) {
			if (i == j) continue;

			bool linked = false;
			for (vector< pair<PathPoint *, double> >::const_iterator neighbor = _points[i]->connectionsBegin(); neighbor != _points[i]->connectionsEnd(); neighbor++) {
				if (neighbor->first == _points[j]) {
					linked = true;
					break;
				}
			}
			if (linked) continue;

			double dist = sqrt(pow(_points[i]->getX() - _points[j]->getX(), 2) + pow(_points[i]->getY() - _points[j]->getY(), 2));

			// Check to see if this path collides with the map
			if (_map->raytracePixel(_points[i]->getX(),
			              _points[i]->getY(),
			              atan2(_points[j]->getY() - _points[i]->getY(), _points[j]->getX() - _points[i]->getX()),
			              dist) < dist)
				continue;

			if (closest.size() < NUM_CONNECTIONS) {
				// If there aren't enough connections, add the point
				closest[dist] = _points[j];
				dists.push_back(dist);
				sort(dists.begin(), dists.end(), std::greater<double>());
			} else {
				// Otherwise, figure out if this new point is closer than any of the previous points
				for(vector<double>::iterator point = dists.begin(); point != dists.end(); point++) {
					if (*point > dist) {
						// If there is a closer point, replace the farthest point with this new point
						closest.erase(*point);
						dists.erase(point);

						closest[dist] = _points[j];
						dists.push_back(dist);
						sort(dists.begin(), dists.end(), std::greater<double>());
						break;
					}
				}
			}
		}

		for (map<double, PathPoint *>::iterator point = closest.begin(); point != closest.end(); point++) {
			_connections.push_back(pair<PathPoint *, PathPoint *>(_points[i], point->second));
			_points[i]->connect(point->second, point->first);
			point->second->connect(_points[i], point->first);
		}
	}

	printf("Map took %ld ms to generate\n", (long)(clock() - start)/(CLOCKS_PER_SEC/1000));
}

void PathPlanner::findAndConnect(PathPoint *point) {
	map<double, PathPoint *> closest;
	vector<double> dists;
	for (int i = 0; i < _points.size(); i++) {
		double dist = sqrt(pow(point->getX() - _points[i]->getX(), 2) + pow(point->getY() - _points[i]->getY(), 2));

		// Check to see if this path collides with the map
		if (_map->raytracePixel(_points[i]->getX(),
					  _points[i]->getY(),
					  atan2(_points[i]->getY() - point->getY(), _points[i]->getX() - point->getX()),
					  dist) < dist)
			continue;

		if (closest.size() < NUM_CONNECTIONS) {
			// If there aren't enough connections, add the point
			closest[dist] = _points[i];
			dists.push_back(dist);
			sort(dists.begin(), dists.end(), std::greater<double>());
		} else {
			// Otherwise, figure out if this new point is closer than any of the previous points
			for(vector<double>::iterator point = dists.begin(); point != dists.end(); point++) {
				if (*point > dist) {
					// If there is a closer point, replace the farthest point with this new point
					closest.erase(*point);
					dists.erase(point);

					closest[dist] = _points[i];
					dists.push_back(dist);
					sort(dists.begin(), dists.end(), std::greater<double>());
					break;
				}
			}
		}
	}

	for (map<double, PathPoint *>::iterator other = closest.begin(); other != closest.end(); other++) {
		_connections.push_back(pair<PathPoint *, PathPoint *>(point, other->second));
		point->connect(other->second, other->first);
		other->second->connect(point, other->first);
	}
}

void PathPlanner::setDestination(double destX, double destY) {
	PathPoint *dest = new PathPoint(destX, destY);
	_points.push_back(dest);

	clock_t start = clock();

	findAndConnect(dest);
	printf("findAndConnect took %ld ms to generate\n", (long)(clock() - start)/(CLOCKS_PER_SEC/1000));

	for (vector<PathPoint *>::const_iterator point = getPointsBegin(); point != getPointsEnd(); point++)
		(*point)->setCost(DBL_MAX);

	start = clock();

	evaluatePathCost(0, dest);
	printf("evaluatePathCost took %ld ms to generate\n", (long)(clock() - start)/(CLOCKS_PER_SEC/1000));
}

void PathPlanner::evaluatePathCost(double cost, PathPoint *point) {
	point->setCost(cost);

	for (vector< pair<PathPoint *, double> >::const_iterator neighbor = point->connectionsBegin(); neighbor != point->connectionsEnd(); neighbor++) {
		double segcost = cost + neighbor->second;
		if (segcost < neighbor->first->getCost()) {
			neighbor->first->setCost(segcost);
			evaluatePathCost(segcost, neighbor->first);
		}
	}
}

// NOTE: If there is no path to the destination, this will lock up
list<PathPoint *> PathPlanner::findRoute(double origX, double origY) {
	list<PathPoint *> path;

	double cost = DBL_MAX;
	PathPoint *cur = NULL;

	// Find the closest waypoint to start at
	for (vector<PathPoint *>::const_iterator point = getPointsBegin(); point != getPointsEnd(); point++) {
		double newcost = sqrt(pow(origX - (*point)->getX(), 2) + pow(origY - (*point)->getY(), 2));
		if (newcost < cost) {
			cost = newcost;
			cur = *point;
		}
	}

	if (cur == NULL) {
		printf("Empty set of waypoints. You must call generatePaths() first\n");
		return path;
	}

	printf("Starting at (%f, %f)\n", cur->getX(), cur->getY());

	path.push_back(cur);

	// Find the shortest path to the destination
	while (true) {
		cost = DBL_MAX;
		PathPoint *next = NULL;

		// Find the closest neighbor to the current waypoint
		for (vector< pair<PathPoint *, double> >::const_iterator neighbor = cur->connectionsBegin(); neighbor != cur->connectionsEnd(); neighbor++) {
			if (neighbor->first->getCost() < cost) {
				cost = neighbor->first->getCost();
				next = neighbor->first;
			}
		}

		if (next == NULL) {
			printf("Node has no neighbors...he's so lonely\n");
			return path;
		}

		path.push_back(next);
		printf(" (%f, %f)\n", cur->getX(), cur->getY());
		cur = next;

		if (cost == 0) break;
	}

	printf("FOUND A PATH (%d steps)\n", path.size());
	return path;
}

vector<PathPoint *>::const_iterator PathPlanner::getPointsBegin() {
	return _points.begin();
}

vector<PathPoint *>::const_iterator PathPlanner::getPointsEnd() {
	return _points.end();
}

vector< pair<PathPoint *, PathPoint *> >::const_iterator PathPlanner::getConnectionsBegin() {
	return _connections.begin();
}

vector< pair<PathPoint *, PathPoint *> >::const_iterator PathPlanner::getConnectionsEnd() {
	return _connections.end();
}


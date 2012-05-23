#include <stdlib.h>
#include <map>
#include <algorithm>
#include <math.h>
#include <float.h>
#include <stdio.h>
#include "PathPlanner.h"

#define NUM_CONNECTIONS 3

using namespace std;

PathPlanner::PathPlanner(Map *map) : _map(map) {

}

void PathPlanner::generatePaths(int count, double destX, double destY) {
	double x;
	double y;
	PathPoint *dest = new PathPoint(destX, destY);

	// Generate 'count' random points and make sure they are in the configuration space
	for (int i = 0; i < count; i++) {
		while (x = _map->getPixelWidth() * rand() / RAND_MAX,
		       y = _map->getPixelHeight() * rand() / RAND_MAX,
		       _map->getPixel(x, y) < 100);

		_points.push_back(new PathPoint(x, y));
	}
	_points.push_back(dest);

	clock_t start = clock();
	// Connect each of the points to their closests points to create the graph
	for (int i = 0; i < _points.size(); i++) {
		map<double, PathPoint *> closest;
		vector<double> dists;
		for (int j = i + 1; j < _points.size(); j++) {
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
			_points[i]->connect(point->second);
			point->second->connect(_points[i]);
		}
	}

	evaluatePathCost(0, dest);

	printf("Map took %ld ms to generate\n", (long)(clock() - start)/(CLOCKS_PER_SEC/1000));
}

void PathPlanner::evaluatePathCost(double cost, PathPoint *point) {
	point->setCost(cost);

	for (vector<PathPoint *>::const_iterator neighbor = point->connectionsBegin(); neighbor != point->connectionsEnd(); neighbor++) {
		double segcost = cost + sqrt(pow(point->getX() - (*neighbor)->getX(), 2) + pow(point->getY() - (*neighbor)->getY(), 2));
		if (segcost < (*neighbor)->getCost()) {
			(*neighbor)->setCost(segcost);
			evaluatePathCost(segcost, *neighbor);
		}
	}
}

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

	if (cur->getCost() == DBL_MAX) {
		printf("No route to host^H^H^H^Hdestination\n");
		return path;
	}

	path.push_back(cur);

	// Find the shortest path to the destination
	while (true) {
		cost = DBL_MAX;
		PathPoint *next = NULL;

		// Find the closest neighbor to the current waypoint
		for (vector<PathPoint *>::const_iterator neighbor = cur->connectionsBegin(); neighbor != cur->connectionsEnd(); neighbor++) {
			if ((*neighbor)->getCost() < cost) {
				cost = (*neighbor)->getCost();
				next = *neighbor;
			}
		}

		if (next == NULL) {
			printf("Node has no neighbors...he's so lonely\n");
			return path;
		}

		path.push_back(next);
		cur = next;

		if (cost == 0) break;
	}

	printf("FOUND A PATH\n");
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


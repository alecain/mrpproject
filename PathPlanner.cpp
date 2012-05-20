#include <stdlib.h>
#include <map>
#include <algorithm>
#include <math.h>
#include <stdio.h>
#include "PathPlanner.h"

#define NUM_CONNECTIONS 4

using namespace std;

PathPlanner::PathPlanner(Map *map) : _map(map) {

}

void PathPlanner::generateWaypoints(int count) {
	double x;
	double y;

	// Generate 'count' random points and make sure they are in the configuration space
	for (int i = 0; i < count; i++) {
		while (x = _map->getPixelWidth() * rand() / RAND_MAX,
		       y = _map->getPixelHeight() * rand() / RAND_MAX,
		       _map->getPixel(x, y) != 255);

		_points.push_back(new PathPoint(x, y));
	}

	// Connect each of the points to their closests points to create the graph
	for (int i = 0; i < _points.size(); i++) {
		map<double, PathPoint *> closest;
		vector<double> dists;
		for (int j = 0; j < _points.size(); j++) {
			if (i == j) continue;

			double dist = pow(_points[i]->getX() - _points[j]->getX(), 2) + pow(_points[i]->getY() - _points[j]->getY(), 2);

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

		for(map<double, PathPoint *>::iterator point = closest.begin(); point != closest.end(); point++)
			_connections.push_back(pair<PathPoint *, PathPoint *>(_points[i], point->second));
	}
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


#ifndef _PATH_PLANNER_H
#define _PATH_PLANNER_H

#include <vector>
#include <list>
#include "PathPoint.h"
#include "map.h"

class PathPlanner {
	private:
		std::vector<PathPoint *> _points;
		Map *_map;
		std::vector< pair<PathPoint *, PathPoint *> > _connections;

		void evaluatePathCost(double cost, PathPoint *point);

	public:
		PathPlanner(Map *map);
		void generatePaths(int count);
		void findAndConnect(PathPoint *point);
		void setDestination(double destX, double destY);
		std::vector<PathPoint *>::const_iterator getPointsBegin();
		std::vector<PathPoint *>::const_iterator getPointsEnd();
		vector< pair<PathPoint *, PathPoint *> >::const_iterator getConnectionsBegin();
		vector< pair<PathPoint *, PathPoint *> >::const_iterator getConnectionsEnd();
		std::list<PathPoint *> findRoute(double origX, double origY);
};

#endif


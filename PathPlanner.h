#ifndef _PATH_PLANNER_H
#define _PATH_PLANNER_H

#include <vector>
#include "PathPoint.h"
#include "map.h"

class PathPlanner {
	private:
		std::vector<PathPoint *> _points;
		Map *_map;
		std::vector< pair<PathPoint *, PathPoint *> > _connections;

	public:
		PathPlanner(Map *map);
		void generateWaypoints(int count);
		std::vector<PathPoint *>::const_iterator getPointsBegin();
		std::vector<PathPoint *>::const_iterator getPointsEnd();
		vector< pair<PathPoint *, PathPoint *> >::const_iterator getConnectionsBegin();
		vector< pair<PathPoint *, PathPoint *> >::const_iterator getConnectionsEnd();
};

#endif


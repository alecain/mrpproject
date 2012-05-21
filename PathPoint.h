#ifndef _PATH_POINT_H
#define _PATH_POINT_H

#include <vector>

class PathPoint {
	private:
		double _x;
		double _y;
		double _cost;
		std::vector<PathPoint *> _connections;

	public:
		PathPoint(double x, double y);
		void connect(PathPoint *other);
		double getX();
		double getY();
		void setCost(double cost);
		double getCost();
		std::vector<PathPoint *>::const_iterator connectionsBegin();
		std::vector<PathPoint *>::const_iterator connectionsEnd();
};

#endif


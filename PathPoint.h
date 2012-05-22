#ifndef _PATH_POINT_H
#define _PATH_POINT_H

#include <vector>

class PathPoint {
	private:
		double _x;
		double _y;
		double _cost;
		std::vector< std::pair<PathPoint *, double> > _connections;

	public:
		PathPoint(double x, double y);
		void connect(PathPoint *other, double cost);
		double getX();
		double getY();
		void setCost(double cost);
		double getCost();
		std::vector< std::pair<PathPoint *, double> >::const_iterator connectionsBegin();
		std::vector< std::pair<PathPoint *, double> >::const_iterator connectionsEnd();
};

#endif


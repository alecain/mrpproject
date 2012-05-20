#ifndef _PATH_POINT_H
#define _PATH_POINT_H

#include <vector>

class PathPoint {
	private:
		double _x;
		double _y;
		std::vector<PathPoint *> _connections;

	public:
		PathPoint(double x, double y);
		void connect(PathPoint *other);
		double getX();
		double getY();
};

#endif



#include <ostream>
class Vector2d{
	public:
		double x;
		double y;

		Vector2d();
		Vector2d(double x, double y);
	
		Vector2d norm();
		Vector2d clip(double max);
		double len() const;
		double distance(Vector2d b);
		double getAngle() const;

		double  dot(Vector2d b);
		Vector2d proj(Vector2d b);

		Vector2d times(double scalar);
		Vector2d divide(double scalar);
		Vector2d plus(Vector2d b);
		Vector2d minus(Vector2d b);
		Vector2d rotate(double theta);
		Vector2d rotateAbs(double theta);

};

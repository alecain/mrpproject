
/*
 *PGM map class
 *
 *
 *	Author: Andrew LeCain
 */

#include <string>
using namespace std;

#define RESOLUTION 0.05 	//5 cm per pixel


class Map{

	public:
		Map(string filename, string copyto); //load an existing map
		Map(string filename, int x, int y);//create a new map of size x y
		Map(string filename, double x, double y);//create a new map of size x y
		void save(void);
		int getVal(int x, int y);
		double getVal(double x, double y);
		void setVal(int x, int y, double newVal);
		void setVal(double x, double y, double newVal);
		void conflate(int x, int y, double newVal);
		void conflate(double x, double y, double newVal);
		double raytrace(double x, double y, double angle);
		~Map();

	private:
		string filename;
		int x,y;
		int maxVal;
		int **map;
			
};

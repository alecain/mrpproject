
/*
 *PGM map class
 *
 *
 *	Author: Andrew LeCain
 */

#include <string>
using namespace std;


#ifndef _MAP_H
#define _MAP_H


class Map{

	public:
		Map(string filename, string copyto, double resolution); //load an existing map
		Map(string filename, int x, int y, double resolution);//create a new map of size x y
		Map(string filename, double x, double y, double resolution);//create a new map of size x y
		void save(void);
		unsigned char getPixel(int x, int y);
		double getVal(double x, double y);
		bool isOccupied(double x, double y);
		void setPixel(int x, int y, unsigned char newVal);
		void setVal(double x, double y, double newVal);
		void conflate(int x, int y, double newVal);
		void conflate(double x, double y, double newVal);
		double raytracePixel(double x, double y, double angle, double maxRange);
		double raytrace(double x, double y, double angle, double maxRange);
		int xToMap(double x);
		int yToMap(double y);
		double xToMeters(int x);
		double yToMeters(int y);
		double getPixelWidth();
		double getPixelHeight();
		double getPixelLeft();
		double getPixelTop();
		~Map();

		string filename;
	private:
		int x0,y0; //origin
		double resolution;
		int x,y;
		int maxVal;
		unsigned char **map;

};

#endif

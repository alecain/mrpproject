
/*
 *PGM map class
 *
 *
 *	Author: Andrew LeCain
 */

#include <string> 
using namespace std;

class Map{

	public:
		Map(string filename, string copyto); //load an existing map
		Map(string filename, int x, int y);//create a new map of size x y
		void save(void);

		~Map();

	private:
		string filename;
		int x,y;
		int maxVal;
		int **map;
			
};

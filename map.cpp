/*
 *PGM map class
 *
 *
 *	Author: Andrew LeCain
 */

#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <iostream>
#include <cmath>
#include "map.h"
#include "util.h"

using namespace std;

Map::Map(string filename, string copyto, double resolution){
	this->filename=copyto;
	this->resolution=resolution;

	FILE *f;

	f=fopen(filename.c_str(),"r");
	char buf[3];
	buf [2]=0; //null terminate string

	int read = fread(buf, sizeof(char), 2, f);

	if (read != 2 || strcmp(buf, "P5") != 0){
		std::cerr<< filename <<" is not a valid PGM file"<<std::endl;
		exit(-1);
	}

	fscanf(f,"%d", &this->x);
	fscanf(f,"%d", &this->y);
	fscanf(f,"%d", &this->maxVal);
	this->x0 = this->x/2;
	this->y0 = this->y/2;


	this->map =(unsigned char**) malloc(sizeof(int **) * this->x); // allocate enough space for head of collumns
	for(int x = 0; x< this->x; x++){
		this->map[x] =(unsigned char*) malloc(sizeof(int *) * this->y); // allocate enough space for the row
	}	

	unsigned char c;
	for (int y = this->y-1; y >=0;y--){
		for(int x = 0; x< this->x; x++){
			if(fread(&c,sizeof(char),1,f) != 1){
				std::cerr<< filename <<" is not a valid PGM file"<<std::endl;
				exit(-1);
			}
			map[x][y]=c;
		}
	}

	fclose(f);

	std::cout<<filename<<" loaded successfully!"<<std::endl;
}

Map::Map(string filename, double x, double y, double resolution){
	this->resolution=resolution;
	Map(filename, x*resolution, y *resolution, resolution);
}

Map::Map(string filename, int x, int y, double resolution){
	this->filename=filename;
	this->x = x;
	this->y = y;
	this->maxVal= 255;
	this->resolution=resolution;

	this->map =(unsigned char**) malloc(sizeof(int **) * this->x); // allocate enough space for head of collumns
	for(int x = 0; x< this->x; x++){
		this->map[x] =(unsigned char*) malloc(sizeof(int *) * this->y); // allocate enough space for the row
		memset(this->map[x],128,this->y);
	}
}

void Map::save(void){
	cout<<"saving map to "<<this->filename<<endl;
	FILE *f;

	f=fopen(filename.c_str(),"w");
	char buf[30];

	int len = sprintf(buf,"P5 %d %d %d ", this->x, this->y, this->maxVal);
	fwrite(buf,sizeof(char), len, f); //print all but the trailing null

	for(int y=0; y<this->y ;y++){
		for(int x=0;x<this->x;x++){
			buf[0]=this->map[x][y];
			fwrite(buf,sizeof(char),1,f);
		}
	}
}

Map::~Map(){
	this->save();

	for (int x=0;x<this->x;x++){
		free(this->map[x]);
	}
	free(this->map);

}

double Map::getVal(double x, double y){
	return this->getPixel(x/this->resolution+x0,y/this->resolution+y0)/255.0;
}
unsigned char Map::getPixel(int x, int y){
	if (x< 0 || x>=this->x || y< 0 || y>= this->y){
		cerr<<x<<","<<y<<" is out of bounds!"<<endl;
		return 0;
	}
	return map[x][y];
}
void Map::setVal(double x, double y, double newVal){
	setVal(x/this->resolution+x0, y/this->resolution+x0, newVal);
}
void Map::setPixel(int x, int y, unsigned char newVal){
	map[x][y]= newVal;
}
void Map::conflate(double x, double y, double newVal){
	conflate(x/this->resolution+x0, y/this->resolution+y0, newVal);
}
void Map::conflate(int x, int y, double newVal){
	map[x][y] = (map[x][y]/this->maxVal + newVal)*this->maxVal/2;
}

double Map::raytrace(double x0, double y0, double angle, double mrange){

	int x=(x0/this->resolution)+this->x0, y=y0/this->resolution+ this->y0;
	int x0i=x,y0i=y;
	int maxRange= pow(mrange/resolution,2);
	double slope = tan(angle);

	double sx,sy;
	if ( angle > -PI/2 && angle < PI/2) sx=1; else sx=-1;
	if  ( angle > 0) sy=1; else sy=-1;
	double dx, dy;

	dx=1;
	dy=abs(slope);
	double err= dx-dy;

	while(1){
		if (x < 0 || x >= this->x || y < 0 || y >= this->y){
			break;
		}
		//really dumb range checking.. just to save computation time
		if((x-x0i)*(x-x0i) + (y-y0i)*(y-y0i) > maxRange)
			break;
		if(this->getPixel(x,y) < 100)
				break;
		//this->setPixel(x,y,100);
		double e2= 2*err;
		
		if(e2 > -1* dy){
			err -= dy;
			x+=sx;
		}
		if(e2 < dx){
			err +=dx;
			y+=sy;	
		}
	}

	double range = sqrt( pow(x0-(x-this->x0)*this->resolution,2)+pow(y0-(y-this->y0)*this->resolution,2));
	return range;
}

/*
   int main(int argc, char *argv[]) {
   Map map(argv[1], argv[2]);
   Map blank("blank.pgm", 100, 100);
   cout<<"1 "<<map.raytrace(0,0, toRad(-45))<<endl;
   cout<<"2 "<<map.raytrace(0,0, toRad(135))<<endl;
   cout<<"3 "<<map.raytrace(0,0, toRad(45))<<endl;
   cout<<"4 "<<map.raytrace(0,0, toRad(-135))<<endl;
   }*/


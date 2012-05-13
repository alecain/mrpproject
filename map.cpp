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

#define PI 3.14159
#define toRad(deg) (deg*PI/180.0)
using namespace std;

Map::Map(string filename, string copyto){
	this->filename=copyto;
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

	this->map =(int **) malloc(sizeof(int **) * this->x); // allocate enough space for head of collumns
	for(int x = 0; x< this->x; x++){
		this->map[x] =(int *) malloc(sizeof(int *) * this->y); // allocate enough space for the row
	}	

	for (int y = 0; y < this->y;y++){
		for(int x = 0; x< this->x; x++){
			if(fread(buf,sizeof(char),1,f) != 1){
				std::cerr<< filename <<" is not a valid PGM file"<<std::endl;
				exit(-1);
			}
			map[x][y]=buf[0];
		}
	}

	fclose(f);

	std::cout<<filename<<" loaded successfully!"<<std::endl;
}

Map::Map(string filename, double x, double y){
	Map(filename, x*RESOLUTION, y *RESOLUTION);
}

Map::Map(string filename, int x, int y){
	this->filename=filename;
	this->x = x;
	this->y = y;
	this->maxVal= 255;
		
	this->map =(int **) malloc(sizeof(int **) * this->x); // allocate enough space for head of collumns
	for(int x = 0; x< this->x; x++){
		this->map[x] =(int *) malloc(sizeof(int *) * this->y); // allocate enough space for the row
		memset(this->map[x],128,this->y*4);
	}
}

void Map::save(void){
	cout<<"saving map to "<<this->filename<<endl;
	FILE *f;

	f=fopen(filename.c_str(),"w");
	char buf[30];

	int len = sprintf(buf,"P5 %d %d %d ", this->x, this->y, this->maxVal);
	fwrite(buf,sizeof(char), len, f); //print all but the trailing null

	for(int y=0;y<this->y;y++){
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
	return this->getVal(x*RESOLUTION, y*RESOLUTION)/maxVal;
}
int Map::getVal(int x, int y){
	return map[x][y];
}
void Map::setVal(double x, double y, double newVal){
	setVal(x*RESOLUTION, y*RESOLUTION, newVal);
}
void Map::setVal(int x, int y, double newVal){
	map[x][y]= newVal*maxVal;
}
void Map::conflate(double x, double y, double newVal){
	conflate(x*RESOLUTION, y*RESOLUTION, newVal);
}
void Map::conflate(int x, int y, double newVal){
	map[x][y] = (map[x][y]/this->maxVal + newVal)*this->maxVal/2;
}

double Map::raytrace(double x0, double y0, double angle){

	int x=(x0)* RESOLUTION + this->x, y=y0*RESOLUTION+ this->y;
	double slope = tan(angle);
	
	double sx,sy;
	if ( angle > toRad(-90) && angle < toRad(90)) sx=1; else sx=-1;
	if  ( angle > 0 && angle < toRad(180)) sy=1; else sy=-1;
	double dx, dy;
	
	dx=1;
	dy=abs(slope);
	double err= dx-dy;

	while(1){
		if (x < 0 || x >= this->x || y < 0 || y >= this->y){
			break;
		}
		if(this->getVal(x,y) < 10)
				break;
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

	return sqrt(pow(x-x0,2)+pow(y-y0,2));
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


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
#include "map.h"


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

Map::Map(string filename, int x, int y){
	this->filename=filename;
	this->x = x;
	this->y = y;

}

void Map::save(void){
	cout<<"saving map to "<<this->filename<<endl;
	FILE *f;

	f=fopen(filename.c_str(),"w");
	char buf[30];

	int len = sprintf(buf,"P5 %d %d %d ", this->x, this->y, this->maxVal);
	fwrite(buf,sizeof(char), len-1, f); //print all but the trailing null

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


int main(int argc, char *argv[]) {
	Map map(argv[1], argv[2]);

}


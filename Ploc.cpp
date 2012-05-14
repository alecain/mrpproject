
/*
 * particle based localization node 
 *
 *	Author: Andrew LeCain
 */

#include <vector>
#include <stdlib.h>
#include <iostream>

#include "util.h"
#include "Particle.h"
#include "map.h"
#include "Ploc.h"



Ploc::Ploc(int minParticles, int maxParticles, Map* map){
	this->minParticles=minParticles;
	this->maxParticles=maxParticles;
	this->map=map;
	replenishParticles();
}
void Ploc::updateParticles(double dlinear, double dtheta){
	bestScore=0xFFFF;
	for(vector<Particle>::iterator it=particles.begin();it<particles.end();it++){
		vector<Particle> ret = it->update(dlinear,dtheta,0);
		int x = it->origin.x;	
		int y = it->origin.y;	
		if(x<-XOFFSET|| x>XOFFSET || y<-YOFFSET || y > YOFFSET)
			particles.erase(it);
	}
}
void Ploc::scoreParticles(Scan *scans){
	double score;

	for(vector<Particle>::iterator it=particles.begin();it!=particles.end();it++){
		score = it->score(map,scans);	
		avgscore += score;
		if(score<bestScore){
			bestScore=score;
		}
	}
	avgscore /=particles.size();
}
void Ploc::replenishParticles(void){
	//Replenish with random valid particles
	while(particles.size()< minParticles){
		double x,y;
		do{
			x= ((double)rand())/RAND_MAX*WIN_X_METERS-XOFFSET;
			y= ((double)rand())/RAND_MAX*WIN_Y_METERS-YOFFSET;

		}while(map->getVal(x,y) < .5 );	
		Particle part(x,y,wrap(((double)rand())/RAND_MAX*2*PI));
		particles.push_back(part);
	}
}
void Ploc::pruneParticles(){
	double score;
	vector<Particle> newList;
	for(vector<Particle>::iterator it=particles.begin();it<particles.end();it++){
		score = it->scoreVal;
		if(score < (avgscore+bestScore)/2 ){	
			for(int i=0;i<3;i++){
				if(particles.size()+newList.size()<maxParticles)
					newList.push_back(Particle(*it));	
			}
		}else{

			particles.erase(it);
		}
	}

	for(int i =0;i<newList.size();i++){
		particles.push_back(newList[i]);
	}

}
Pose Ploc::getPose(void){
	Pose p;
	double counter=0;
	double score;
	p.x=0;
	p.y=0;
	p.theta=0;

	//perform a weighted average of the particles based on score
	for(vector<Particle>::iterator it=particles.begin();it<particles.end();it++){
		score = it->scoreVal;
		if(score < avgscore){
			p.x+=it->origin.x*avgscore/score;
			p.y+=it->origin.y*avgscore/score;
			p.theta+=it->theta*avgscore/score;
			counter+=avgscore/score;
		}
	}	
	p.x /= counter;
	p.y /= counter;
	p.theta/= counter;

	std::cout<<particles.size()<<" particles. x:"<<p.x<<" y:"<<p.y<<" theta:"<<p.theta <<std::endl;
	return p;
}


/*
 * particle based localization node
 *
 *    Author: Andrew LeCain
 */


#include <stdlib.h>
#include <iostream>
#include <list>
#include <vector>
#include <cmath>

#include "util.h"
#include "Particle.h"
#include "map.h"
#include "Ploc.h"



bool compare(Particle a, Particle b){
	return (a.scoreVal < b.scoreVal );
}


Ploc::Ploc(int minParticles, int maxParticles, Map* map){
	this->minParticles=minParticles;
	this->maxParticles=maxParticles;
	this->map=map;
	replenishParticles();
}
void Ploc::updateParticles(double dlinear, double dtheta){
	bestScore=0xFFFF;

	for(list<Particle>::iterator it=particles.begin();it!=particles.end();){
		vector<Particle> ret= it->update(dlinear,dtheta,0);
		double x = it->origin.x;
		double y = it->origin.y;
		if(x<-XOFFSET|| x>XOFFSET || y<-YOFFSET || y > YOFFSET){
			particles.erase(it++);
		}else{
			it++;
		}
	}
}
void Ploc::scoreParticles(Scan *scans){
	double score;

	for(list<Particle>::iterator it=particles.begin();it!=particles.end();it++){
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
	while(particles.size()< maxParticles){
		double x,y;
		do{
			x= ((double)rand())/RAND_MAX*WIN_X_METERS-XOFFSET;
			y= ((double)rand())/RAND_MAX*WIN_Y_METERS-YOFFSET;

		}while(map->isOccupied(x,y));
		Particle part(x,y,wrap(((double)rand())/RAND_MAX*2*PI));
		particles.push_back(part);
	}
}
void Ploc::pruneParticles(){
	double score;
	int count=0;
	int cloneCount=0;
	list<Particle> newList;

	//first sort from best to worst
	particles.sort(compare);
	//now prune the list
	for(list<Particle>::iterator it=particles.begin();it!=particles.end();){
		if(count++ < this->minParticles){
			if(newList.size()< minParticles*2){
				newList.push_back(it->clone());
			}
			++it;
		}else if(count < this->minParticles*3){
			//don't duplicate these.. just let them be
		}
		else{
			particles.erase(it++);
		}
	}
	//insert the cloned particles in order
	particles.merge(newList,compare);
}
Pose Ploc::getPose(int toAverage){
	Pose p;
	double counter=0;
	double score;
	p.origin.x=0;
	p.origin.y=0;
	p.sigx=0;
	p.sigy=0;
	p.theta=0;
	

	//perform a weighted average of the particles based on score
	particles.sort(compare);

	for(list<Particle>::iterator it=particles.begin();it!=particles.end();it++){
		if(counter < toAverage){
			p.origin.x+=it->origin.x;
			p.origin.y+=it->origin.y;
			p.theta+=it->theta;
		}else{
			break;
		}
		counter++;
	}
	
	counter=0;
	p.origin.x /= toAverage;
	p.origin.y /= toAverage;
	p.theta/= toAverage;
	p.theta=wrap(p.theta);

	for(list<Particle>::iterator it=particles.begin();it!=particles.end();it++){
		if(counter++ < toAverage ){
			p.sigx+=pow(it->origin.x-p.origin.x,2);
			p.sigy+=pow(it->origin.y-p.origin.y,2);
			p.sigtheta+=pow(it->theta-p.theta,2);
		}else{
			break;
		}
	}

	p.sigx=sqrt(p.sigx/(toAverage-1));
	p.sigy=sqrt(p.sigy/(toAverage-1));
	p.sigtheta=sqrt(p.sigtheta/(toAverage-1));


	return p;
}

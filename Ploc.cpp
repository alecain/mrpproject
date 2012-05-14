
/*
 * particle based localization node
 *
 *    Author: Andrew LeCain
 */


#include <stdlib.h>
#include <iostream>
#include <list>
#include <vector>

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

		}while(map->getVal(x,y) < .5 );
		Particle part(x,y,wrap(((double)rand())/RAND_MAX*2*PI));
		particles.push_back(part);
	}
}
void Ploc::pruneParticles(){
	double score;
	int count=0;
	int cloneCount=0;
	list<Particle> newList;

	cout<<"before: "<< particles.size() ;
	//first sort from best to worst
	particles.sort(compare);
	//now prune the list
	for(list<Particle>::iterator it=particles.begin();it!=particles.end();){
		if(count++ < this->minParticles){
			if(newList.size()< minParticles){
				newList.push_back(*it);
				cout<<".";
			}
			++it;
		}else if(count < this->minParticles*2){
			//don't duplicate these.. just let them be
		}
		else{
			particles.erase(it++);
		}
	}
	//insert the cloned particles in order
	cout<<"pruned: "<< particles.size() ;
	cout<<"after: "<< newList.size()<<endl;
	particles.merge(newList,compare);
}
Pose Ploc::getPose(void){
	Pose p;
	double counter=0;
	double score;
	p.x=0;
	p.y=0;
	p.theta=0;

	//perform a weighted average of the particles based on score
	for(list<Particle>::iterator it=particles.begin();it!=particles.end();it++){
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

	return p;
}

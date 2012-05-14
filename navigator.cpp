// Example of pixel-plotting together with Player/Stage robot control
// zjb 4/10, based on code by John Hawley and original uncredited
//   Player/Stage example code.
// Not guaranteed to be decent code style but seems to work OK.

#include <GL/glut.h>
#include <libplayerc++/playerc++.h>
#include <pthread.h>
#include <vector>

#include "util.h"
#include "Vector2d.h"
#include "Scan.h"
#include "map.h"
#include "Particle.h"

using namespace PlayerCc;
using namespace std;

/**
 ****************************************************************************
 *			Constants
 ****************************************************************************
 */


#define MAX_DIST 4

#define SCALE 2


#define UNEXPLORED 1

#define X_RES	0.066	//resolution of 5cm per pixel yielding 30m x
#define Y_RES	0.066	//resolution of 5cm per pixel yielding 30m y

#define WIN_X 2000/SCALE
#define WIN_Y 700/SCALE

#define WIN_X_METERS (WIN_X *SCALE * X_RES)
#define WIN_Y_METERS (WIN_Y *SCALE * Y_RES)

#define XOFFSET WIN_X_METERS/2 //set origin in the middle
#define YOFFSET WIN_Y_METERS/2

#define windowX(x) ((x+XOFFSET)/X_RES/SCALE)
#define windowY(y) ((y+YOFFSET)/Y_RES/SCALE)

/**
 ****************************************************************************
 *			Globals
 ****************************************************************************
 */


const double pose[8][4] = {
	{  0.075, 0.130, toRad(90)},
	{  0.115, 0.115, toRad(50)},
	{  0.150, 0.080, toRad(30)},
	{  0.170, 0.025, toRad(10)},
	{  0.170, -0.025, toRad(-10)},
	{  0.150, -0.080, toRad(-30)},
	{  0.115, -0.115, toRad(-50)},
	{  0.075, -0.130, toRad(-90)}};
static PlayerClient *pRobot;
static Position2dProxy *pPosition;
static SonarProxy *pSonar;
pthread_mutex_t display_mut;
pthread_mutex_t particles_mut;
vector<Particle> particles;


void Particle::draw(){
	//draw a vector of lenth 1m starting v.x, v.y at angle theta
	//	  v1
	//	/ | \
	//    v2  |  v3
	//	  |
	//        v0

	double x0=this->origin.x;
	double y0=this->origin.y;
	double x1=(x0+cos(this->theta)*2);
	double y1=(y0+sin(this->theta)*2);
	double x2=(x1-cos(this->theta+toRad(30))*.6);
	double y2=(y1-sin(this->theta+toRad(30))*.6);
	double x3=(x1-cos(this->theta-toRad(30))*.6);
	double y3=(y1-sin(this->theta-toRad(30))*.6);

	glColor3ub(255,0,0);	
	glLineWidth(1);
	glBegin(GL_LINE_STRIP);	
	glVertex2d(windowX(x0),windowY(y0));	
	glVertex2d(windowX(x1),windowY(y1));	
	glVertex2d(windowX(x2),windowY(y2));	
	glVertex2d(windowX(x1),windowY(y1));	
	glVertex2d(windowX(x3),windowY(y3));	
	glEnd();

}

/**
 ****************************************************************************
 *			Definitions
 ****************************************************************************
 */

void conflatePixel( double x, double y, double value);
/**
 ****************************************************************************
 *			Sensor abstraction
 ****************************************************************************
 */



//abstraction of the ranger and sonar
double getRange(int index){
	return (*pSonar)[index];
}


void processScan(Scan *scan){

	//use the minimum x to start.
	double xStart= -MAX_DIST+scan->origin.x; 
	double xEnd= MAX_DIST+scan->origin.x;

	double yStart= -MAX_DIST+scan->origin.y;
	double yEnd= MAX_DIST+scan->origin.y;
/*
	for (double dx = xStart; dx < xEnd; dx +=X_RES){
		for (double dy = yStart; dy < yEnd; dy +=Y_RES){
			for (int i=0;i<scan->len();i++){
				Vector2d obstacle = scan->getObstacle(i);

				if(abs(dtheta) > toRad(15)){
					continue;
				}

				// lose precision based on angle from center.
				if (scalingVal >0){

					double dist = point.len();
					if (abs(dist - obs) < X_RES*2){
						//near the obstacle.
						val = 5;
					}
					else if (dist < .2) {
						//near the sensor. Don't trust the range as much
						val = .5;

					}else if (dist < obs){
						//clear out space in front of us
						val = .5;
					}else{
						//this is space behind the obstacle.. don't modify
						val=1;
					}

					conflatePixel(x+dx,y+dy,pow(val,scalingVal));//reduce the darkness of cells we are sitting on
				}	
			}
		}	
	}*/
}




/**
 ****************************************************************************
 *			mapping
 ****************************************************************************
 */

// values here should be between 0 and 1 to plot correctly.
// 0 will plot as white and 1 will plot as black.

Map localMap("test.pgm", "out.pgm", X_RES);

/* initMap()
 *
 *set the map to all unknown.)
 */

void initMap(){
}



static void display() {

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(	0, WIN_X, 0, WIN_Y );

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glBegin( GL_POINTS );
	for( int x = 0; x < WIN_X; x++ ) {
		for( int y = 0; y < WIN_Y; y++ ) {
			glColor3ub( localMap.getPixel(x*SCALE,y*SCALE), localMap.getPixel(x*SCALE,y*SCALE),localMap.getPixel(x*SCALE,y*SCALE));
			glVertex2i(x,y);
		}
	}
	glEnd();

	pthread_mutex_lock(&particles_mut);
	for(vector<Particle>::iterator it=particles.begin();it<particles.end();it++){
		it->draw();
	}

	pthread_mutex_unlock(&particles_mut);

	pthread_mutex_unlock(&display_mut);
	// Flush the pipeline
	glFlush();

	// Swap the buffers now that we're done drawing.
	glutSwapBuffers();
}


void redisplay(void) {
	pthread_mutex_lock(&display_mut);	
	glutPostRedisplay();
}

void* robotLoop(void* args) {
	double lastx,lasty,lasttheta;
	double x,y,theta;
	double dx,dy,dtheta;
	while(true) {

		redisplay();

		double turnrate, speed;

		// read from the proxies
		pRobot->Read();

		// Here is where you do your robot stuff
		// including presumably updating your map somehow


		x= pPosition->GetXPos();
		y= pPosition->GetYPos();
		theta = wrap(pPosition->GetYaw());

		dx=x-lastx;
		dy=y-lasty;
		dtheta=theta-lasttheta;

		Vector2d odometry(dx,dy);

		Scan scans(Vector2d(x,y),SONAR);

		for (int scan = 0; scan <8 ;scan ++){
			scans.addScan(scan,getRange(scan));
		}

		pthread_mutex_lock(&particles_mut);	
		//seed particles at our current position
		//update particles when we move
		if(odometry.len() > .5 || abs(dtheta) > .1){
			lastx=x;
			lasty=y;
			lasttheta=theta;	
		
			vector<Particle> newList;
			for(vector<Particle>::iterator it=particles.begin();it<particles.end();it++){
				vector<Particle> ret = it->update(odometry,dtheta,0);
				int x = it->origin.x;	
				int y = it->origin.y;	
				if(x<-XOFFSET|| x>XOFFSET || y<-YOFFSET || y > YOFFSET)
					particles.erase(it);
				for (int j=0;j<ret.size();j++){
					newList.push_back(ret[j]);
				}
			}

			while(particles.size()< 1000){
				double x,y;
				do{
					x= ((double)rand())/RAND_MAX*WIN_X_METERS-XOFFSET;
					y= ((double)rand())/RAND_MAX*WIN_Y_METERS-YOFFSET;

				}while(localMap.getVal(x,y) < .5 );	
				Particle part(x,y,wrap(((double)rand())/RAND_MAX*2*PI));
				particles.push_back(part);
			}
			//score each particle.
			double avgscore=0;
			double bestScore=0xFFFF;
			double averagex,averagey,averagetheta;
			int counter=0;
			double score;
			for(vector<Particle>::iterator it=particles.begin();it<particles.end();it++){
				score = it->score(&localMap,&scans);	
				avgscore += score;
				if(score<bestScore){
					bestScore=score;
				}
			}
			avgscore /=particles.size();
			for(vector<Particle>::iterator it=particles.begin();it<particles.end();it++){
				score = it->score(&localMap,&scans);	
				if(score < (avgscore+bestScore)/2 ){	
					counter++;
					for(int i=0;i<3;i++){
						if(particles.size()<5000)
							newList.push_back(Particle(*it));	
					}
					averagex+=it->origin.x;
					averagey+=it->origin.y;
					averagetheta+=it->theta;
				}else{

					particles.erase(it);
				}
			}

			for(int i =0;i<newList.size();i++){
				particles.push_back(newList[i]);
			}
			cout<<particles.size()<<" particles"<<endl;
			averagex /= counter;
			averagey /= counter;
			averagetheta /= counter;
			if (averagex < -XOFFSET || averagex > XOFFSET || averagey < -YOFFSET || averagey > YOFFSET){
				averagex=0;
				averagey=0;
			}

			pPosition->SetOdometry(averagex,averagey,wrap(averagetheta));
			//lastx=averagex;
			//lasty=averagey;
			//lasttheta=wrap(averagetheta);
		}
		pthread_mutex_unlock(&particles_mut);
		processScan(&scans);

	}

}


int main(int argc, char *argv[]) {
	int port = 0;
	char* host = (char *)"localhost";

	if (argc > 1) {
		port = atoi( argv[2] );
		host = argv[1];
	} else {
		cout<<"usage: "<<argv[0]<<"host port\r\n";
		exit(-1);
	}

	pRobot = new PlayerClient( host, port );
	pPosition = new Position2dProxy( pRobot, 0 );
	pSonar = new SonarProxy( pRobot, 0 );

	printf("player connected on port %d, proxies allocated\n", port);
	pPosition->SetMotorEnable(1);

	pthread_mutex_init(&display_mut, NULL);
	pthread_mutex_init(&particles_mut, NULL);


	glutInit( &argc, argv );
	glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE );
	glutInitWindowPosition( 50, 50 );
	glutInitWindowSize( WIN_X, WIN_Y );
	glutCreateWindow( "Map" );

	pthread_t thread_id;
	pthread_create(&thread_id, NULL, robotLoop, NULL);

	initMap();

	// Callbacks
	glutDisplayFunc( display );
	glutIdleFunc( display );

	glutMainLoop();

	pthread_mutex_lock(&display_mut);

	return 0;
}

// Example of pixel-plotting together with Player/Stage robot control
// zjb 4/10, based on code by John Hawley and original uncredited
//   Player/Stage example code.
// Not guaranteed to be decent code style but seems to work OK.

#include <GL/glut.h>
#include <libplayerc++/playerc++.h>
#include <pthread.h>

#include "Vector2d.h"
#include "Scan.h"
#include "map.h"

using namespace PlayerCc;
using namespace std;

/**
 ****************************************************************************
 *			Constants
 ****************************************************************************
 */

#define PI 3.14159265

#define MAX_DIST 4

#define WIN_X 1000
#define WIN_Y 1000

#define UNEXPLORED 1

#define X_RES	0.1	//resolution of 5cm per pixel yielding 30m x
#define Y_RES	0.1	//resolution of 5cm per pixel yielding 30m y

#define XOFFSET 50
#define YOFFSET 50

/**
 ****************************************************************************
 *			Globals
 ****************************************************************************
 */


#define toRad(deg) (deg*PI/180.0)


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

static volatile int good;
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

double wrap(double angle){
	double ret = angle;
	if (angle<-PI) ret+= 2*PI;
	if (angle>PI) ret+= -2*PI;
	return ret;

}


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

Map localMap("test.pgm", "out.pgm");

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
			glColor3f( localMap.getVal(x,y), localMap.getVal(x,y),localMap.getVal(x,y));
			glVertex2i(x, y);
		}
	}
	glEnd();

	// Flush the pipeline
	glFlush();

	// Swap the buffers now that we're done drawing.
	glutSwapBuffers();
	good = 1;
}


void redisplay(void) {
	if(good)
		glutPostRedisplay();
}

void* robotLoop(void* args) {

	while(true) {

		redisplay();

		double turnrate, speed;

		// read from the proxies
		pRobot->Read();

		// Here is where you do your robot stuff
		// including presumably updating your map somehow

		double x= pPosition->GetXPos();
		double y= pPosition->GetYPos();

		Scan scans(Vector2d(x,y),SONAR);

		for (int scan = 0; scan <8 ;scan ++){
			scans.addScan(scan,getRange(scan));
		}
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

	//  pthread_mutex_init(&mut, NULL);

	pthread_t thread_id;
	pthread_create(&thread_id, NULL, robotLoop, NULL);

	glutInit( &argc, argv );
	glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE );
	glutInitWindowPosition( 50, 50 );
	glutInitWindowSize( WIN_X, WIN_Y );
	glutCreateWindow( "Map" );

	initMap();

	// Callbacks
	glutDisplayFunc( display );
	glutIdleFunc( display );

	glutMainLoop();

	return 0;
}

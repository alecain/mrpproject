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
#include "Ploc.h"
#include "Particle.h"

using namespace PlayerCc;
using namespace std;


/**
 ****************************************************************************
 *			Definitions
 ****************************************************************************
 */


/**
 ****************************************************************************
 *			Globals
 ****************************************************************************
 */


static PlayerClient *pRobot;
static Position2dProxy *pPosition;
static SonarProxy *pSonar;
static RangerProxy *pRanger;
pthread_mutex_t display_mut;
pthread_mutex_t particles_mut;

//vector<Particle> particles;


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
 *			Sensor abstraction
 ****************************************************************************
 */



//abstraction of the ranger and sonar
double getRange(int index){
	return (*pSonar)[index];
}




/**
 ****************************************************************************
 *			mapping
 ****************************************************************************
 */

// values here should be between 0 and 1 to plot correctly.
// 0 will plot as white and 1 will plot as black.

Map localMap("test.pgm", "out.pgm", X_RES);
Ploc localizer(100,500,&localMap);


static void display() {

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D( 0, WIN_X, 0, WIN_Y );

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

	for(list<Particle>::iterator it=localizer.particles.begin();it!=localizer.particles.end();it++){
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
	double lastx=0,lasty=0,lasttheta=0;
	double x,y,theta;
	double dx,dy,dtheta;
	while(true) {

		redisplay();

		double turnrate, speed;

		// read from the proxies
		do{
			pRobot->Read();
		}while(pRanger->GetRangeCount() == 0);

		// Here is where you do your robot stuff
		// including presumably updating your map somehow


		x= pPosition->GetXPos();
		y= pPosition->GetYPos();
		theta = wrap(pPosition->GetYaw());

		dx=x-lastx;
		dy=y-lasty;
		dtheta=theta-lasttheta;

		Vector2d odometry(dx,dy);

		Scan scans(Vector2d(x,y),LASER);
		for (int scan = 0; scan < pRanger->GetRangeCount();scan ++){
			scans.addScan(scan,(*pRanger)[scan]);
		}

		cout<<"dtheta"<<dtheta<<" odom:"<<odometry<<endl;
		//seed particles at our current position
		//update particles when we move
		if( odometry.len() > .1 || abs(dtheta) > .1){
			pthread_mutex_lock(&particles_mut);
			lastx=x;
			lasty=y;
			lasttheta=theta;

			double dlinear=odometry.len(); 
			if(fabs(wrap(odometry.getAngle()-theta)) > PI/2){
				dlinear *= -1;
			}
			localizer.replenishParticles();
			localizer.updateParticles(dlinear,dtheta);
			localizer.scoreParticles(&scans);
			localizer.pruneParticles();
			localizer.getPose();
			//score each particle.



			//pPosition->SetOdometry(averagex,averagey,wrap(averagetheta));
			//lastx=averagex;
			//lasty=averagey;
			//lasttheta=wrap(averagetheta);
			pthread_mutex_unlock(&particles_mut);
		}

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
	pRanger= new RangerProxy( pRobot, 0 );

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


	// Callbacks
	glutDisplayFunc( display );
	glutIdleFunc( display );

	glutMainLoop();

	pthread_mutex_lock(&display_mut);

	free(pRobot);
	free(pPosition);
	free(pSonar);
	free(pRanger);

	return 0;
}

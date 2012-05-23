/*
 *Main navigator source file. I bet you can't guess what I gutted to write it
 *
 *
 */

//#define SCAN_SONAR

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
#include "PathPlanner.h"
#include "PathPoint.h"
#include "safeGoto.h"

using namespace PlayerCc;
using namespace std;


/**
 ****************************************************************************
 *			Definitions
 ****************************************************************************
 */
#define MIN_COV			3.0
#define GOAL_TOLERANCE	2.5

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
pthread_mutex_t ploc_mut;
//vector<Particle> particles;

double lastx=0,lasty=0,lasttheta=0;

Map localMap("test.pgm", "out.pgm", X_RES);
Map confMap("conf.pgm", "cout.pgm", X_RES);
Ploc localizer(800,2000,&localMap);
list<Particle> particles;
Pose estimate;
Pose est_modified;
PathPlanner planner(&confMap);
list<PathPoint *> route;
list<PathPoint *>::const_iterator current_waypoint;
SafeGoTo nav;

list<Vector2d> goals;

Scan scans_copy(Vector2d(0,0),SONAR);
double dx,dy,dtheta, dlinear;
double dtheta_copy, dlinear_copy;

double x,y,theta;

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
void Pose::draw(){
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


	//pose is green
	glColor3ub(0,255,0);	
	glLineWidth(2);
	glBegin(GL_LINE_STRIP);	
	glVertex2d(windowX(x0),windowY(y0));	
	glVertex2d(windowX(x1),windowY(y1));	
	glVertex2d(windowX(x2),windowY(y2));	
	glVertex2d(windowX(x1),windowY(y1));	
	glVertex2d(windowX(x3),windowY(y3));	
	glEnd();

}

void SafeGoTo::drawVelocity(Pose *pose){
	//draw a vector of lenth 1m starting v.x, v.y at angle theta
	//	  v1
	//	/ | \
	//    v2  |  v3
	//	  |
	//        v0

	double x0=pose->origin.x;
	double y0=pose->origin.y;
	double theta = pose->theta+this->Velocity.getAngle();
	double len = this->Velocity.len()*10;
	double x1=(x0+cos(theta)*len);
	double y1=(y0+sin(theta)*len);
	double x2=(x1-cos(theta+toRad(30))*.6);
	double y2=(y1-sin(theta+toRad(30))*.6);
	double x3=(x1-cos(theta-toRad(30))*.6);
	double y3=(y1-sin(theta-toRad(30))*.6);

	//velocity is blue
	glColor3ub(0,0,255);
	glLineWidth(2);
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
int rangeCount(){
#ifdef SCAN_SONAR
	return 8;
#else
	return pRanger->GetRangeCount();
#endif
}

//abstraction of the ranger and sonar
double getRange(int index){
#ifdef SCAN_SONAR
	return (*pSonar)[index];
#else
	return (*pRanger)[index];
#endif
}




/**
 ****************************************************************************
 *			mapping
 ****************************************************************************
 */

// values here should be between 0 and 1 to plot correctly.
// 0 will plot as white and 1 will plot as black.


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

	for(list<Particle>::iterator it=particles.begin();it!=particles.end();it++){
		it->draw();
	}

	pthread_mutex_unlock(&particles_mut);

	// Draw the paths
	glLineWidth(1);
	glColor3ub(230, 230, 255);
	for (vector< pair<PathPoint *, PathPoint *> >::const_iterator path = planner.getConnectionsBegin(); path < planner.getConnectionsEnd(); path++) {
		glBegin(GL_LINE_STRIP);
		glVertex2d(path->first->getX()/SCALE, path->first->getY()/SCALE);
		glVertex2d(path->second->getX()/SCALE, path->second->getY()/SCALE);
		glEnd();
	}

	// Draw the current route
	glLineWidth(1);
	glColor3ub(255, 200, 0);
	glBegin(GL_LINE_STRIP);
	for (list<PathPoint *>::const_iterator point = route.begin(); point != route.end(); point++)
		glVertex2d((*point)->getX()/SCALE, (*point)->getY()/SCALE);
	glEnd();

	// Draw the current goal
	glLineWidth(5);
	glColor3ub(255, 200, 0);
	glBegin(GL_LINE_STRIP);
		glVertex2d(localMap.xToMap(goals.front().x-.2)/SCALE,localMap.yToMap(goals.front().y-.2)/SCALE);
		glVertex2d(localMap.xToMap(goals.front().x+.2)/SCALE,localMap.yToMap(goals.front().y+.2)/SCALE);
	glEnd();


	// Draw the waypoints
	glBegin(GL_POINTS);
	glColor3ub(0, 150, 0);

	for (vector<PathPoint *>::const_iterator point = planner.getPointsBegin(); point < planner.getPointsEnd(); point++)
		// Why do I have to divide by 2
		glVertex2i((*point)->getX()/SCALE, (*point)->getY()/SCALE);

	glEnd();

	//draw estimated pose and current velocity
	est_modified.draw();
	nav.drawVelocity(&estimate);


	pthread_mutex_unlock(&display_mut);
	// Flush the pipeline
	glFlush();

	// Swap the buffers now that we're done drawing.
	glutSwapBuffers();
}


void* plocLoop(void* args) {
	while (true){

		pthread_mutex_lock(&ploc_mut);

		localizer.replenishParticles();
		localizer.updateParticles(dlinear_copy,dtheta_copy);
		localizer.scoreParticles(&scans_copy);
		localizer.pruneParticles();
		estimate =  localizer.getPose(pPosition, 50);

		lastx=x;
		lasty=y;
		lasttheta=theta;

		pthread_mutex_lock(&particles_mut);
		particles = localizer.particles;
		pthread_mutex_unlock(&particles_mut);

	}
}

void redisplay(void) {
	pthread_mutex_lock(&display_mut);
	glutPostRedisplay();
}

void* robotLoop(void* args) {
	bool	position_unknown = true;

	//set current nav goal to get us moving
	nav.goTo(Vector2d(goals.front().x,goals.front().y));


	while(true) {

		redisplay();
		double turnrate, speed;

		// read from the proxies
		do{
			pRobot->Read();
		}while(rangeCount() == 0);

		// Here is where you do your robot stuff
		// including presumably updating your map somehow

		x= pPosition->GetXPos();
		y= pPosition->GetYPos();
		theta = wrap(pPosition->GetYaw());

		dx=x-lastx;
		dy=y-lasty;
		dtheta=theta-lasttheta;

		Vector2d odometry(dx,dy);
#ifdef SCAN_SONAR
		Scan scans(Vector2d(x,y),SONAR);
#else
		Scan scans(Vector2d(x,y),LASER);
#endif

		for (int scan = 0; scan < rangeCount();scan++){
			scans.addScan(scan,getRange(scan));
		}

		//seed particles at our current position
		//update particles when we move
		if( odometry.len() > .1 || abs(dtheta) > .08){

			dlinear=odometry.len();
			if(fabs(wrap(odometry.getAngle()-theta)) > PI/2){
				dlinear *= -1;
			}

			dlinear_copy=dlinear;
			dtheta_copy=dtheta;
			scans_copy=scans;
			//We've moved far enough.. unblock processing thread
			pthread_mutex_unlock(&ploc_mut);

			if (estimate.sigx < MIN_COV && estimate.sigy < MIN_COV){
				if (position_unknown){
					position_unknown=false;

					route = planner.findRoute(localMap.xToMap(estimate.origin.x), localMap.yToMap(estimate.origin.y));
					current_waypoint = route.begin();
					//set current nav goal

					double goalx = localMap.xToMeters((*current_waypoint)->getX());
					double goaly = localMap.yToMeters((*current_waypoint)->getY());
					nav.goTo(Vector2d(goalx,goaly));
				}

				if (estimate.origin.distance(nav.goal) < GOAL_TOLERANCE && route.size() > 0 ){
					cout<<"waypoint reached"<<endl;
					current_waypoint++;
					if (current_waypoint == route.end() && current_waypoint != route.begin()){
						cout<<"goal reached"<<endl;
						if (goals.size() > 0){
							goals.pop_front();
							Vector2d goal = goals.front();
							planner.generatePaths(1000, localMap.xToMap(goal.x),localMap.xToMap(goal.y));
						} else{
							cout<<"All goals finished"<<endl;
							exit(0);
						}
					}
					double goalx = localMap.xToMeters((*current_waypoint)->getX());
					double goaly = localMap.yToMeters((*current_waypoint)->getY());
					nav.goTo(Vector2d(goalx,goaly));
				}
			} else if (estimate.sigx > MIN_COV*2 && estimate.sigy > MIN_COV *2){ //leave a gap to trigger saying our position is unknown
				position_unknown=true;
			}

		}
		//cout<<"Pose: "<< estimate.origin.x<< "," <<estimate.origin.y<<"->"<< estimate.origin.distance(nav.goal)<<endl;
		est_modified = estimate;
		//modify the estimate based on the odometry.
		est_modified.origin = estimate.odomTransform.plus(Vector2d(pPosition->GetXPos(), pPosition->GetYPos()));
		est_modified.theta = estimate.thetaodom+ pPosition->GetYaw();

		//recalculate velocity
		nav.update(&scans,&est_modified);
		nav.applyVelocity(pPosition);

	}
}

int main(int argc, char *argv[]) {
	int port = 0;
	char* host = (char *)"localhost";

	if (argc > 3) {
		port = atoi( argv[2] );
		host = argv[1];
	} else {
		cout<<"usage: "<<argv[0]<<"host port point-file\r\n";
		exit(-1);
	}

	FILE *fp = fopen(argv[3],"rd");
	int read;
	double x,y;
	do{
		read=fscanf(fp,"%lf %lf",&x,&y);
		if (read == 2){
			cout<<"goal :"<<x <<","<<y<<endl;
			goals.push_back(Vector2d(x,y));
		}
	}while (read== 2);

	srand(time(NULL));

	pRobot = new PlayerClient( host, port );
	pPosition = new Position2dProxy( pRobot, 0 );
#ifdef SCAN_SONAR
	pSonar = new SonarProxy( pRobot, 0 );
#else
	pRanger= new RangerProxy( pRobot, 0 );
#endif

	printf("player connected on port %d, proxies allocated\n", port);
	pPosition->SetMotorEnable(1);

	pthread_mutex_init(&display_mut, NULL);
	pthread_mutex_init(&particles_mut, NULL);
	pthread_mutex_init(&ploc_mut,NULL);

	if (goals.size() > 0){
		Vector2d goal = goals.front();
		planner.generatePaths(1000, localMap.xToMap(goal.x),localMap.xToMap(goal.y));
		cout<<"current goal :"<<localMap.xToMap(goal.x) <<","<<localMap.xToMap(goal.y)<<endl;
	} else{
		cout<<"No goals specified"<<endl;
		exit(-1);
	}

	glutInit( &argc, argv );
	glutInitDisplayMode( GLUT_RGB | GLUT_DOUBLE );
	glutInitWindowPosition( 50, 50 );
	glutInitWindowSize( WIN_X, WIN_Y );
	glutCreateWindow( "Map" );

	pthread_t thread_id;
	pthread_create(&thread_id, NULL, robotLoop, NULL);

	pthread_t ploc_thread_id;
	pthread_create(&ploc_thread_id, NULL, plocLoop, NULL);

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

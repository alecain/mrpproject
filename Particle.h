
/*
 * particle for localization 
 *
 *	Author: Andrew LeCain
 */
#include "Vector2d.h"
#include <vector>
using namespace std;



class Particle{
	public:
		Particle(double x, double y, double theta);
		vector<Particle> update(Vector2d odometry, int toSpawn);
		Vector2d origin;
		double theta;
		static double thetaThetaCov;
		static double thetaLinearCov;
		static double linearLinearCov;
		static double linearThetaCov;

		static void setTTCovariance(double thetaThetaCov); 
		static void setTLCovariance(double thetaLinearCov); 
		static void setLLCOvariance(double linearLinearCov); 
		static void setLTCovariance(double linearThetaCov); 

};


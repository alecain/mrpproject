

#include <math.h>
#include "Vector2d.h"

#include <ostream>
using namespace std;
/**
 ****************************************************************************
 *			Vector2d functions
 ****************************************************************************
 */

#define PI 3.14159265


double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}


std::ostream& operator<<(std::ostream& out, const Vector2d& v){
	return out << "x: "<<v.x <<"\ty : " << v.y << "\tmagnitude: "<< v.len()  <<"\ttheta: " << v.getAngle()*180.0 / PI <<std::endl;
}


Vector2d::Vector2d(void){
	this->x=0;
	this->y=0;
}
Vector2d::Vector2d(double x, double y){
	this->x=x;
	this->y=y;
}
Vector2d Vector2d::minus(Vector2d b){
	Vector2d ret;
	ret.x=this->x-b.x;
	ret.y=this->y-b.y;
	return ret;
}

Vector2d Vector2d::plus(Vector2d b){
	Vector2d ret;
	ret.x=this->x+b.x;
	ret.y=this->y+b.y;
	return ret;
}

Vector2d Vector2d::operator+=(Vector2d other){
	this->x += other.x;
	this->y += other.y;
}

Vector2d Vector2d::divide(double scalar){
	Vector2d ret;
	ret.x = this->x/scalar;
	ret.y = this->y/scalar;
	return ret;

}
Vector2d Vector2d::times(double scalar){
	Vector2d ret;
	ret.x = this->x*scalar;
	ret.y = this->y*scalar;
	return ret;
}

double Vector2d::getAngle() const { //returns angle in radians
	return atan2(this->y,this->x);
}

double Vector2d::distance( Vector2d b){
	return getDistance(this->x,this->y,b.x,b.y);
}

double Vector2d::len() const{
	return sqrt(this->x*this->x + this->y*this->y);
}
Vector2d Vector2d::norm(){

	Vector2d ret=this->divide(this->len());
	return ret;
}

Vector2d Vector2d::clip(double max){
	Vector2d ret = Vector2d(*this);
	if(this->len() > max){
		return this->norm().times(max);
	}
	return ret;
}
double Vector2d::dot(Vector2d b){
	return this->x*b.x+this->y*b.y;
}


//returns the projection of this vector onto b.
Vector2d Vector2d::proj(Vector2d b){
	Vector2d c = b.norm().times(this->dot(b.norm()));
	return c;
}


//rotates a Vector2d by a given angle
Vector2d Vector2d::rotate(double theta){
	Vector2d ret (0,0);
	ret.x= this->x*cos(theta) -this->y*sin(theta);
	ret.y= this->x*sin(theta) +this->y*cos(theta);
	return ret;	

}
Vector2d Vector2d::rotateAbs(double theta){
	Vector2d ret(0,0);
	ret.x= this->len()*cos(theta);
	ret.y = this->len()*sin(theta);
	return ret;	

}

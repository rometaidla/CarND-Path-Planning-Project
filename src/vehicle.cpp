#include "vehicle.h"
#include <math.h>

Vehicle::Vehicle(double velocity_x, double velocity_y, double s, double d) {
	this->velocity_x = velocity_x;
	this->velocity_y = velocity_y;
	this->velocity = sqrt(velocity_x*velocity_x+velocity_y*velocity_y);
	this->s = s;
	this->d = d;
}
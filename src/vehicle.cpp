#include "vehicle.h"
#include <math.h>

Vehicle::Vehicle(double velocity_x, double velocity_y, double s, double d) {
	this->s = s;
	this->d = d;
	this->velocity = sqrt(velocity_x*velocity_x+velocity_y*velocity_y);
}

Vehicle::Vehicle(double x, double y, double yaw, double velocity, double s, double d) {
	this->x = x;
	this->y = y;
	this->yaw = yaw;
	this->s = s;
	this->d = d;
	this->velocity = velocity;
}


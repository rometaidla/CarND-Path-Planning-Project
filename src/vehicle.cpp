#include <math.h>

#include "vehicle.h"
#include "constants.h"

Vehicle::Vehicle(double x, double y, double yaw, double velocity, double s, double d) {
	this->x = x;
	this->y = y;
	this->yaw = yaw;
	this->s = s;
	this->d = d;
	this->velocity = velocity;
	this->lane = (int)(d/LANE_WIDTH);
}


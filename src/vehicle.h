#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle {
public:
	Vehicle(double x, double y, double yaw, double velocity, double s, double d);

	double x;
	double y;
	double yaw;
    double velocity;
    double s;
    double d;
    int lane;
};

#endif /* VEHICLE_H */
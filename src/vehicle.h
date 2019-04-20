#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle {
public:
	Vehicle(double velocity_x, double velocity_y, double s, double d);
	Vehicle(double x, double y, double yaw, double velocity, double s, double d);

	double x;
	double y;
	double yaw;
    double velocity;
    double s;
    double d;
};

#endif /* VEHICLE_H */
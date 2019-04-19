#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle {
public:
	Vehicle(double velocity_x, double velocity_y, double s, double d);

	double velocity_x;
    double velocity_y;
    double velocity;
    double s;
    double d;
};

#endif /* VEHICLE_H */
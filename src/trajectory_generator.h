#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>

#include "vehicle.h"

using std::vector;

class TrajectoryGenerator {
public:
	TrajectoryGenerator(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s);
	// TODO: pack parameters into a class (vehicle or car)
	vector<vector<double>> generateTrajectory(double car_x, double car_y, double car_yaw, double car_s, int lane, double reference_velocity, vector<vector<double>> previous_path);

	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
};

#endif /* TRAJECTORY_GENERATOR_H */
#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <vector>
#include "vehicle.h"
#include "trajectory_generator.h"

using std::vector;

class BehaviorPlanner {
public:
	BehaviorPlanner(double ref_vel, int lane);

	double ref_vel;
	int lane;
	vector<vector<double>> best_trajectory;

	void plan_trajectory(Vehicle ego_vehicle, vector<Vehicle> other_vehicles, 
		TrajectoryGenerator trajectory_generator, vector<vector<double>> previous_path);

	vector<double> generate_candidate_velocities();

	vector<int> generate_candidate_lanes(Vehicle ego_vehicle);

	double calculateCost(Vehicle ego_vehicle, vector<Vehicle> other_vehicles, 
		int prev_size, int candidate_lane, double candidate_velocity);
};

#endif /* BEHAVIOR_PLANNER_H */
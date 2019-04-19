#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <vector>
#include "vehicle.h"

using std::vector;

class BehaviorPlanner {
public:
	BehaviorPlanner();
	double calculateCost(vector<vector<double>> trajectory, vector<Vehicle> other_vehicles, 
		int prev_size, int lane, double car_s, double velocity);
};

#endif /* BEHAVIOR_PLANNER_H */
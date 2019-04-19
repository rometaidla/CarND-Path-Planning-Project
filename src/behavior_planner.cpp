#include <iostream>
#include <string>

#include "behavior_planner.h"
#include "constants.h"

using std::string;
using namespace std;

BehaviorPlanner::BehaviorPlanner() {

}

double BehaviorPlanner::calculateCost(vector<vector<double>> trajectory, vector<Vehicle> other_vehicles, int prev_size, int lane, double car_s, double velocity) {
	
	double cost = 50 - velocity;


	// COLLISION
	double safety_distance = 30.0; // todo: make constant or better relative to vehicles speed
	for (int i = 0; i < other_vehicles.size(); i++) {
		Vehicle other_vehicle = other_vehicles[i];
		if (other_vehicle.d < (2+4*lane+2) && other_vehicle.d > (2+4*lane-2)) {
		    double other_vehicle_projected_s = other_vehicle.s + (double)prev_size * SIMULATOR_DT * other_vehicle.velocity;
		    double distance_to_car_in_front = other_vehicle_projected_s-car_s;
		    if (other_vehicle_projected_s > car_s && distance_to_car_in_front < safety_distance) {
		    	cout << "WARNING: vehicle on same lane: distance=" << distance_to_car_in_front << endl;
		        cost += velocity * 100;
		    }
		}
	}

	// SPEED LIMIT
	if (velocity > (SPEED_LIMIT - 0.5)) {
		cost += 100.0;
	}

	return cost;
}
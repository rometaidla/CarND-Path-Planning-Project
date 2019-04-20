#include <iostream>
#include <string>

#include "behavior_planner.h"
#include "constants.h"

using std::string;
using namespace std;

BehaviorPlanner::BehaviorPlanner(double ref_vel, int lane) {
	this->ref_vel = ref_vel; // TODO: should be store in vehicle object
	this->lane = lane;
}

void BehaviorPlanner::plan_trajectory(Vehicle ego_vehicle, vector<Vehicle> other_vehicles, int current_lane, 
	TrajectoryGenerator trajectory_generator, vector<vector<double>> previous_path) {
	cout << "**** FINDING BEST TRAJECTORY ****" << endl;
	vector<double> candidate_velocities = { ref_vel+0.50, ref_vel+0.25, 
		ref_vel, ref_vel-0.25, ref_vel-0.50};
	
	vector<int> candidate_lanes = { current_lane };
	if (current_lane-1 >= 0) {
		candidate_lanes.push_back(current_lane-1);
	}
	if (current_lane+1 <= 2) {
		candidate_lanes.push_back(current_lane+1);
	}

	int prev_size = previous_path[0].size();

	double current_minimal_cost = -1.0;

	for (int l = 0; l < candidate_lanes.size(); l++) {
		int candidate_lane = candidate_lanes[l];

		for (int i = 0; i < candidate_velocities.size(); i++) {
		  double candidate_velocity = candidate_velocities[i];
		  
		  vector<vector<double>> candidate_trajectory = trajectory_generator.generateTrajectory(ego_vehicle.x, ego_vehicle.y, ego_vehicle.yaw, ego_vehicle.s, 
		  	current_lane, candidate_velocity, previous_path);
		  
		  double candidate_trajectory_cost = this->calculateCost(candidate_trajectory, other_vehicles, 
		    prev_size, candidate_lane, ego_vehicle.s, candidate_velocity);

		  cout << "Candidate trajectory: velocity=" << candidate_velocity << ", lane=" << candidate_lane << ", cost=" << candidate_trajectory_cost << endl;

		  if (current_minimal_cost < 0.0 || candidate_trajectory_cost < current_minimal_cost) {
		    current_minimal_cost = candidate_trajectory_cost;
		    
		    this->best_trajectory = candidate_trajectory;
		    this->ref_vel = candidate_velocity;
		    this->lane = candidate_lane; 
		  } 
		}
	}
}

double BehaviorPlanner::calculateCost(vector<vector<double>> trajectory, vector<Vehicle> other_vehicles, int prev_size, int lane, double car_s, double velocity) {
	
	double cost = 50 - velocity;


	// COLLISION
	double safety_distance = 30.0; // todo: make constant or better relative to vehicles speed
	for (int i = 0; i < other_vehicles.size(); i++) {
		Vehicle other_vehicle = other_vehicles[i];
		double other_vehicle_projected_s = other_vehicle.s + (double)prev_size * SIMULATOR_DT * other_vehicle.velocity;
		//cout << "other vehicle d=" << other_vehicle.d << ", s=" << other_vehicle.s << " projected s=" << other_vehicle_projected_s << endl;
		if (other_vehicle.d < (2+4*lane+2) && other_vehicle.d > (2+4*lane-2)) {
		    double distance_to_car_in_front = other_vehicle_projected_s-car_s;
		    if (other_vehicle_projected_s > car_s && distance_to_car_in_front < safety_distance) {
		    	//cout << "WARNING: vehicle on same lane: distance=" << distance_to_car_in_front << endl;
		        cost += velocity * 100;
		    }
		}
	}

	// SPEED LIMIT
	if (velocity > (SPEED_LIMIT - 0.5)) {
		cost += 100.0;
	}

	// PREFER MIDDLE LANE
	if (lane != MIDDLE_LANE) {
		cost += 10;
	}

	return cost;
}
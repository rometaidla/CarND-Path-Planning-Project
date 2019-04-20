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

void BehaviorPlanner::plan_trajectory(Vehicle ego_vehicle, vector<Vehicle> other_vehicles, 
	TrajectoryGenerator trajectory_generator, vector<vector<double>> previous_path) {
	cout << "**** PLANNING TRAJECTORY ****" << endl;
	vector<double> candidate_velocities = { ref_vel+0.50, ref_vel+0.25, 
		ref_vel, ref_vel-0.25, ref_vel-0.50};
	
	vector<int> candidate_lanes;
	if (ego_vehicle.lane-1 >= 0) {
		candidate_lanes.push_back(ego_vehicle.lane-1);
	}
	
	candidate_lanes.push_back(ego_vehicle.lane);

	if (ego_vehicle.lane+1 <= 2) {
		candidate_lanes.push_back(ego_vehicle.lane+1);
	}

	int prev_size = previous_path[0].size();

	double current_minimal_cost = -1.0;

	for (int l = 0; l < candidate_lanes.size(); l++) {
		int candidate_lane = candidate_lanes[l];

		for (int i = 0; i < candidate_velocities.size(); i++) {
		  double candidate_velocity = candidate_velocities[i];
		  double candidate_trajectory_cost = this->calculateCost(ego_vehicle, other_vehicles, 
		    prev_size, candidate_lane, candidate_velocity);

		  cout << "Candidate trajectory: canidate_lane=" << candidate_lane << ", candidate_velocity=" << candidate_velocity << ", cost=" << candidate_trajectory_cost << endl; 
		  if (current_minimal_cost < 0.0 || candidate_trajectory_cost < current_minimal_cost) {
		    current_minimal_cost = candidate_trajectory_cost;
		    
		    double candidate_lane_d = 2+4*candidate_lane; // TODO: refactor
			this->best_trajectory = trajectory_generator.generateTrajectory(ego_vehicle.x, ego_vehicle.y, ego_vehicle.yaw, ego_vehicle.s, 
				candidate_lane_d, candidate_velocity, previous_path);		  
		    this->ref_vel = candidate_velocity;
		    this->lane = candidate_lane; 
		  } 
		}
	}
}

double BehaviorPlanner::calculateCost(Vehicle ego_vehicle, vector<Vehicle> other_vehicles, int prev_size, int candidate_lane, double candidate_velocity) {
	
	double cost = 0;

	// COLLISION & LINE SELECTION
	double safety_distance_front = 30.0; // todo: make constant or better relative to vehicles speed
	double safety_distance_back = 20.0; // smaller safety distance, so line changes would be more easily available
	
	for (int i = 0; i < other_vehicles.size(); i++) {
		Vehicle other_vehicle = other_vehicles[i];
		
		double other_vehicle_projected_s = other_vehicle.s + (double)prev_size * SIMULATOR_DT * other_vehicle.velocity;
		bool is_lane_change = ego_vehicle.lane != candidate_lane;

		if (!is_lane_change && other_vehicle.lane == candidate_lane) {
		    if (other_vehicle_projected_s > ego_vehicle.s && other_vehicle_projected_s-ego_vehicle.s < safety_distance_front) {
		        cost += candidate_velocity * 100;
		    }
		}

		if (is_lane_change && other_vehicle.lane == candidate_lane) {
			if (other_vehicle_projected_s > ego_vehicle.s && other_vehicle_projected_s-ego_vehicle.s < safety_distance_front) {
		        cost += candidate_velocity * 200;
		    }

		    if (other_vehicle_projected_s < ego_vehicle.s && ego_vehicle.s-other_vehicle_projected_s < safety_distance_back) {
		        cost += candidate_velocity * 200;
		    }

		    // PREFER EMPTY LANES
			if (other_vehicle_projected_s > ego_vehicle.s) {
				cost += 10 * (other_vehicle_projected_s-ego_vehicle.s);
			}
		}
	}

	// PREFER FASTER SPEED
	cost += (SPEED_LIMIT - candidate_velocity) * 10.0;

	// SPEED LIMIT
	if (candidate_velocity > (SPEED_LIMIT - 0.5)) {
		cost += 1000.0;
	}

	// PREFER MIDDLE LANE
	if (candidate_lane != MIDDLE_LANE) {
		cost += 10;
	}

	return cost;
}
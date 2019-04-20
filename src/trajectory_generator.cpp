#include "trajectory_generator.h"

#include <vector>
#include "spline.h"
#include "helpers.h"

using std::vector;

TrajectoryGenerator::TrajectoryGenerator(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s) {
	this->map_waypoints_x = map_waypoints_x;
	this->map_waypoints_y = map_waypoints_y;
	this->map_waypoints_s = map_waypoints_s;
}

vector<vector<double>> TrajectoryGenerator::generateTrajectory(double car_x, double car_y, double car_yaw, double car_s, double target_d, double reference_velocity, vector<vector<double>> previous_path) {
	
	vector<double> previous_path_x = previous_path[0];
	vector<double> previous_path_y = previous_path[1];
	int prev_size = previous_path_x.size();

	// Create a list of widely spaced (x,y) anchor points, evenly spaced at 30m
	vector<double> anchor_points_x;
	vector<double> anchor_points_y;

	// Reference x, y, yaw states
	double ref_x = car_x;
	double ref_y = car_y;
	double ref_yaw = deg2rad(car_yaw);

	// if previous size is almost empty, use the car as starting reference
	if (prev_size < 2) {
		double prev_car_x = car_x - cos(car_yaw);
		double prev_car_y = car_y - sin(car_yaw);

		anchor_points_x.push_back(prev_car_x);
		anchor_points_y.push_back(prev_car_y);

		anchor_points_x.push_back(car_x);
		anchor_points_y.push_back(car_y);
	}
	// use the previous path's end point as starting reference
	else {
		ref_x = previous_path_x[prev_size-1];
		ref_y = previous_path_y[prev_size-1];

		double ref_x_prev = previous_path_x[prev_size-2];
		double ref_y_prev = previous_path_y[prev_size-2];
		ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

		anchor_points_x.push_back(ref_x_prev);
		anchor_points_x.push_back(ref_x);

		anchor_points_y.push_back(ref_y_prev);
		anchor_points_y.push_back(ref_y);
	}

	int anchor_points_count = 3;
	int anchor_points_spacing = 30;
	for (int i = 1; i <= anchor_points_count; i++) {
		int anchor_point_s = car_s + i*anchor_points_spacing;
		vector<double> anchor_point = getXY(anchor_point_s, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		anchor_points_x.push_back(anchor_point[0]);
		anchor_points_y.push_back(anchor_point[1]);  
	}

	for (int i = 0; i <  anchor_points_x.size(); i++) { // todo: remove double loop over anchor points
		// shift car reference angle to 0 degrees
		double shift_x = anchor_points_x[i]-ref_x;
		double shift_y = anchor_points_y[i]-ref_y;

		anchor_points_x[i] = shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
		anchor_points_y[i] = shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
	}

	tk::spline s;
	s.set_points(anchor_points_x, anchor_points_y);

	vector<double> interpolated_points_x; // todo: refactor interpolated / spline points
	vector<double> interpolated_points_y;

	for (int i = 0; i < previous_path_x.size(); i++) {
		interpolated_points_x.push_back(previous_path_x[i]);
		interpolated_points_y.push_back(previous_path_y[i]);
	}

	double target_x = 30.0; // todo: rename horizon length
	double target_y = s(target_x);
	double target_dist = sqrt(target_x*target_x + target_y*target_y); // todo: helper, ecluadian distance 

	double x_add_on = 0;

	for (int i = 1; i <= 50-previous_path_x.size(); i++) {
		double N = target_dist / (.02*reference_velocity/2.24); // todo: rename step length, move outside of loop
		double x_point = x_add_on+target_x/N;
		double y_point = s(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		// rotate back to normal after rotating it earlier
		x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
		y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

		x_point += ref_x;
		y_point += ref_y;

		interpolated_points_x.push_back(x_point);
		interpolated_points_y.push_back(y_point);
	}

	return { interpolated_points_x, interpolated_points_y };
}


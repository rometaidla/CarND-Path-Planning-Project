#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  double ref_vel = 0.0; // mph
  int lane = 1;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ref_vel, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          bool too_close = false;
          double safety_distance = 30.0; // todo: make constant or better relative to vehicles speed
          for (int i = 0; i < sensor_fusion.size(); i++) {
            float d = sensor_fusion[i][6];

            if (d < (2+4*lane+2) && d > (2+4*lane-2)) {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double other_vehicle_velocity = sqrt(vx*vx+vy*vy);
                double other_vehicle_s = sensor_fusion[i][5]; 

                double other_vehicle_projected_s = other_vehicle_s + (double)prev_size * .02 * other_vehicle_velocity;
                if (other_vehicle_projected_s > car_s && other_vehicle_projected_s-car_s < safety_distance) {
                    too_close = true;
                }
            }
          }

          if (too_close) { // todo: remove magic value
            ref_vel -= .224;
          }
          else if (ref_vel < 49.5) { // todo: remove magic value
            ref_vel += .224;
          }

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
            vector<double> anchor_point = getXY(anchor_point_s, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
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
            double N = target_dist / (.02*ref_vel/2.24); // todo: rename step length, move outside of loop
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

          json msgJson;
          msgJson["next_x"] = interpolated_points_x;
          msgJson["next_y"] = interpolated_points_y;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
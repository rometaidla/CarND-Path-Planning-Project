#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "vehicle.h"
#include "trajectory_generator.h"
#include "behavior_planner.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

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
          vector<vector<double>> previous_path = { previous_path_x, previous_path_y };

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          // PARSE SENSOR FUSION DATA 
          vector<Vehicle> other_vehicles;
          for (int i = 0; i < sensor_fusion.size(); i++) {
            double velocity_x = sensor_fusion[i][3];
            double velocity_y = sensor_fusion[i][4];
            double s = sensor_fusion[i][5];
            double d = sensor_fusion[i][6];
            Vehicle vehicle = Vehicle(velocity_x, velocity_y, s, d);
            other_vehicles.push_back(vehicle);
          }

          cout << "**** FINDING BEST TRAJECTORY ****" << endl;
          vector<double> candidate_velocities = { ref_vel+0.50, ref_vel+0.25, ref_vel, ref_vel-0.25, ref_vel-0.50};

          TrajectoryGenerator trajectory_generator = TrajectoryGenerator(map_waypoints_x, map_waypoints_y, map_waypoints_s);
          BehaviorPlanner behavior_planner = BehaviorPlanner();
          double current_minimal_cost = -1.0;
          vector<vector<double>> current_best_trajectory; // TODO: create trajectory class
          for (int i = 0; i < candidate_velocities.size(); i++) {
            double candidate_velocity = candidate_velocities[i];
            vector<vector<double>> candidate_trajectory = trajectory_generator.generateTrajectory(car_x, car_y, car_yaw, car_s, lane, candidate_velocity, previous_path);
            double candidate_trajectory_cost = behavior_planner.calculateCost(candidate_trajectory, other_vehicles, 
              prev_size, lane, car_s, candidate_velocity);

            cout << "Candidate trajectory: velocity=" << candidate_velocity << ", cost=" << candidate_trajectory_cost << endl;

            if (current_minimal_cost < 0.0 || candidate_trajectory_cost < current_minimal_cost) {
              current_minimal_cost = candidate_trajectory_cost;
              current_best_trajectory = candidate_trajectory;
              ref_vel = candidate_velocity; 
            } 
          }

          cout << "best trajectory with cost=" << current_minimal_cost << endl;

          json msgJson;
          msgJson["next_x"] = current_best_trajectory[0];
          msgJson["next_y"] = current_best_trajectory[1];

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
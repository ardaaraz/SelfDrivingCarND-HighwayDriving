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
#include "param.h"

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
  // PARAMETERS

  // Define initial ego lane
  double lane_ego = 1.0;
  // Define a target velocity for ego
  double ref_vel = 0.0; //m/s

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

  h.onMessage([&lane_ego, &ref_vel,
               &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // PREDICTION
          
          // Previos path size
          int prev_size = previous_path_x.size();
          if(prev_size > 0)
          {
            car_s = end_path_s;
          }
          // Get adjusted velocity for avoiding collision
          vector<double> ego_vehicle = {car_x, car_y, car_yaw, car_speed, car_s, car_d, lane_ego};
          
          double adjusted_vel = adjustEgoTargetVel(ego_vehicle, sensor_fusion, prev_size);
          std::cout << lane_ego << std::endl;
          // Adjust ego speed according to predictions
          if(adjusted_vel < ref_vel)
          {
            ref_vel -= ACC_LIM * 0.02; //Increase reference velocity according to acceleration limt (10 m/s^2)
          }
          if(adjusted_vel > ref_vel)
          {
            ref_vel += ACC_LIM * 0.02; //Increase reference velocity according to deceleration limt (-10 m/s^2)
          }

          // BEHAVIOUR PLANNING

          // Calculate costs for the lanes
          vector<double> velocity_cost = speedCostForLanes(ego_vehicle, sensor_fusion, prev_size);
          vector<double> dist_cost = distCostForLanes(ego_vehicle, sensor_fusion, prev_size);
          vector<double> total_cost{0.0, 0.0, 0.0};
          for(int i = 0; i < MAX_LANE; ++i)
          {
            total_cost[i] = velocity_cost[i] + dist_cost[i];
          }
          // Find minimum cost lane
          vector<double>::iterator cost_iterator = std::min_element(total_cost.begin(), total_cost.end());
          int min_cost_lane = std::distance(total_cost.begin(), cost_iterator);
          
          // Set ego lane to the lowest cost lane
          lane_ego = static_cast<double>(min_cost_lane);

          std::cout << lane_ego << std::endl;
          // TRAJECTORY GENERATION
          // Spline x-y coordinates
          vector<double> splinepts_x;
          vector<double> splinepts_y;

          // Reference x, y & yaw
          double ref_x   = car_x;
          double ref_y   = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // Check previous path size: If the previous path is almost empty use the car states as starting reference
          if(prev_size < 2)
          {
            // Use two points to generate path tangent to the car
            splinepts_x.push_back(car_x - cos(car_yaw));
            splinepts_y.push_back(car_y - sin(car_yaw));

            splinepts_x.push_back(car_x);
            splinepts_y.push_back(car_y);
          }
          // Use the previous path to generate reference
          else
          {
            // Update reference x,y & yaw
            ref_x             = previous_path_x[prev_size-1];
            ref_y             = previous_path_y[prev_size-1];
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw           = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            // Use two points to generate path tangent to the car
            splinepts_x.push_back(ref_x_prev);
            splinepts_y.push_back(ref_y_prev);

            splinepts_x.push_back(ref_x);
            splinepts_y.push_back(ref_y);
          }

          // In FreNet add 3 waypoints which are evenly 30 m spaced points ahead of the starting refence point
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane_ego), map_waypoints_s,map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane_ego), map_waypoints_s,map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane_ego), map_waypoints_s,map_waypoints_x, map_waypoints_y);

          splinepts_x.push_back(next_wp0[0]);
          splinepts_x.push_back(next_wp1[0]);
          splinepts_x.push_back(next_wp2[0]);

          splinepts_y.push_back(next_wp0[1]);
          splinepts_y.push_back(next_wp1[1]);
          splinepts_y.push_back(next_wp2[1]);

          // Shift & rotate
          for(int i = 0; i < splinepts_x.size(); i++)
          {
            double shift_x = splinepts_x[i] - ref_x;
            double shift_y = splinepts_y[i] - ref_y;
            
            splinepts_x[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
            splinepts_y[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);

          }

          // Create a spline instance
          tk::spline s;
          // Set spline x-y points
          s.set_points(splinepts_x, splinepts_y);
          
          // Create trajectory x-y points starting from previous path points
          for(int i = 0; i < prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          // Create way points so that ego vehicle move at desired velocity
          double target_x    = 30.0;
          double target_y    = s(target_x);
          double target_dist = distance(0.0, 0.0, target_x, target_y); 
          
          double x_add_on = 0.0;

          // Fill the rest waypoints 
          for(int i = 0; i < WAY_POINT_NUM - prev_size; i++)
          {
            double N = target_dist/(ref_vel*0.02);
            double x_point = x_add_on + (target_x/N);
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // Reverse rotate & shift
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            // Fill the trajectory waypoints
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          //END
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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
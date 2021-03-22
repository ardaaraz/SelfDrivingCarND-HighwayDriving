#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "param.h"

// for convenience
using std::string;
using std::vector;

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

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

//Adjust ego vehicle speed according to sensor fusion data
double adjustEgoTargetVel(const vector<double> &ego_vehicle, 
                          const vector<vector<double>> &sensor_fusion,
                          const int prev_size)
{
  // Define adjusted target speed for ego vehicle
  double adjusted_vel = TARGET_VEL;
  // Adjust ego vehicle speed using sensor fusion info
  for(int i = 0; i < sensor_fusion.size(); i++)
  {
    double vehicle_d = sensor_fusion[i][6]; //Other vehicles d location
    // Check another car exist in ego lane
    if(vehicle_d < (2+ego_vehicle[6]*4+2) && vehicle_d > (2+ego_vehicle[6]*4-2))
    {
      double vehicle_vx    = sensor_fusion[i][3];
      double vehicle_vy    = sensor_fusion[i][4];
      double vehicle_speed = sqrt(vehicle_vx*vehicle_vx + vehicle_vy*vehicle_vy);
      double vehicle_s     = sensor_fusion[i][5];
      
      // Predict vehicle s coordinate using constant speed assumption
      vehicle_s += ((double)prev_size*0.02*vehicle_speed);

      // Check the vehicle which is in the ego lane s value grater than ego vehicle and less than safe distance
      if(vehicle_s > ego_vehicle[4] && (vehicle_s - ego_vehicle[4] < SAFE_DIST))
      {
        adjusted_vel = vehicle_speed;
      }
    }
  }
  return adjusted_vel;
}

// Cost function for speed, i.e vote for fastest lane
vector<double> speedCostForLanes(const vector<double> &ego_vehicle, 
                         const vector<vector<double>> &sensor_fusion,
                         const int prev_size)
{
  // Slowest vehicles on the lanes
  double id_slowest_right;
  double id_slowest_center;
  double id_slowest_left;

  double speed_slowest_right  = TARGET_VEL;
  double speed_slowest_center = TARGET_VEL;
  double speed_slowest_left   = TARGET_VEL; 

  // Cost for velocity for each lane
  vector<double> velocity_cost{0.0, 0.0, 0.0}; // {left_lane, center_lane, right_lane}

  for(int i = 0; i < sensor_fusion.size(); i++)
  {
    double vehicle_d = sensor_fusion[i][6];
    // Determine the lane of vehicle
    if(vehicle_d < 4 && vehicle_d > 0) //Left lane
    {
      double vehicle_id    = sensor_fusion[i][0];
      double vehicle_vx    = sensor_fusion[i][3];
      double vehicle_vy    = sensor_fusion[i][4];
      double vehicle_speed = sqrt(vehicle_vx*vehicle_vx + vehicle_vy*vehicle_vy);
      double vehicle_s     = sensor_fusion[i][5];
      // Predict vehicle s coordinate using constant speed assumption
      vehicle_s += ((double)prev_size*0.02*vehicle_speed);
      // Check the vehicle which is in the ego lane s value grater than ego vehicle
      if(vehicle_s > ego_vehicle[4])
      {
        // Check the vehicle is the slowest one of its lane
        if(vehicle_speed<speed_slowest_left)
        {
          speed_slowest_left = vehicle_speed;
          id_slowest_left    = vehicle_id;
        }
      }
    }
    if(vehicle_d < 8 && vehicle_d > 4) //Center lane
    {
      double vehicle_id    = sensor_fusion[i][0];
      double vehicle_vx    = sensor_fusion[i][3];
      double vehicle_vy    = sensor_fusion[i][4];
      double vehicle_speed = sqrt(vehicle_vx*vehicle_vx + vehicle_vy*vehicle_vy);
      double vehicle_s     = sensor_fusion[i][5];
      // Predict vehicle s coordinate using constant speed assumption
      vehicle_s += ((double)prev_size*0.02*vehicle_speed);
      // Check the vehicle which is in the ego lane s value grater than ego vehicle
      if(vehicle_s > ego_vehicle[4])
      {
        // Check the vehicle is the slowest one of its lane
        if(vehicle_speed<speed_slowest_center)
        {
          speed_slowest_center = vehicle_speed;
          id_slowest_center    = vehicle_id;
        }
      }
    }
    if(vehicle_d < 12 && vehicle_d > 8) //Right lane
    {
      double vehicle_id    = sensor_fusion[i][0];
      double vehicle_vx    = sensor_fusion[i][3];
      double vehicle_vy    = sensor_fusion[i][4];
      double vehicle_speed = sqrt(vehicle_vx*vehicle_vx + vehicle_vy*vehicle_vy);
      double vehicle_s     = sensor_fusion[i][5];
      // Predict vehicle s coordinate using constant speed assumption
      vehicle_s += ((double)prev_size*0.02*vehicle_speed);
      // Check the vehicle which is in the ego lane s value grater than ego vehicle
      if(vehicle_s > ego_vehicle[4])
      {
        // Check the vehicle is the slowest one of its lane
        if(vehicle_speed<speed_slowest_right)
        {
          speed_slowest_right = vehicle_speed;
          id_slowest_right = vehicle_id;
        }
      }
    }

  }
  velocity_cost[0] = VELOCITY_COST_GAIN * (TARGET_VEL - speed_slowest_left);
  velocity_cost[1] = VELOCITY_COST_GAIN * (TARGET_VEL - speed_slowest_center);
  velocity_cost[2] = VELOCITY_COST_GAIN * (TARGET_VEL - speed_slowest_right);
  return velocity_cost;
}

// Cost function for distance, i.e vote for safe lane
vector<double> distCostForLanes(const vector<double> &ego_vehicle, 
                         const vector<vector<double>> &sensor_fusion,
                         const int prev_size)
{
  // Close vehicles on the lanes
  double id_close_right;
  double id_close_center;
  double id_close_left;

  double dist_close_right  = SAFE_DIST;
  double dist_close_center = SAFE_DIST;
  double dist_close_left   = SAFE_DIST; 

  // Cost for distance for each lane
  vector<double> dist_cost{0.0, 0.0, 0.0}; // {left_lane, center_lane, right_lane}

  for(int i = 0; i < sensor_fusion.size(); i++)
  {
    double vehicle_d = sensor_fusion[i][6];
    // Determine the lane of vehicle
    if(vehicle_d < 4 && vehicle_d > 0) //Left lane
    {
      double vehicle_id    = sensor_fusion[i][0];
      double vehicle_vx    = sensor_fusion[i][3];
      double vehicle_vy    = sensor_fusion[i][4];
      double vehicle_speed = sqrt(vehicle_vx*vehicle_vx + vehicle_vy*vehicle_vy);
      double vehicle_s     = sensor_fusion[i][5];
      // Predict vehicle s coordinate using constant speed assumption
      vehicle_s += ((double)prev_size*0.02*vehicle_speed);
      // Check the vehicle which is in the ego lane s value grater than ego vehicle
      if(vehicle_s > ego_vehicle[4] && (vehicle_s - ego_vehicle[4] < SAFE_DIST))
      {
        double dist_left = vehicle_s - ego_vehicle[4];
        // Check the vehicle is the close one of its lane
        if(dist_left<dist_close_left)
        {
          dist_close_left = dist_left;
          id_close_left   = vehicle_id;
        }
      }
    }
    if(vehicle_d < 8 && vehicle_d > 4) //Center lane
    {
      double vehicle_id    = sensor_fusion[i][0];
      double vehicle_vx    = sensor_fusion[i][3];
      double vehicle_vy    = sensor_fusion[i][4];
      double vehicle_speed = sqrt(vehicle_vx*vehicle_vx + vehicle_vy*vehicle_vy);
      double vehicle_s     = sensor_fusion[i][5];
      // Predict vehicle s coordinate using constant speed assumption
      vehicle_s += ((double)prev_size*0.02*vehicle_speed);
      // Check the vehicle which is in the ego lane s value grater than ego vehicle
      if(vehicle_s > ego_vehicle[4] && (vehicle_s - ego_vehicle[4] < SAFE_DIST))
      {
        double dist_center = vehicle_s - ego_vehicle[4];
        // Check the vehicle is the close one of its lane
        if(dist_center<dist_close_center)
        {
          dist_close_center = dist_center;
          id_close_center   = vehicle_id;
        }
      }
    }
    if(vehicle_d < 12 && vehicle_d > 8) //Right lane
    {
      double vehicle_id    = sensor_fusion[i][0];
      double vehicle_vx    = sensor_fusion[i][3];
      double vehicle_vy    = sensor_fusion[i][4];
      double vehicle_speed = sqrt(vehicle_vx*vehicle_vx + vehicle_vy*vehicle_vy);
      double vehicle_s     = sensor_fusion[i][5];
      // Predict vehicle s coordinate using constant speed assumption
      vehicle_s += ((double)prev_size*0.02*vehicle_speed);
      // Check the vehicle which is in the ego lane s value grater than ego vehicle
      if(vehicle_s > ego_vehicle[4] && (vehicle_s - ego_vehicle[4] < SAFE_DIST))
      {
        double dist_right = vehicle_s - ego_vehicle[4];
        // Check the vehicle is the close one of its lane
        if(dist_right<dist_close_right)
        {
          dist_close_right = dist_right;
          id_close_right   = vehicle_id;
        }
      }
    }

  }
  dist_cost[0] = DIST_COST_GAIN * (SAFE_DIST - dist_close_left);
  dist_cost[1] = DIST_COST_GAIN * (SAFE_DIST - dist_close_center);
  dist_cost[2] = DIST_COST_GAIN * (SAFE_DIST - dist_close_right);
  return dist_cost;
}
#endif  // HELPERS_H
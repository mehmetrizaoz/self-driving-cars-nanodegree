//cmake .. && make

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

#define SAFETY_DISTANCE 25
#define _20_MS 0.02
#define MAX_ACCELERATION 0.224
#define LANE_WIDTH 4  
#define LANE_MID 2
#define NUMBER_OF_POINTS 50

// for convenience
using nlohmann::json;
using namespace std;

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

    int lane = 1; // start in lane 1
    double ref_vel = 0; // miles per hour
    double max_ref_vel = 49.0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel,
               &max_ref_vel]
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
//get data from simulator
                  //our car's information
                  double car_x = j[1]["x"];
                  double car_y = j[1]["y"];
                  double car_s = j[1]["s"];
                  double car_d = j[1]["d"];
                  double car_yaw = j[1]["yaw"];
                  double car_speed = j[1]["speed"];
                  auto previous_path_x = j[1]["previous_path_x"];
                  auto previous_path_y = j[1]["previous_path_y"];
                  double end_path_s = j[1]["end_path_s"];
                  double end_path_d = j[1]["end_path_d"];
                  //other cars' in the environment 
                  auto otherCars = j[1]["sensor_fusion"];
                  json msgJson;
                  //update our car's s position 
                  int prev_size = previous_path_x.size();
                  if (prev_size > 0) 
                      car_s = end_path_s;                 
//high level logic start
                  bool slowDown = false;
                  vector<bool> isLaneAvailable = {true, true, true};
                  isLaneAvailable[lane] = false;

                  for (int i = 0; i < otherCars.size(); i++) {                      
                      double vx = otherCars[i][3];
                      double vy = otherCars[i][4];
                      double otherCarSpeed = sqrt(vx*vx+vy*vy);
                      double otherCarS = otherCars[i][5];
                      double otherCarsD = otherCars[i][6];
                      //double otherCarLane = whichLane(otherCarsD);
                      int otherCarLane = 0;
                      while(otherCarsD>=double(LANE_WIDTH)){
                          otherCarLane++;
                          otherCarsD -= double(LANE_WIDTH);
                      }

                      otherCarS += (double) prev_size * _20_MS * otherCarSpeed;          
                      bool inSameLane = (otherCarLane == lane) ? true : false;
                      bool otherCarIsFaster = (otherCarS > car_s) ? true : false;
                      //todo: some more logic is possible
                      if (inSameLane && otherCarIsFaster && otherCarS-car_s < SAFETY_DISTANCE)
                          slowDown = true;
                      if (!inSameLane && otherCarIsFaster && otherCarS-car_s < SAFETY_DISTANCE) 
                          isLaneAvailable[otherCarLane] = false;
                      if (!inSameLane && !otherCarIsFaster && car_s-otherCarS < SAFETY_DISTANCE) 
                          isLaneAvailable[otherCarLane] = false;
                  }

                  if (slowDown) {
                      ref_vel -= MAX_ACCELERATION;
                      for (int i = 0; i < isLaneAvailable.size(); i++)
                          if (isLaneAvailable[i]){//change lane after checking available lanes starting from left lane
                              if(abs(lane-i)<=1){//dont change 2 lanes at the same time
                                 lane = i;
                                 break;
                              }
                          }                      
                  } 
                  else if (ref_vel < max_ref_vel)
                      ref_vel += MAX_ACCELERATION; //speed up
//high level logic end                      
//trajectory generation start
                  //get 2 points from previous path
                  vector<double> ptsx;
                  vector<double> ptsy;
                  double ref_x = car_x; 
                  double ref_y = car_y;
                  double ref_yaw;
                  
                  if (prev_size < 2) { //if no previous path exist make one point artificially using car's velocity vector
                      ref_yaw = deg2rad(car_yaw);
                      double prev_car_x = car_x - cos(car_yaw);
                      double prev_car_y = car_y - sin(car_yaw);
                      ptsx.push_back(prev_car_x);
                      ptsx.push_back(car_x);
                      ptsy.push_back(prev_car_y);
                      ptsy.push_back(car_y);
                  } else { //add previous path directly to points
                      ref_x = previous_path_x[prev_size-1];
                      ref_y = previous_path_y[prev_size-1];
                      double ref_x_prev = previous_path_x[prev_size-2];
                      double ref_y_prev = previous_path_y[prev_size-2];
                      ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
                      ptsx.push_back(ref_x_prev);
                      ptsx.push_back(ref_x);
                      ptsy.push_back(ref_y_prev);
                      ptsy.push_back(ref_y);
                  }
                  //get 3 future points - (transform waypoints that are in frenet coorinate to global cartesian coordinate first)
                  vector<double> next_wp0 = getXY(car_s+30, (LANE_MID+LANE_WIDTH*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                  vector<double> next_wp1 = getXY(car_s+60, (LANE_MID+LANE_WIDTH*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                  vector<double> next_wp2 = getXY(car_s+90, (LANE_MID+LANE_WIDTH*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

                  ptsx.push_back(next_wp0[0]);
                  ptsx.push_back(next_wp1[0]);
                  ptsx.push_back(next_wp2[0]);

                  ptsy.push_back(next_wp0[1]);
                  ptsy.push_back(next_wp1[1]);
                  ptsy.push_back(next_wp2[1]);

                  //convert global cartesian coordinates to cars local coordinates
                  for (int i = 0; i < ptsx.size(); i++) {
                      double shift_x = ptsx[i] - ref_x;//shift
                      double shift_y = ptsy[i] - ref_y;
                      ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
                      ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
                  }
                  //use spline to generate curve
                  tk::spline s;
                  s.set_points(ptsx, ptsy);
                  vector<double> next_x_vals;
                  vector<double> next_y_vals;
                  //add previous coordinates for smooth behavior (to decrease jerk)
                  for (int i = 0; i < previous_path_x.size(); i++) {
                      next_x_vals.push_back(previous_path_x[i]);
                      next_y_vals.push_back(previous_path_y[i]);
                  }
                  //find spline points
                  double target_x = 30.0; //horizon
                  double target_y = s(target_x);
                  double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
                  double x_add_on = 0;
                  //find spline points upto the horizon
                  for (int i = 1; i <= NUMBER_OF_POINTS-previous_path_x.size(); i++) {
                      double N = (target_dist/(_20_MS * ref_vel / 2.24));
                      double x_point = x_add_on + (target_x)/N;
                      double y_point = s(x_point); //give x value and get y value

                      x_add_on = x_point;
                      double x_ref = x_point;
                      double y_ref = y_point;
                      //transform back to global coord. - shifted above and now do rotation
                      x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                      y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
                      x_point += ref_x;
                      y_point += ref_y;

                      next_x_vals.push_back(x_point); //add new points on previous path
                      next_y_vals.push_back(y_point);
                  }
//trajectroy generation end
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
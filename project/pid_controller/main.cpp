/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge, Aaron Brown
 *  Updated: 2026 - Fixed PID integration and State initialization
 **********************************************/

#include <string>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <vector>
#include <fstream>
#include <limits>

#include "json.hpp"
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include "Eigen/QR"

#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "utils.h"
#include "pid_controller.h"

#include <uWS/uWS.h>
#include <math.h>
#include <time.h>

using namespace std;
using json = nlohmann::json;

#define _USE_MATH_DEFINES

string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("{");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

double angle_between_points(double x1, double y1, double x2, double y2) {
  return atan2(y2 - y1, x2 - x1);
}

// Global planners
BehaviorPlannerFSM behavior_planner(
    P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
    P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
    P_MAX_ACCEL, P_STOP_LINE_BUFFER);

MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

bool have_obst = false;
vector<State> obstacles;

void path_planner(vector<double>& x_points, vector<double>& y_points, vector<double>& v_points,
                  double yaw, double velocity, State goal, bool is_junction, string tl_state,
                  vector<vector<double>>& spirals_x, vector<vector<double>>& spirals_y,
                  vector<vector<double>>& spirals_v, vector<int>& best_spirals) {

  State ego_state;
  ego_state.location.x = x_points.back();
  ego_state.location.y = y_points.back();
  ego_state.velocity.x = velocity;

  if (x_points.size() > 1) {
    ego_state.rotation.yaw = angle_between_points(x_points[x_points.size()-2], y_points[y_points.size()-2],
                                                  x_points.back(), y_points.back());
    ego_state.velocity.x = v_points.back();
    if (velocity < 0.01)
      ego_state.rotation.yaw = yaw;
  }

  Maneuver behavior = behavior_planner.get_active_maneuver();
  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);

  if (behavior == STOPPED) {
    int max_points = 20;
    double point_x = x_points.back();
    double point_y = y_points.back();
    while (x_points.size() < max_points) {
      x_points.push_back(point_x);
      y_points.push_back(point_y);
      v_points.push_back(0.0);
    }
    return;
  }

  auto goal_set = motion_planner.generate_offset_goals(goal);
  auto spirals = motion_planner.generate_spirals(ego_state, goal_set);
  auto desired_speed = utils::magnitude(goal.velocity);

  State lead_car_state;

  if (spirals.empty()) {
    cout << "Error: No spirals generated" << endl;
    return;
  }

  for (size_t i = 0; i < spirals.size(); ++i) {
    auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory(
        spirals[i], desired_speed, ego_state, lead_car_state, behavior);

    vector<double> spiral_x, spiral_y, spiral_v;
    for (const auto& point : trajectory) {
      spiral_x.push_back(point.path_point.x);
      spiral_y.push_back(point.path_point.y);
      spiral_v.push_back(point.v);
    }
    spirals_x.push_back(spiral_x);
    spirals_y.push_back(spiral_y);
    spirals_v.push_back(spiral_v);
  }

  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);
  int best_spiral_idx = best_spirals.empty() ? -1 : best_spirals.back();

  size_t index = 0;
  int max_points = 20;
  size_t add_points = (best_spiral_idx >= 0) ? spirals_x[best_spiral_idx].size() : 0;

  while (x_points.size() < max_points && index < add_points) {
    x_points.push_back(spirals_x[best_spiral_idx][index]);
    y_points.push_back(spirals_y[best_spiral_idx][index]);
    v_points.push_back(spirals_v[best_spiral_idx][index]);
    ++index;
  }
}

void set_obst(const vector<double>& x_points, const vector<double>& y_points,
              vector<State>& obstacles, bool& obst_flag) {
  for (size_t i = 0; i < x_points.size(); ++i) {
    State obstacle;
    obstacle.location.x = x_points[i];
    obstacle.location.y = y_points[i];
    obstacles.push_back(obstacle);
  }
  obst_flag = true;
}

int main() {
  cout << "Starting server..." << endl;
  uWS::Hub h;

  double new_delta_time = 0.0;
  int frame = 0;

  // PID Controllers with good starting gains
  PID pid_steer;
  pid_steer.Init(5.0, 0.01, 2.0, 1.0, -1.0);   // Kp, Ki, Kd, max, min

  PID pid_throttle;
  pid_throttle.Init(0.65, 0.0, 0.08, 1.0, -1.0);

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    string s = hasData(string(data, length));
    if (s.empty()) return;

    auto data_json = json::parse(s);

    vector<double> x_points = data_json["traj_x"];
    vector<double> y_points = data_json["traj_y"];
    vector<double> v_points = data_json["traj_v"];
    double yaw       = data_json["yaw"];
    double velocity  = data_json["velocity"];
    double waypoint_x = data_json["waypoint_x"];
    double waypoint_y = data_json["waypoint_y"];
    double waypoint_t = data_json["waypoint_t"];
    bool is_junction  = data_json["waypoint_j"];
    string tl_state   = data_json["tl_state"];

    double x_position = data_json["location_x"];
    double y_position = data_json["location_y"];

    if (!have_obst) {
      vector<double> x_obst = data_json["obst_x"];
      vector<double> y_obst = data_json["obst_y"];
      set_obst(x_obst, y_obst, obstacles, have_obst);
    }

    // Create goal state safely
    State goal = {};
    goal.location.x = waypoint_x;
    goal.location.y = waypoint_y;
    goal.rotation.yaw = waypoint_t;

    vector<vector<double>> spirals_x, spirals_y, spirals_v;
    vector<int> best_spirals;

    path_planner(x_points, y_points, v_points, yaw, velocity, goal,
                 is_junction, tl_state, spirals_x, spirals_y, spirals_v, best_spirals);

    // Compute delta time
    static time_t prev_timer = time(nullptr);
    time_t timer;
    time(&timer);
    new_delta_time = difftime(timer, prev_timer);
    prev_timer = timer;

    ////////////////////////////////////////
    // Steering Control
    ////////////////////////////////////////
    pid_steer.UpdateDeltaTime(new_delta_time);

    // Find closest point on trajectory
    size_t closest_idx = 0;
    double closest_dist = numeric_limits<double>::max();
    for (size_t j = 0; j < x_points.size(); ++j) {
      double dx = x_points[j] - x_position;
      double dy = y_points[j] - y_position;
      double dist = sqrt(dx*dx + dy*dy);
      if (dist < closest_dist) {
        closest_dist = dist;
        closest_idx = j;
      }
    }

    size_t target_idx = min(closest_idx + 1, x_points.size() - 1);

    double desired_yaw = yaw;
    if (target_idx != closest_idx) {
      desired_yaw = angle_between_points(x_points[closest_idx], y_points[closest_idx],
                                         x_points[target_idx], y_points[target_idx]);
    }

    double error_steer = atan2(sin(desired_yaw - yaw), cos(desired_yaw - yaw));

    pid_steer.UpdateError(error_steer);
    double steer_output = pid_steer.TotalError();

    ////////////////////////////////////////
    // Throttle Control
    ////////////////////////////////////////
    pid_throttle.UpdateDeltaTime(new_delta_time);

    double target_speed = v_points[min(target_idx, v_points.size() - 1)];
    double error_throttle = target_speed - velocity;

    pid_throttle.UpdateError(error_throttle);
    double throttle = pid_throttle.TotalError();

    double throttle_output = max(0.0, throttle);
    double brake_output = (throttle < 0.0) ? -throttle : 0.0;

    // Launch assist
    if (target_speed > 0.5 && velocity < 0.5 && throttle_output < 0.2) {
      throttle_output = 0.2;
      brake_output = 0.0;
    }

    // Logging
    ofstream fs("steer_pid_data.txt", ios::app);
    fs << frame << " " << error_steer << " " << steer_output << endl;
    fs.close();

    ofstream ft("throttle_pid_data.txt", ios::app);
    ft << frame << " " << error_throttle << " " << brake_output << " " << throttle_output << endl;
    ft.close();

    // Send control to simulator
    json msgJson;
    msgJson["brake"] = brake_output;
    msgJson["throttle"] = throttle_output;
    msgJson["steer"] = steer_output;

    msgJson["trajectory_x"] = x_points;
    msgJson["trajectory_y"] = y_points;
    msgJson["trajectory_v"] = v_points;
    msgJson["spirals_x"] = spirals_x;
    msgJson["spirals_y"] = spirals_y;
    msgJson["spirals_v"] = spirals_v;
    msgJson["spiral_idx"] = best_spirals;
    msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();
    msgJson["update_point_thresh"] = 16;

    auto msg = msgJson.dump();
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

    frame++;
  });

  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    cout << "Connected to simulator!" << endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    cout << "Disconnected" << endl;
  });

  int port = 4567;
  if (h.listen("0.0.0.0", port)) {
    cout << "Listening on port " << port << endl;
    h.run();
  } else {
    cerr << "Failed to listen on port " << port << endl;
    return -1;
  }

  return 0;
}
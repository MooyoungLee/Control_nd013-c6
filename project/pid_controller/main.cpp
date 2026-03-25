/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 				Aaron Brown
 **********************************************/

/**
 * @file main.cpp
 **/

#include <string>
#include <array>
#include <algorithm>
#include <cctype>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <iostream>
#include <fstream>
#include <typeinfo>

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

#include <limits>
#include <iostream>
#include <fstream>
#include "local_uws.h"
#include <math.h>
#include <vector>
#include <cmath>
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
    }
    else if (b1 != string::npos && b2 != string::npos) {
      return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double angle_between_points(double x1, double y1, double x2, double y2){
  return atan2(y2-y1, x2-x1);
}

string to_lower_copy(string value) {
  transform(value.begin(), value.end(), value.begin(),
            [](unsigned char c) { return static_cast<char>(tolower(c)); });
  return value;
}

string get_env_string(const char* name, const string& default_value) {
  const char* raw_value = getenv(name);
  if (raw_value == nullptr) {
    return default_value;
  }
  return string(raw_value);
}

bool get_env_bool(const char* name, bool default_value) {
  const string value = to_lower_copy(get_env_string(name, default_value ? "1" : "0"));
  if (value == "1" || value == "true" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "0" || value == "false" || value == "no" || value == "off") {
    return false;
  }
  return default_value;
}

int get_env_int(const char* name, int default_value) {
  const char* raw_value = getenv(name);
  if (raw_value == nullptr) {
    return default_value;
  }
  try {
    return stoi(raw_value);
  } catch (const exception&) {
    return default_value;
  }
}

double get_env_double(const char* name, double default_value) {
  const char* raw_value = getenv(name);
  if (raw_value == nullptr) {
    return default_value;
  }
  try {
    return stod(raw_value);
  } catch (const exception&) {
    return default_value;
  }
}

vector<double> get_env_triplet(const char* name, const vector<double>& default_value) {
  const char* raw_value = getenv(name);
  if (raw_value == nullptr) {
    return default_value;
  }

  vector<double> values;
  string token;
  stringstream stream(raw_value);
  while (getline(stream, token, ',')) {
    try {
      values.push_back(stod(token));
    } catch (const exception&) {
      return default_value;
    }
  }

  if (values.size() != 3) {
    return default_value;
  }
  return values;
}

struct AutoTuneSettings {
  bool enabled;
  bool tune_steer;
  bool tune_throttle;
  bool sequential;
  double tolerance;
  int settle_frames;
  int eval_frames;
  vector<double> steer_dp;
  vector<double> throttle_dp;
};

AutoTuneSettings load_auto_tune_settings() {
  AutoTuneSettings settings;
  settings.enabled = get_env_bool("PID_AUTO_TUNE", false);
  settings.tune_steer = false;
  settings.tune_throttle = false;
  settings.sequential = true;
  settings.tolerance = get_env_double("PID_AUTO_TUNE_TOLERANCE", 0.005);
  settings.settle_frames = get_env_int("PID_AUTO_TUNE_SETTLE_FRAMES", 80);
  settings.eval_frames = get_env_int("PID_AUTO_TUNE_EVAL_FRAMES", 200);
  settings.steer_dp = get_env_triplet("PID_AUTO_TUNE_STEER_DP", {0.05, 0.0002, 0.01});
  settings.throttle_dp = get_env_triplet("PID_AUTO_TUNE_THROTTLE_DP", {0.03, 0.0001, 0.002});

  const string mode = to_lower_copy(get_env_string("PID_AUTO_TUNE_MODE", "sequential"));
  if (!settings.enabled || mode == "off") {
    settings.enabled = false;
    settings.sequential = true;
    settings.steer_dp = {0.0, 0.0, 0.0};
    settings.throttle_dp = {0.0, 0.0, 0.0};
    return settings;
  }

  if (mode == "steer") {
    settings.tune_steer = true;
  } else if (mode == "throttle") {
    settings.tune_throttle = true;
  } else if (mode == "both") {
    settings.tune_steer = true;
    settings.tune_throttle = true;
    settings.sequential = false;
  } else {
    settings.tune_steer = true;
    settings.tune_throttle = true;
    settings.sequential = true;
  }

  settings.tune_steer = get_env_bool("PID_AUTO_TUNE_STEER", settings.tune_steer);
  settings.tune_throttle = get_env_bool("PID_AUTO_TUNE_THROTTLE", settings.tune_throttle);

  if (!settings.tune_steer) {
    settings.steer_dp = {0.0, 0.0, 0.0};
  }
  if (!settings.tune_throttle) {
    settings.throttle_dp = {0.0, 0.0, 0.0};
  }
  if (!settings.tune_steer && !settings.tune_throttle) {
    settings.enabled = false;
  }
  return settings;
}

BehaviorPlannerFSM behavior_planner(
      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);

// Decalre and initialized the Motion Planner and all its class requirements
MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

bool have_obst = false;
vector<State> obstacles;

void path_planner(vector<double>& x_points, vector<double>& y_points, vector<double>& v_points,
                  double ego_x, double ego_y, double yaw, double velocity, State goal,
                  bool is_junction, string tl_state,
                  vector< vector<double> >& spirals_x, vector< vector<double> >& spirals_y,
                  vector< vector<double> >& spirals_v, vector<int>& best_spirals){

  State ego_state;

  ego_state.location.x = ego_x;
  ego_state.location.y = ego_y;
  ego_state.velocity.x = velocity;
  ego_state.rotation.yaw = yaw;

  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);

  Maneuver behavior = behavior_planner.get_active_maneuver();  // avoid obstable helpers

  if(behavior == STOPPED){

  	int max_points = 20;
  	double point_x = ego_x;
  	double point_y = ego_y;
    x_points.clear();
    y_points.clear();
    v_points.clear();
    x_points.push_back(point_x);
    y_points.push_back(point_y);
    v_points.push_back(0.0);
   	while( x_points.size() < max_points ){
   	  x_points.push_back(point_x);
   	  y_points.push_back(point_y);
  	  v_points.push_back(0);

  	}
  	return;
  }

  auto goal_set = motion_planner.generate_offset_goals(goal);

  auto spirals = motion_planner.generate_spirals(ego_state, goal_set);

  auto desired_speed = utils::magnitude(goal.velocity);

  State lead_car_state;  // = to the vehicle ahead...

  if(spirals.size() == 0){
  	cout << "Error: No spirals generated " << endl;
  	return;
  }

  for(int i = 0; i < spirals.size(); i++){

    auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory( spirals[i], desired_speed, ego_state,
                                                                                    lead_car_state, behavior);

    vector<double> spiral_x;
    vector<double> spiral_y;
    vector<double> spiral_v;
    for(int j = 0; j < trajectory.size(); j++){
      double point_x = trajectory[j].path_point.x;
      double point_y = trajectory[j].path_point.y;
      double velocity = trajectory[j].v;
      spiral_x.push_back(point_x);
      spiral_y.push_back(point_y);
      spiral_v.push_back(velocity);
    }

    spirals_x.push_back(spiral_x);
    spirals_y.push_back(spiral_y);
    spirals_v.push_back(spiral_v);

  }

  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);
  int best_spiral_idx = -1;

  if(best_spirals.size() > 0)
  	best_spiral_idx = best_spirals[best_spirals.size()-1];

  if (best_spiral_idx < 0) {
    x_points.clear();
    y_points.clear();
    v_points.clear();
    x_points.push_back(ego_x);
    y_points.push_back(ego_y);
    v_points.push_back(0.0);
    return;
  }

  int index = 0;
  int max_points = 20;
  x_points.clear();
  y_points.clear();
  v_points.clear();
  x_points.push_back(ego_x);
  y_points.push_back(ego_y);
  v_points.push_back(velocity);
  int add_points = spirals_x[best_spiral_idx].size();
  while( x_points.size() < max_points && index < add_points ){
    double point_x = spirals_x[best_spiral_idx][index];
    double point_y = spirals_y[best_spiral_idx][index];
    double velocity = spirals_v[best_spiral_idx][index];
    index++;
    x_points.push_back(point_x);
    y_points.push_back(point_y);
    v_points.push_back(velocity);
  }


}

void set_obst(vector<double> x_points, vector<double> y_points, vector<double> yaw_points, vector<State>& obstacles, bool& obst_flag){

  obstacles.clear(); // avoid obstacle helpers

	for (size_t i = 0; i < x_points.size(); i++) {
		State obstacle;
		obstacle.location.x = x_points[i];
		obstacle.location.y = y_points[i];
    obstacle.rotation.yaw = 0.0;   // avoid obstacle helpers
    if (i < yaw_points.size()) {
      obstacle.rotation.yaw = yaw_points[i];
    }
		obstacles.push_back(obstacle);
	}
	obst_flag = true;
}

int main ()
{
  cout << "starting server" << endl;
  uWS::Hub h;

  double new_delta_time;
  double prev_sim_time = 0.0;
  int i = 0;

  fstream file_steer;
  file_steer.open("steer_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_steer.close();
  fstream file_throttle;
  file_throttle.open("throttle_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_throttle.close();

  const AutoTuneSettings auto_tune = load_auto_tune_settings();

  // initialize pid steer
  /**
  * TODO (Step 1): create pid (pid_steer) for steer command and initialize values
  **/
  PID pid_steer = PID();
  pid_steer.Init(0.8, 0.0005, 0.08, 1.0, -1.0);
  Twiddle twiddle_steer;
  twiddle_steer.Init({pid_steer.Kp, pid_steer.Ki, pid_steer.Kd},
                     auto_tune.tune_steer ? auto_tune.steer_dp
                                          : vector<double>{0.0, 0.0, 0.0},
                     auto_tune.tolerance,
                     auto_tune.settle_frames,
                     auto_tune.eval_frames);

  // initialize pid throttle
  /**
  * TODO (Step 1): create pid (pid_throttle) for throttle command and initialize values
  **/
  PID pid_throttle = PID();
  pid_throttle.Init(0.2, 0.0, 0.004, 1.0, -1.0);
  Twiddle twiddle_throttle;
  twiddle_throttle.Init({pid_throttle.Kp, pid_throttle.Ki, pid_throttle.Kd},
                        auto_tune.tune_throttle ? auto_tune.throttle_dp
                                                : vector<double>{0.0, 0.0, 0.0},
                        auto_tune.tolerance,
                        auto_tune.settle_frames,
                        auto_tune.eval_frames);
  bool throttle_tuning_announced = false;

  if (auto_tune.enabled) {
    cout << "PID auto-tune enabled. mode="
         << (auto_tune.sequential ? "sequential" : "parallel")
         << ", steer=" << auto_tune.tune_steer
         << ", throttle=" << auto_tune.tune_throttle
         << ", settle_frames=" << auto_tune.settle_frames
         << ", eval_frames=" << auto_tune.eval_frames
         << ", tolerance=" << auto_tune.tolerance
         << endl;
  }

  h.onMessage([&pid_steer, &pid_throttle, &twiddle_steer, &twiddle_throttle,
               &auto_tune, &throttle_tuning_announced,
               &new_delta_time, &prev_sim_time, &i](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
        auto s = hasData(data);

        if (s != "") {

          auto data = json::parse(s);

          // create file to save values
          fstream file_steer;
          file_steer.open("steer_pid_data.txt");
          fstream file_throttle;
          file_throttle.open("throttle_pid_data.txt");

          vector<double> x_points = data["traj_x"];
          vector<double> y_points = data["traj_y"];
          vector<double> v_points = data["traj_v"];
          double yaw = data["yaw"];
          double velocity = data["velocity"];
          double sim_time = data["time"];
          double waypoint_x = data["waypoint_x"];
          double waypoint_y = data["waypoint_y"];
          double waypoint_t = data["waypoint_t"];
          bool is_junction = data["waypoint_j"];
          string tl_state = data["tl_state"];

          double x_position = data["location_x"];
          double y_position = data["location_y"];
          double z_position = data["location_z"];

          // if(!have_obst){  // avoid obstacle helper
          vector<double> x_obst = data["obst_x"];
          vector<double> y_obst = data["obst_y"];
          vector<double> yaw_obst;
          if (data.count("obst_yaw") > 0) {
            yaw_obst = data["obst_yaw"].get<vector<double>>();
          }
          set_obst(x_obst, y_obst, yaw_obst, obstacles, have_obst);
          // }  // avoid obstacle helper

          State goal;
          goal.location.x = waypoint_x;
          goal.location.y = waypoint_y;
          goal.rotation.yaw = waypoint_t;

          vector< vector<double> > spirals_x;
          vector< vector<double> > spirals_y;
          vector< vector<double> > spirals_v;
          vector<int> best_spirals;

          path_planner(x_points, y_points, v_points, x_position, y_position, yaw, velocity,
                       goal, is_junction, tl_state, spirals_x, spirals_y, spirals_v, best_spirals);

          // Use simulator time so PID and Twiddle update every frame.
          new_delta_time = (i == 0) ? 0.0 : max(sim_time - prev_sim_time, 0.0);
          prev_sim_time = sim_time;

          ////////////////////////////////////////
          // Steering control
          ////////////////////////////////////////

          /**
          * TODO (step 3): uncomment these lines
          **/
          // Update the delta time with the previous command
          pid_steer.UpdateDeltaTime(new_delta_time);

          // Compute steer error
          double error_steer;


          double steer_output = 0.0;

          /**
          * TODO (step 3): compute the steer error (error_steer) from the position and the desired trajectory
          **/
          int closest_idx = 0;
          double closest_dist = numeric_limits<double>::max();
          for (int j = 0; j < x_points.size(); ++j) {
            double dx = x_points[j] - x_position;
            double dy = y_points[j] - y_position;
            double dist = sqrt(dx * dx + dy * dy);
            if (dist < closest_dist) {
              closest_dist = dist;
              closest_idx = j;
            }
          }
          int target_idx = closest_idx;
          const double lookahead_distance = 3.0;
          const double heading_x = cos(yaw);
          const double heading_y = sin(yaw);
          for (int j = closest_idx + 1; j < static_cast<int>(x_points.size()); ++j) {
            const double dx = x_points[j] - x_position;
            const double dy = y_points[j] - y_position;
            const double forward_projection = dx * heading_x + dy * heading_y;
            const double dist = sqrt(dx * dx + dy * dy);
            if (forward_projection > 0.0 && dist >= lookahead_distance) {
              target_idx = j;
              break;
            }
          }
          if (target_idx == closest_idx && closest_idx + 1 < static_cast<int>(x_points.size())) {
            target_idx = closest_idx + 1;
          }
          double desired_yaw = yaw;
          if (target_idx != closest_idx) {
            desired_yaw = angle_between_points(x_position, y_position,
                                               x_points[target_idx], y_points[target_idx]);
          }
          error_steer = atan2(sin(desired_yaw - yaw), cos(desired_yaw - yaw));

          /**
          * TODO (step 3): uncomment these lines
          **/
          // Compute control to apply
          pid_steer.UpdateError(error_steer);
          
          steer_output = pid_steer.TotalError();
          twiddle_steer.Update(error_steer, pid_steer);

          // Save data
          file_steer.seekg(std::ios::beg);
          for(int j=0; j < i - 1; ++j) {
              file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          }
          file_steer  << i ;
          file_steer  << " " << error_steer;
          file_steer  << " " << steer_output << endl;

          ////////////////////////////////////////
          // Throttle control
          ////////////////////////////////////////

          /**
          * TODO (step 2): uncomment these lines
          **/
          // Update the delta time with the previous command
          pid_throttle.UpdateDeltaTime(new_delta_time);

          // Compute error of speed
          double error_throttle;
          /**
          * TODO (step 2): compute the throttle error (error_throttle) from the position and the desired speed
          **/
          // modify the following line for step 2
          const double target_speed = v_points[min(target_idx, static_cast<int>(v_points.size())-1)];
          error_throttle = target_speed - velocity;

          double throttle_output = 0.0;
          double brake_output = 0.0;

          /**
          * TODO (step 2): uncomment these lines
          **/
          // Compute control to apply
          pid_throttle.UpdateError(error_throttle);
          double throttle = pid_throttle.TotalError();

          // Adapt the negative throttle to break
          if (throttle > 0.0) {
            throttle_output = throttle;
            brake_output = 0;
          } else {
            throttle_output = 0;
            brake_output = -throttle;
          }

          // Help overcome launch dead-zone when target speed is non-zero.
          if (target_speed > 0.5 && velocity < 0.5 && throttle_output < 0.2){
            throttle_output = 0.2;
            brake_output = 0.0;
          }

          const bool throttle_tuning_active =
              twiddle_throttle.IsEnabled() &&
              (!auto_tune.sequential || !auto_tune.tune_steer || twiddle_steer.IsFinished());
          if (auto_tune.sequential &&
              auto_tune.tune_steer &&
              auto_tune.tune_throttle &&
              twiddle_steer.IsFinished() &&
              throttle_tuning_active &&
              !throttle_tuning_announced) {
            cout << "Steering tuning finished. Starting throttle tuning." << endl;
            throttle_tuning_announced = true;
          }
          if (throttle_tuning_active) {
            twiddle_throttle.Update(error_throttle, pid_throttle);
          }

          // debug
          std::cout << "steer_pid: [" << pid_steer.Kp
          << ", " << pid_steer.Ki
          << ", " << pid_steer.Kd
          << "], twiddle_active: " << twiddle_steer.IsEnabled()
          << ", best_steer_error: " << twiddle_steer.GetBestError()
          << ", throttle_pid: [" << pid_throttle.Kp
          << ", " << pid_throttle.Ki
          << ", " << pid_throttle.Kd
          << "], throttle_twiddle_active: " << throttle_tuning_active
          << ", best_throttle_error: " << twiddle_throttle.GetBestError()
          << ", steer_output: " << steer_output
          << ", error_steer: " << error_steer
          << ", target_speed: " << target_speed
          << ", velocity: " << velocity
          << ", error_throttle: " << error_throttle
          << ", throttle_output: " << throttle_output
          << ", brake_output: " << brake_output
          << std:: endl;

          // Save data
          file_throttle.seekg(std::ios::beg);
          for(int j=0; j < i - 1; ++j){
              file_throttle.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
          }
          file_throttle  << i ;
          file_throttle  << " " << error_throttle;
          file_throttle  << " " << brake_output;
          file_throttle  << " " << throttle_output << endl;


          // Send control
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

          //  min point threshold before doing the update
          // for high update rate use 19 for slow update rate use 4
          msgJson["update_point_thresh"] = 16;

          auto msg = msgJson.dump();

          i = i + 1;
          file_steer.close();
          file_throttle.close();

      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

    }

  });


  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
      cout << "Connected!!!" << endl;
    });


  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
      ws.close();
      cout << "Disconnected" << endl;
    });

  int port = 4567;
  if (h.listen("0.0.0.0", port))
    {
      cout << "Listening to port " << port << endl;
      h.run();
    }
  else
    {
      cerr << "Failed to listen to port" << endl;
      return -1;
    }


}

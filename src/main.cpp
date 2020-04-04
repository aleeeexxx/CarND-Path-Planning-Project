#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
// #include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "trajectory.h"
#include "behaviorplanner.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

void print(std::vector<double> const &input)
{
	for (auto const& i: input) {
		std::cout << i << " ";
	}
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




	h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
		&map_waypoints_dx, &map_waypoints_dy]
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

					// if previous size is almost empty, use the car as starting reference
					int prev_size = previous_path_x.size();
					// initialize ego car
					if (prev_size > 0) {
						car_s = end_path_s;
					}

					// Initialize ego car 
					Vehicle ego(999);
                  
					ego.update_position(car_s, car_d);
					ego.x = car_x;
					ego.y = car_y;
					ego.yaw = car_yaw;
					ego.update_speed(car_speed);

					// Update otherCars
					vector<Vehicle> otherCars;

					for (int i = 0; i < sensor_fusion.size(); i++) {

						int id = sensor_fusion[i][0];
						double s = sensor_fusion[i][5];
						double d = sensor_fusion[i][6];
						double vx = sensor_fusion[i][3];
						double vy = sensor_fusion[i][4];

						Vehicle car(id);
						car.update_position(s, d);
						car.update_speed(sqrt(vx * vx + vy * vy));
						otherCars.emplace_back(car);
					}
                  




					// Decide behavior according to stats of ego and otherCars
					BehaviorPlanner planner;
					BehaviorType turn_behavior = planner.turn_planner(ego, otherCars);
					GasPedalType gas_behavior = planner.speed_planner(ego, otherCars);
					//define the acutal (x,y) points we will use for the planner
                    //vector<double> next_x_vals;
                    //vector<double> next_y_vals;
                  
                   	 
					// get_trajectory(ego, turn_behavior, gas_behavior, previous_path_x, previous_path_y, prev_size, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                 	 // Print for debugging
                                // Print for debugging
                    cout << "---------------------------------" << endl;
                    cout << "STATE: s, d --- x, y --- v:" << endl;
                    cout << car_s << " , "
                    << car_d << " --- "
                    << car_x << " , "
                    << car_y  << " --- "
                    << car_speed << ":" << endl;
                  	

                    // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
                    vector<double> ptsx;
                    vector<double> ptsy;


                    // Kineamatics 
                    double ref_v = ego.v;
                    if (gas_behavior == GasPedalType::DEA) {
                      ref_v -= 0.9;
                    }
                    else if (gas_behavior == GasPedalType::ACC) {
                      ref_v += 0.9;
                    }
                  // Print for debugging
                    cout << "ref_v : "<< ref_v << endl;

                    // reference x, y, yaw states
                    double ref_x = ego.x;
                    double ref_y = ego.y;
                    double ref_yaw = deg2rad(ego.yaw);
                    int lane;
                    if (turn_behavior == BehaviorType::LCL) {
                        lane = ego.lane - 1;
                    } else if (turn_behavior == BehaviorType::KL) {
                        lane = ego.lane;
                    }else if (turn_behavior == BehaviorType::LCR) {
                        lane = ego.lane + 1;
                    }
                    if (prev_size < 2) {
                        double prev_car_x = ego.x - cos(ref_yaw);
                        double prev_car_y = ego.y - sin(ref_yaw);

                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(ego.x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(ego.y);
                    }
                    // Use the previous path's end point as starting reference
                    else {
                        ref_x = previous_path_x[prev_size - 1];
                        ref_y = previous_path_y[prev_size - 1];

                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                        // Use two points that make the path tangent to the previous path's end point
                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);


                    }

                    // In Frenet add evenly 30m spaced points ahead of the starting reference
                    vector<double> next_wp0 = getXY(ego.s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp1 = getXY(ego.s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp2 = getXY(ego.s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);

                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);

                    for (int i = 0; i < ptsx.size(); i++) {

                        //shift car reference angle to 0 degrees
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;

                        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
                        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
                        std::cout << ptsx[i] << " ";
                    }

                    //create a spline
                    tk::spline s;

                    //set (x,y) points to the spline
                    s.set_points(ptsx, ptsy);

                    //define the acutal (x,y) points we will use for the planner
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;


                    //start with all of the previous path ponts form last time
                    for (int i = 0; i < previous_path_x.size(); i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    // calculate how to break up spline points 
                    double target_x = 30.0;
                    double target_y = s(target_x);
                    double target_dist = sqrt(target_x*target_x + target_y * target_y);

                    double x_add_on = 0;



                  // fill up the rest of our path planner after filling it with previous points, here we will always output 50 points


                    for (int i = 0; i <= 50 - previous_path_x.size(); i++) {
                        double N = target_dist / (0.02 * ref_v / 2.24);
                        double x_point = x_add_on + target_x / N;
                        double y_point = s(x_point);


                        x_add_on = x_point;

                        double x_ref = x_point;
                        double y_ref = y_point;

                        // rotate back to normal after rotating it eariler
                        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

                        x_point += ref_x;
                        y_point += ref_y;



                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
                    }
					//print(next_x_vals);
                    //cout << "get_trajectory done" << endl;
                   


					//

					



					json msgJson;
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}  // end "telemetry" if
			}
			else {
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
	}
	else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}

	h.run();
}
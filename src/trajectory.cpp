#include "trajectory.h"

void get_trajectory(Vehicle &ego, const BehaviorType turn_behavior, const GasPedalType gas_behavior, const vector<double>& previous_path_x, const vector<double>& previous_path_y, int prev_size, 
                    vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y) {


	// Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
	vector<double> ptsx;
	vector<double> ptsy;


    // Kineamatics 
	double ref_v = 49;
    if (gas_behavior == GasPedalType::DEA) {
      ref_v -= 0.224;
    }
    else if (gas_behavior == GasPedalType::ACC) {
      ref_v += 0.224;
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
}

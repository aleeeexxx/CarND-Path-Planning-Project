#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <vector>
#include <iostream>
#include "helpers.h"
#include "behaviorplanner.h"
#include "vehicle.h"
#include "spline.h"




void get_trajectory(Vehicle &ego, const BehaviorType turn_behavior, const GasPedalType gas_behavior, const vector<double>& previous_path_x, const vector<double>& previous_path_y, int prev_size, 
                    vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y);


#endif //TRAJECTORY_H_
#ifndef BEHAVIORPLANNER_H_
#define BEHAVIORPLANNER_H_

#include <vector>
#include <iostream>
#include <math.h>
#include <functional>

#include <map>
#include <string>

#include "vehicle.h"
// #include "helpers.h"


class BehaviorPlanner {

public:
	BehaviorPlanner(); 
    double gaps[6] = {100.0, 100.0,100.0,100.0,100.0,100.0};
    double front_v[3] = {50.0,50.0,50.0,};
    double cost[3] = {100000,100000,100000};
	BehaviorType turn_planner(Vehicle& ego, std::vector<Vehicle>& otherCars);
	void get_gaps(const Vehicle& ego, const std::vector<Vehicle>& otherCars);
	GasPedalType speed_planner(Vehicle& ego, std::vector<Vehicle>& otherCars);
 
};


#endif //BEHAVIORPLANNER_H_
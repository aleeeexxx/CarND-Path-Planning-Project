#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "helpers.h"

using std::map;
using std::string;
using std::vector;

class Vehicle {
	public:
		// Contributes
		int id;
		int lane;
		double s;
		double d;
		double v;
		double x;
		double y;
		double yaw;



		// Constructors
		Vehicle(const int id);


		// Vehicle functions
		void update_position(const double s, const double d);
        int convert_d_to_lane(const double d);
		void update_speed(const double v);
};

#endif  // VEHICLE_H
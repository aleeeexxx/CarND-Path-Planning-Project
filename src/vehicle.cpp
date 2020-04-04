#include "vehicle.h"


using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle(const int i){
	this->id = i;
}

void Vehicle::update_position(const double s, const double d) {
  this->s = s;
  this->d = d;
  this->lane = this->convert_d_to_lane(this->d);
}

void Vehicle::update_speed(const double v) {
	this->v = v;
}

int Vehicle::convert_d_to_lane(const double d) {
  int lane;
  if (d > 0.0 && d < 4.0) {
    lane = 0;
  } else if (d > 4.0 && d < 8.0) {
    lane = 1;
  } else if (d > 8.0 && d < 12.0) {
    lane = 2;
  }
  return lane;
}



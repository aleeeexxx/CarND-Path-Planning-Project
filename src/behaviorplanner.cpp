#include "behaviorplanner.h"

using std::string;
using std::vector;

// Constructor
BehaviorPlanner::BehaviorPlanner() {}



double super_big_num = 100000;
BehaviorType BehaviorPlanner::turn_planner(Vehicle& ego, std::vector<Vehicle>& otherCars) {
	/*  | 0 | 1 | 2 |
		| - |ego| - |
		| 3 | 4 | 5 |  get cloest vehicles in 6 area respectively
	*/
	

    this->get_gaps(ego, otherCars);

	for (int i = 0; i < 3; i++) {
      	std::cout<< "front_v : " << this->front_v[i]<< std::endl;
        std::cout<< "gaps : " << this->gaps[i]<< std::endl;
		this->cost[i] = (100 - this->front_v[i]) * 8000 / (50 * this->gaps[i] );
		// punish lane change action
		if (i != 1) {
			this->cost[i] *= 1.2;
		}
        // punish out lane action 
      	if (ego.lane + i > 3 || ego.lane + i < 1) {
          this->cost[i] = super_big_num;
        }
      	// punish small gap lane change action 
      	if (gaps[i]<20 || gaps[i+3]<20) {
          this->cost[i] = super_big_num;
        }
      
        std::cout<< "cost : "<<this->cost[i]<< std::endl;
	}
	int best_index = 1;
	for (int i = 0; i < 3; i++) {
		if (this->cost[i] < this->cost[best_index]) {
			best_index = i;
           std::cout<< "best_index : "<<best_index<< std::endl;
        }

      
	}

	// if gap is larger than 60m, keep lane
	if (this->gaps[1] > 60 ) {
		return BehaviorType::KL;
	}
	// if gap is larger less than 60m and fornt car's velocity is less than ego, consider lane change
	else {
		// shift to left if left front car's veloctiy is larger than ego's, and gaps is larger than 30,
		if (best_index == 0) {
			return BehaviorType::LCL;
		}
		else if (best_index == 1) {
			return BehaviorType::KL;
		}
		else if (best_index == 2) {
			return BehaviorType::LCR;
		}
	}
}

GasPedalType BehaviorPlanner::speed_planner(Vehicle& ego, std::vector<Vehicle>& otherCars) {
    this->get_gaps(ego, otherCars);
    // gap less than 30m, deaccelerate

    std::cout<<"gaps[1] : "<<this->gaps[1]<<std::endl;
	if (ego.v > 48 || gaps[1] < 30) {
      return GasPedalType::DEA;
    } else if (ego.v > 45) {
      return GasPedalType::KEEP;
    } else {
      return GasPedalType::ACC;
    }
  
}

// Here we have provided two possible suggestions for cost functions, but feel 
//   free to use your own! The weighted cost over all cost functions is computed
//   in calculate_cost. The data from get_helper_data will be very useful in 
//   your implementation of the cost functions below. Please see get_helper_data
//   for details on how the helper data is computed.
void BehaviorPlanner::get_gaps(const Vehicle& ego, const std::vector<Vehicle>& otherCars) {
    double gap;
	//this->gaps[6] = { 100.0,100.0,100.0,100.0,100.0,100.0 };  // 100 m
	//this->front_v[3] = { 55.0, 55.0, 55.0 }; // 55 mph
	
	for (auto &otherCar : otherCars) {

		/*  | 0 | 1 | 2 |
			| - |ego| - |
			| 3 | 4 | 5 |  get cloest vehicles in 6 area respectively
		*/

		// front cars
		if (otherCar.s > ego.s) {
			// area 0 
			if (otherCar.lane < ego.lane) {
				gap = otherCar.s - ego.s;
					if (gap < gaps[0]) {
						this->gaps[0] = gap;
						this->front_v[0] = otherCar.v;
					}
			}
			// area1
			else if (otherCar.lane == ego.lane) {
				gap = otherCar.s - ego.s;
					if (gap < gaps[1]) {
						this->gaps[1] = gap;
						this->front_v[1] = otherCar.v;
                        std::cout<< "gaps[1] in get_gaps:"<<this->gaps[1]<<std::endl;
					}
			}
			// area2
			else if (otherCar.lane > ego.lane) {
				gap = otherCar.s - ego.s;
					if (gap < gaps[2]) {
						this->gaps[2] = gap;
						this->front_v[2] = otherCar.v;
					}
			}
		}
		else if (otherCar.s < ego.s) {
			// area 3 
			if (otherCar.lane < ego.lane) {
				gap = ego.s - otherCar.s;
					if (gap < gaps[3]) {
						this->gaps[3] = gap;
					}
			}
			// area 4
			else if (otherCar.lane == ego.lane) {
				gap = ego.s - otherCar.s;
					if (gap < gaps[4]) {
						this->gaps[4] = gap;
					}
			}
			// area 5
			else if (otherCar.lane > ego.lane) {
				gap = ego.s - otherCar.s;
					if (gap < gaps[5]) {
						this->gaps[5] = gap;
					}
			}
		}
	}


}





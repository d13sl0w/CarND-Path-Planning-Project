//
// Created by dieslow on 11/11/17.
//

#ifndef PATH_PLANNING_MOTIONPLANNER_H
#define PATH_PLANNING_MOTIONPLANNER_H
#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

using namespace std;

enum LANE { LEFT_LANE = 1, CENTER_LANE = 2, RIGHT_LANE = 3};

enum STATE { KEEPING_LANE = 0, CHANGING_LANE = 1};

// This should probably be an inheritance isssue, ProtoCar with OtherCar as a simple
//  descendant and ControlledCar as the full on one... but of course data is given differently
struct OtherCar {
    int uid;
    double other_x;
    double other_y; // in map coordinates for all
    double other_vx; // in meters per second, of course
    double other_vy;
    double other_s;
    double other_d;
};

class MotionPlanner {
public:
    double my_x;
    double my_y;
    double my_yaw; // in map coordinates for all
    double my_s;
    double my_d;
    double my_speed_mph;

    LANE current_lane;
    LANE target_lane;

    double front_buffer_tolerance;
    double passing_buffer_tolerance;

    double target_median;
    double target_speed; // in meters per second  *25*
    double max_accel; // in m/s**2                *10*
    double max_jerk; // in m/s**3                 *50*

    std::vector<double>  previous_xs;
    std::vector<double>  previous_ys;

    double previous_final_s;
    double previous_final_d;

    const OtherCar& current_lane_car;

    bool currently_obstructed;
    bool left_lane_free;
    bool right_lane_free;
    bool imminent_collision_from_side;


private:
    double TIME_STEP_BETWEEN_PTS = 0.2; // seconds
    double max_sep_for_pts;

    OtherCar& find_leading_car() {
        return &OtherCar;
    };

    bool determine_if_obstructed() {
        return true;
    }

};


#endif //PATH_PLANNING_MOTIONPLANNER_H

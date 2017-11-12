//
// Created by dieslow on 11/11/17.
//

//#include "MotionPlanner.h"

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;

class FrenetDouble {
    // modular double for frenet s, maybe some mods for d/ <dx,dy> too
    // mod subtraction gonna work?
};

//template or generated depending on lane width?
enum LANE { LEFT_LANE = 1, CENTER_LANE = 2, RIGHT_LANE = 3};

enum STATE { KEEPING_LANE = 0, CHANGING_LANE = 1};

struct Waypoint {
    double x; // in map coordinates
    double y;
    double s; // distance along rode to get to point
    double dx; // unit normal vector components of vector pointing out of loop
    double dy;
};

// This should probably be an inheritance isssue, ProtoCar with OtherCar as a simple
//  descendant and ControlledCar as the full on one... but of course data is given differently
struct OtherCar {
    int uid;
    double other_x;
    double other_y; // in map coordinates for all

    double other_vx; // in meters per second, of course
    double other_vy; // OTHER CARS CAN EXCEED SPEED LIMIT BY 10MPH, WILL ALSO LIKE STAY ABOVE 50-10MPH
    double calculated_speed_from_above_vxvy;
    double calculated_yaw_from_above_vxvy;

    double other_s;
    double other_d;
};

class MotionPlanner {
private:
    // in map coordinates for all
    double ego_x, ego_y, ego_yaw, ego_s, ego_d, ego_speed_mph, prev_final_s, prev_final_d;
    std::vector<double>  prev_path_xs, prev_path_ys;
    auto sensor_fusion;



    double front_buffer_tolerance, passing_buffer_tolerance;

    // in meters per second ** n
    double target_median, target_speed, speed_limit, max_accel, max_jerk;

    double time_step_between_pts, max_sep_for_pts; // 0.02 seconds

    int latency_in_steps, steps_over_which_to_define_jerk; //1 to 3 expected; 5 is suggested

    double LANE_WIDTH; //width of lanes, in this case 4 meters each (6 lanes)

    const OtherCar& current_lane_car;

    bool currently_obstructed, left_lane_free, right_lane_free, imminent_collision {false};

    LANE current_lane, target_lane;

//    OtherCar& find_leading_car() {
//        return &OtherCar;
//    };

    bool determine_if_obstructed() {
        return true;
    }

public:


    void generate_new_path() {
        // update next xs and ys to return to simulator
        //  possibly melding with returned ones
    }

    void telemetry_update(auto const telemetry_packet) {
        // Main car's localization Data
        ego_x = telemetry_packet["x"];
        ego_y = telemetry_packet["y"];
        ego_s = telemetry_packet["s"];
        ego_d = telemetry_packet["d"];
        ego_yaw = telemetry_packet["yaw"];
        ego_speed_mph = telemetry_packet["speed"];

        // Previous path data given to the Planner
        prev_path_xs = telemetry_packet["previous_path_x"];
        prev_path_ys = telemetry_packet["previous_path_y"];

        // Previous path's end s and d values
        prev_final_s = telemetry_packet["end_path_s"];
        prev_final_d = telemetry_packet["end_path_d"];

        // Sensor Fusion Data, a list of all other cars on the same side of the road.
        sensor_fusion = telemetry_packet["sensor_fusion"];
        cout << "SENSOR****" << sensor_fusion[0] << endl;
    }

    MotionPlanner()

};
//
//
//#endif //PATH_PLANNING_MOTIONPLANNER_H

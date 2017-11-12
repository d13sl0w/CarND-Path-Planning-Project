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

//class FrenetDouble {
//    // modular double for frenet s, maybe some mods for d/ <dx,dy> too
//    // mod subtraction gonna work?
//};

// For converting back and forth between radians and degrees.

constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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
struct PathPair { vector<double> x_vals, y_vals; };

class MotionPlanner {
private:
    // in map coordinates for all
    // all internal variables in meters per second ** n
    double ego_x, ego_y, ego_yaw, ego_s, ego_d, ego_speed, prev_final_s, prev_final_d;
    std::vector<double>  prev_path_xs, prev_path_ys;
    auto sensor_fusion;
//    double front_buffer_tolerance, passing_buffer_tolerance;
//    double target_median, target_speed;
    double speed_limit, max_accel, max_jerk, lane_width; //width of lanes, in this case 4 meters each (6 lanes)
//    double time_step_between_pts, max_sep_for_pts; // 0.02 seconds
//    int latency_in_steps, steps_over_which_to_define_jerk; //1 to 3 expected; 5 is suggested
//    LANE current_lane, target_lane;
//    bool currently_obstructed, left_lane_free, right_lane_free, imminent_collision {false};
    double MPH_2_METPERSEC_FACTOR{0.44704}; //const? but breaks constructor????


public:
    MotionPlanner(double set_speed_limit, double set_max_accel, double set_max_jerk, double set_lane_width) :
            speed_limit(set_speed_limit), max_accel(set_max_accel), max_jerk(set_max_jerk), lane_width(set_lane_width) {}

    void set_speed_limit(const double set_speed_mph) {
        ego_speed = set_speed_mph * MPH_2_METPERSEC_FACTOR;
    }

    PathPair generate_new_path() {
        PathPair new_path;
        double dist_inc = 0.02;
        for(int i = 0; i < 50; i++) // 50 should actually be internal variable, same for above
        {
            new_path.x_vals.push_back(ego_x+(dist_inc*i)*cos(deg2rad(ego_yaw)));
            new_path.y_vals.push_back(ego_y+(dist_inc*i)*sin(deg2rad(ego_yaw)));
        }
        return new_path; //  possibly melding with returned old ones
    }

    void telemetry_update(auto const telemetry_packet) {
        // Main car's localization Data
        ego_x = telemetry_packet["x"];
        ego_y = telemetry_packet["y"];
        ego_s = telemetry_packet["s"];
        ego_d = telemetry_packet["d"];
        ego_yaw = telemetry_packet["yaw"];
        ego_speed = telemetry_packet["speed"];

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
//    const OtherCar& current_lane_car;
//    OtherCar& find_leading_car() {return &OtherCar;};
//    bool determine_if_obstructed() {return true;}
};
//
//

// ***************** TEST/MY DUMMY CODE *****************************************
//                    double pos_x, pos_y, pos_s, pos_d;
//                    double angle;
//                    int previous_path_size = previous_path_x.size();

//                    if (previous_path_size == 0) {
//                        pos_s = car_s;
//                        pos_d = car_d;
//                    }
//                    else {
//                        cout << "cars: " << car_s << ", card: " << car_d << endl;
//
////                        pos_x = previous_path_x[1];
////                        pos_y = previous_path_y[1];
////                        double pos1_x2 = previous_path_x[0];
////                        double pos1_y2 = previous_path_y[0];
////                        angle = atan2(pos_y - pos1_y2, pos_x - pos1_x2);
////                        auto frenet1 = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
////                        cout << "curposs: " << frenet1[0] << ", curposd: " << frenet1[1] << endl;
//
////                        pos_x = previous_path_x[previous_path_size - 1];
////                        pos_y = previous_path_y[previous_path_size - 1];
////
////                        double pos_x2 = previous_path_x[previous_path_size - 2];
////                        double pos_y2 = previous_path_y[previous_path_size - 2];
////                        angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
////
////                        auto frenet = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
////                        pos_s = frenet[0];
////                        pos_d = frenet[1];
////                        cout << "poss: " << pos_s << ", posd: " << pos_d << endl;
//
//                        pos_s = end_path_s;
//                        pos_d = end_path_d;
//                        // fill up this iterations path with remaining points first
//                        for (int i = 0; i < previous_path_size; i++) {
//                            next_x_vals.push_back(previous_path_x[i]);
//                            next_y_vals.push_back(previous_path_y[i]);
//                        }
//                    }

//                    double dist_inc = 30;
//                    double dist_inc_pt = 0.02;
//                    double next_s, next_d, next_x, next_y;
////                    pos_s = car_s;
////                    pos_d = car_d;
//                    vector<double> x_spline_feeder, y_spline_feeder;
//                    for (int i = 0; i < 3 ; i++) {
//                        next_s = pos_s + dist_inc;
//                        next_d = pos_d;
//
//                        auto xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//                        next_x = xy[0];
//                        next_y = xy[1];
//
//                        x_spline_feeder.push_back(next_x);
//                        y_spline_feeder.push_back(next_y);
//
//                        pos_s = next_s;
//                        pos_d = 6; //next_d;
//                    }
//
//                    tk::spline spline;
//                    spline.set_points(x_spline_feeder, y_spline_feeder);
//
//                    vector<double> xs, ys;
//                    int count = int(floor((dist_inc * 3) * dist_inc_pt));
//                    double xstep = (x_spline_feeder[2] - x_spline_feeder[0]) / count;
//                    double start_x;
//                    for (int i = 0; i <= count; i++) {
//                        start_x += i * xstep;
//                        xs.push_back(start_x);
//                    }
//
//                    double start_y;
//                    for (int i = 0; i <= count; i++) {
//                        start_y = spline(xs[0]);
//                        ys.push_back(start_y);
//                    }
//                    // END TEST DUMMY CODE
//
//                    next_x_vals = xs;
//                    next_y_vals = ys;

//                    cout << j[1] << endl;


//*************** END TO-DO/MY CODE!!!!!**************************************
//#endif //PATH_PLANNING_MOTIONPLANNER_H

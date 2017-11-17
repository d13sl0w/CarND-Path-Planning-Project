//
// Created by dieslow on 11/11/17.
//
#include <random>
#include "json.hpp"
#include "PlanningUtils.cpp"
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <sstream>
#include <vector>
#include <string>
#include <iterator>
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// TODO: PACKAGE UTILS INTO A NAMESPACE
using json = nlohmann::json;

using namespace std;

//class FrenetDouble {
//    // modular double for frenet s, maybe some mods for d/ <dx,dy> too
//    // mod subtraction gonna work?
//};

// For converting back and forth between radians and degrees.

//template or generated depending on lane width?
enum LANE {
    LEFT_LANE = 0, CENTER_LANE = 1, RIGHT_LANE = 2
};
enum STATE {
    KEEPING_LANE = 0, CHANGING_LANE = 1
};
struct Waypoint {
    double x; // in map coordinates
    double y;
    double s; // distance along rode to get to point
    double dx; // unit normal vector components of vector pointing out of loop
    double dy;
};
// This should probably be an inheritance isssue, ProtoCar with OtherCar as a simple
//  descendant and ControlledCar as the full on one... but of course data is given differently


// This should probably be an inheritance isssue, ProtoCar with OtherCar as a simple
//  descendant and ControlledCar as the full on one... but of course data is given differently
class OtherCar {
public:
    double uid = -1;
    double x, y; // in map coordinates for all
    double vx, vy; // in meters per second, of course OTHER CARS CAN EXCEED SPEED LIMIT BY 10MPH, WILL ALSO LIKE STAY ABOVE 50-10MPH
    double s, d;
    double speed, yaw; // relative to ego yaw?
    LANE current_lane;
//         STATE state;
    double distance_from_ego_s = std::numeric_limits<double>::max();

    OtherCar(int _uid, double _x, double _y, double _vx, double _vy, double _s, double _d) {
        uid = _uid;
        x   = _x;
        y   = _y;
        vx  = _vx;
        vy  = _vy;
        s   = _s;
        d   = _d;

        speed = sqrt(vx*vx + vy*vy); // both from_above_vxvy
        yaw   = atan2(vy, vx); // presumeably radians and map coordinates

        current_lane = static_cast<LANE>((int)round((d - 2.) / 4. )); // hacky, I know
    }

    OtherCar() = default;
};


struct PathPair {
    vector<double> x_vals, y_vals;
};

class MotionPlanner {
private:
    PlanUtils planUtils;
    // in map coordinates for all
    // all internal variables in meters per second ** n
    double ego_x, ego_y, ego_yaw, ego_s, ego_d, ego_speed, prev_final_s, prev_final_d;
    std::vector<double> prev_path_xs, prev_path_ys; // TODO: THIS RETURNS ONLY POINTS NOT TRAVELLED TO IN LATER SIMULATION ITERATION
    std::vector<std::vector<double>> sensor_fusion;
    vector<double> map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_s, map_waypoints_dy;
    double speed_limit, max_accel, max_jerk, lane_width; //width of lanes, in this case 4 meters each (6 lanes)
//    double time_step_between_pts, max_sep_for_pts; // 0.02 seconds
//    int latency_in_steps, steps_over_which_to_define_jerk; //1 to 3 expected; 5 is suggested
    LANE current_lane = CENTER_LANE; //, target_lane;
    double MPH_2_METPERSEC_FACTOR{0.44704}; //const? but breaks constructor????

    std::vector<OtherCar> other_cars;
    OtherCar leading_car;

    double front_buffer_distance = 25; //meters
    double target_speed {20}, ctrl_speed {0};
    bool LEFT_LANE_CLEAR {false}, RIGHT_LANE_CLEAR {false}, LEFT_LANE_ADVANTAGE {false}, RIGHT_LANE_ADVANTAGE {false};
    bool TOO_SLOW {false}, CAR_IN_ZONE {false}, ACCIDENT_INCIPIENT {false}, LEADING_CAR {false}, DESIRE_TURN {false};
    bool RIGHT_LEAD_CAR {false}, LEFT_LEAD_CAR {false}, IS_CHANGING_LANES {false}, SPEED_LIMITED {false};

public:
    MotionPlanner(double set_speed_limit, double set_max_accel, double set_max_jerk, double set_lane_width,
                  vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s,
                  vector<double> map_waypoints_dx, vector<double> map_waypoints_dy) :
            speed_limit(set_speed_limit), max_accel(set_max_accel), max_jerk(set_max_jerk),
            lane_width(set_lane_width), map_waypoints_x(map_waypoints_x), map_waypoints_y(map_waypoints_y),
            map_waypoints_s(map_waypoints_s), map_waypoints_dx(map_waypoints_dx),
            map_waypoints_dy(map_waypoints_dy) {} //TODO: make this one data structure
//
//    void set_speed_limit(const double set_speed_mph) {
//        ego_speed = set_speed_mph * MPH_2_METPERSEC_FACTOR;
//    }


    void build_car_list(std::vector<std::vector<double>> sensor_fusion_data) {
        other_cars.clear();
        for (const std::vector<double>& other_car : sensor_fusion_data) {
            OtherCar new_car = OtherCar(other_car[0], other_car[1], other_car[2], other_car[3],
                                        other_car[4], other_car[5], other_car[6]);
            new_car.distance_from_ego_s = new_car.s - ego_s; // frenet/modulo issues
            other_cars.push_back(new_car);
        }
    }

    void find_leading_car() { //TODO: has issue if there is none in front
        auto min_car_dist = std::numeric_limits<double>::max();
        for (const OtherCar& other: other_cars) {
            if (other.current_lane == current_lane &&
                    0 < other.distance_from_ego_s  &&
                    other.distance_from_ego_s < min_car_dist  &&
                    other.distance_from_ego_s < 50) {
                leading_car = other; //copy or what?
                min_car_dist = other.distance_from_ego_s;
                LEADING_CAR = true;
            }
        }
//        std::cout << "ego s: " << ego_s << " ego current_lane: " << current_lane << " uid: " << leading_car.uid << " lane: " << leading_car.current_lane << " d: " << leading_car.d <<
//                        " s: " << leading_car.s << " distance from s: " << leading_car.distance_from_ego_s << " speed: " << leading_car.speed << std::endl;

    }

    void is_leading_car_in_zone() {
        if (0 < leading_car.distance_from_ego_s && leading_car.distance_from_ego_s < front_buffer_distance) {
            CAR_IN_ZONE = true;
        }
    }

    // TODO: WHY ISN'T CAR OBEYING LEAD CAR IN RIGHT LANE
    double find_nearest_car_speed(LANE lane) { //TODO: has issue if there is none in front
        std::cout << "I'M RUNNING AT ALL EVEEEEERRR!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        OtherCar nearest_in_lane;
        bool CAR_EXISTS {false};
        double lane_change_front_buffer = 35;
        auto min_car_dist = std::numeric_limits<double>::max();
        for (const OtherCar& other: other_cars) {
            if (other.current_lane == lane && 0 < other.distance_from_ego_s  &&
                    other.distance_from_ego_s < min_car_dist &&
                    other.distance_from_ego_s < lane_change_front_buffer) {
                nearest_in_lane = other; //copy or what?
                min_car_dist = other.distance_from_ego_s;
                CAR_EXISTS = true;
            }
        }
        std::cout << "NIL uid: " << nearest_in_lane.uid << "NIL lane: " << nearest_in_lane.current_lane << "NIL d: " << nearest_in_lane.d <<
                  "NIL s: " << nearest_in_lane.s << "NIL distance from s: " << nearest_in_lane.distance_from_ego_s << "NIL speed: " << nearest_in_lane.speed << std::endl;
        double result = std::numeric_limits<double>::max();
        if (CAR_EXISTS) { result = nearest_in_lane.speed;}
        return result;
    }

    void are_speed_limited() {
        if ( speed_limit - leading_car.speed > 2.0) { SPEED_LIMITED = true; };
    }

    void check_lanes_available() {

        double turn_front_buffer = 40;
        double turn_rear_buffer = 15;

        LEFT_LANE_CLEAR = true;
        if (static_cast<int>(current_lane) - 1 < 0) {
            LEFT_LANE_CLEAR = false;
        } else {
            for (const auto& other : other_cars) {
                if (other.current_lane == current_lane - 1) {
                    if (other.s < ego_s + turn_front_buffer && other.s > ego_s - turn_rear_buffer) {
                        LEFT_LANE_CLEAR = false; }}}}

        RIGHT_LANE_CLEAR = true;
        if (static_cast<int>(current_lane) + 1 > 2) {
            RIGHT_LANE_CLEAR = false;
        } else {
            for (const auto& other : other_cars) {
                if (other.current_lane == current_lane+1) {
                    if (other.s < ego_s + turn_front_buffer && other.s > ego_s - turn_rear_buffer) {
                        RIGHT_LANE_CLEAR = false; }}}}
    }

    void is_lane_advantage() {
        std::cout << "CHECK ADVANTAGE!!!" << std::endl;
        double left_lead_speed = find_nearest_car_speed(static_cast<LANE>(current_lane - 1));
        if (left_lead_speed >= leading_car.speed + 1.0) { LEFT_LANE_ADVANTAGE = true; }

        double right_lead_speed = find_nearest_car_speed(static_cast<LANE>(current_lane + 1));
        if (right_lead_speed >= leading_car.speed + 1.0) { RIGHT_LANE_ADVANTAGE = true; }
    }

// TODO: I THINK OTHER CARS SPEED IS OFF!!!!
// TODO: THIS REALLY REALLY NEEDS TO BE MADE INTO A TRUE FSM
    void state_update() {
        // TODO: prevent switches till completion probs if {IS_CHANGING_LANES}
        LEADING_CAR = false;
        CAR_IN_ZONE = false;
        SPEED_LIMITED = false;
        LEFT_LANE_CLEAR = false;
        RIGHT_LANE_CLEAR = false;
        LEFT_LANE_ADVANTAGE = false;
        RIGHT_LANE_ADVANTAGE = false;


        find_leading_car();
        if (LEADING_CAR) {
            is_leading_car_in_zone();
            if (CAR_IN_ZONE) {
                target_speed = leading_car.speed - 1.0;
                are_speed_limited();
            }
            else { target_speed = (speed_limit + leading_car.speed) / 2; }
        }
        else { target_speed = speed_limit; }
        if (SPEED_LIMITED) { check_lanes_available(); }
        if (LEFT_LANE_CLEAR) { is_lane_advantage(); }
        if (LEFT_LANE_ADVANTAGE) {
            current_lane = static_cast<LANE>((int) current_lane - 1);
            target_speed = speed_limit;}
        else {
            if (RIGHT_LANE_CLEAR) { is_lane_advantage(); }
            if (RIGHT_LANE_ADVANTAGE) {
                current_lane = static_cast<LANE>((int) current_lane + 1);
                target_speed = speed_limit;}
        }


//        std::cout << " LEADING_CAR: " << LEADING_CAR
//                  << " CAR_IN_ZONE: " << CAR_IN_ZONE
//                  << " SPEED_LIMITED: " << SPEED_LIMITED
//                  << " RIGHT_LANE_CLEAR: " << RIGHT_LANE_CLEAR
//                  << " RIGHT_LANE_ADVANTAGE: " << RIGHT_LANE_ADVANTAGE
//                  << " LEFT_LANE_CLEAR: " << LEFT_LANE_CLEAR
//                  << " LEFT_LANE_ADVANTAGE: " << LEFT_LANE_ADVANTAGE <<std::endl;

    }



    void telemetry_update(json telemetry_packet) {
        // Main car's localization Data
        ego_x = telemetry_packet["x"];
        ego_y = telemetry_packet["y"];
        ego_s = telemetry_packet["s"];
        ego_d = telemetry_packet["d"];
        ego_yaw = telemetry_packet["yaw"];
        ego_speed = telemetry_packet["speed"];

        // Previous path data given to the Planner TODO: NO IDEA IF THIS ASSIGN SHIT IS GONNA FLY
        prev_path_xs.assign(telemetry_packet["previous_path_x"].begin(), telemetry_packet["previous_path_x"].end());
        prev_path_ys.assign(telemetry_packet["previous_path_y"].begin(), telemetry_packet["previous_path_y"].end());

        // Previous path's end s and d values
        prev_final_s = telemetry_packet["end_path_s"];
        prev_final_d = telemetry_packet["end_path_d"];

        // Sensor Fusion Data, a list of all other cars on the same side of the road.
        sensor_fusion = telemetry_packet["sensor_fusion"].get<std::vector<std::vector<double>>>();

        build_car_list(sensor_fusion);

        state_update();

//        std::cout << "target speed: " << target_speed << std::endl;
    }




    PathPair spline_next() { //splines guaranteed through points, typically made of piece wise from polynomials
        // ***************** TEST/MY DUMMY CODE *****************************************
        int prev_path_size = prev_path_xs.size(); //sim tells you this TODO: THIS RETURNS ONLY POINTS NOT TRAVELLED TO IN LATER SIMULATION ITERATION

        // create list of sparce x,y wpts, evenly spaced at 30m
        //  will later interpolate with spline and fill
        vector<double> ptsx, ptsy;

        // reference x,y yaw states; will either reference the starting point as where the car is
        //  or at the previous path's end point
        double ref_x = ego_x;
        double ref_y = ego_y;
//        cout << "ref x: " << ref_x << ", ref y: " << ego_y << endl;
        if (ctrl_speed > target_speed) { ctrl_speed -= 0.223 * target_speed / 15;}
        if (ctrl_speed < target_speed) { ctrl_speed += 0.223 * target_speed / 15;}
        double ref_vel = ctrl_speed;
//        double ref_vel = target_speed;
        double ref_yaw = planUtils.deg2rad(ego_yaw);

        if (prev_path_size > 0) { ego_s = prev_final_s; }
        // find angle car was heading via tangency of points, push two points to ptss

        // if previous size is almost empty use the car as starting reference???: does sim return full path or no
        if (prev_path_size < 2) {
            // use two points that make the path tangent to the car
//            cout << "option 1" << endl;
            double prev_car_x = ego_x - cos(ego_yaw);
            double prev_car_y = ego_y - sin(ego_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(ego_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(ego_y);

        } else { // use the prev path's end point as the starting reference
//            cout << "option 2" << endl;
            ref_x = prev_path_xs[prev_path_size - 1];
            ref_y = prev_path_ys[prev_path_size - 1];

            double ref_x_prev = prev_path_xs[prev_path_size - 2];
            double ref_y_prev = prev_path_ys[prev_path_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            // use two pts that make the path tanget to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
        }

        // in Frenet coords add i(here, 3) 30m spaced points ahead of starting reference
        for (int i = 1; i <= 2; i++) {
            vector<double> bkbone_waypts = planUtils.getXY(ego_s + (30 * i), (2 + 4 * current_lane), map_waypoints_s,
                                                           map_waypoints_x, map_waypoints_y);
            ptsx.push_back(bkbone_waypts[0]);
            ptsy.push_back(bkbone_waypts[1]);
        }

        // shift car ref angle to 0 to make math easie
        for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
        }

        tk::spline spline;
        // set x,y pts to spline

        spline.set_points(ptsx, ptsy);

        // define actual x,y pts we will use for this planner
        vector<double> next_x_vals, next_y_vals;

        // start with all previous points from last time
        for (int i = 0; i < prev_path_xs.size(); i++) {
            next_x_vals.push_back(prev_path_xs[i]);
            next_y_vals.push_back(prev_path_ys[i]);
        }

        // calculate how to break up spline points so that we travel at our desired ref velocity
        double target_x = 30.0;
        double target_y = spline(target_x);
        double target_dist = sqrt(((target_x) * (target_x)) + ((target_y) * (target_y)));


        // fill up the rest of our path planner after filling it with previous pts, here we will always
        //  output 50 pts
        double x_add_on = 0;
        for (int i = 1; i <= (50 - prev_path_xs.size()); i++) {
            double N = (target_dist / (0.02 * ref_vel));
            double x_point = x_add_on + (target_x) / N;
            double y_point = spline(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after rotating it earlier
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
        }
        PathPair results = {next_x_vals, next_y_vals};
        return results;
    }
};

































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

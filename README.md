# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Solution
My solution was to use splines for path definition (as implemented in the video) along with a parameterized control structure that implemented mostly as an ad hoc FSM, with some aspects of a cost function. The parameters were tuned largely by hand and meant to simulate my own personal driving style, i.e. the car is relatively risk averse, and will choose to slow down somewhat if it will continue to make progress relative to other lanes; once it is ahead of the adjacent lane's cars, with plenty of room, then it will actually change lanes (as opposed to aggressive, frequent, and speculative lane changing). The extent to which the car is willing to hold a sub-speed limit speed is, of course tuneable, as are the buffers it requires for following and for making lane changes. 

I worked to create the beginnings of a more modular system, creating some classes/structs to encapsulate the desired behavior, as well as to create more legible code. That said, the actual code leaves plenty to work on after I pass, including the packing up of magic numbers, seperation of car behavior and car motion concerns, and inheritance structure to simplify car/other car implementations. I also think using a true FSM library would likely simplify the state/behavioral code, especially as it increases in complexity. 

Finally, I'd like to try differing implementations altogether for this code. The JMTs seem appealing for more variant conditions, and are also just interesting mathematically, and I've experimented somewhat with a reinforcement learning/deep solutions via the MIT SDC courselet. 

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)
["x"] The car's x position in map coordinates
["y"] The car's y position in map coordinates
["s"] The car's s position in frenet coordinates
["d"] The car's d position in frenet coordinates
["yaw"] The car's yaw angle in the map
["speed"] The car's speed in MPH

#### Previous path data given to the Planner
//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 
["previous_path_x"] The previous list of x points previously given to the simulator
["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 
["end_path_s"] The previous list's last point's frenet s value
["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 



---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```


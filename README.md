# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Project Writeup

Starting with the steps outlined in the Project Walkthrough video, I implemented the Behavior Planning routine. Instead of using a Cost function, I used a logical process using the concepts outlined in the Finite State Machine discussion in the Behavior Planning lecture. I was concerned that tuning the Cost functions, and optimizing them could very rapidly become unwieldy, with changes having unanticipated consequences.
The Finite State Machine implemented has 3 states: Keep_Lane, Prepare for Change Lane, and Change Lane. The trigger for changing states is proximity to the vehicle in front (same lane as ego_car). Unless the ego_car comes up to a slow moving vehicle ahead,the ego_car will attempt to maintain current lane and drive at a speed of 49MPH (just below the 50MPH speed limit). The allowable proximity is dependent on ego_car's speed-- at higher speeds greater clearance must be maintained, analogous to the "2-second rule" drivers are instructed to follow; this calculation is performed on line 310 in main.cpp. A lane change is predicated on backwards proximity to vehicles in other lanes (and forward too), so that a lane change should not be attempted if a car is not adequately far behind (30m) or is traveling significantly faster (+10mph) than the ego_car. I may need to increase the speed threshold (from 10mph to 20mph) for greater robustness. The logic for Behavior Planning is in lines 346 to 385.

Sensor Fusion data being fed back from the simulator is used to track vehicles that are in close proximity to the ego_car, based on their Frenet s (displacement) coordinate. The code for the tracker is in lines 293-330. There was a glitch in calculating the distance between the ego_car and other cars as the track looped over itself after 6945.5m; the correction on line 187 corrects for this.

The trajectory generation is identical to the discussion in the walk through and utilizes the spline library for smoothing the trajectory. I extended the trajectory generation horizon to 75 points to help further smoothen point transitions.

### Results
The ego_vehicle is able to go around the track multiple times, usually without any incidents, and meeting all six of the criteria listed in the rubric. However, I believe that the "Sensor Fusion" reported speed of other vehicles is sometimes incorrect, and speeds of vehicles behind the ego_car get calculated to under 20mph, even as they came closer to the ego_car generally traveling above 30mph. It is possible that the simulator may be significantly modulating speeds of adjacent vehicles in order to challenge the ego_car.
I have noticed situations where the other cars "trap" the ego_car (nicely implemented simulator), which does simulate worst case scenarios that are less likely to occur in the real world, even though I concede that a self-driving car ought to be able to handle any scenario in order to be better than a human driver. Under such a situation, based on the logic implemented, the ego_car just stays in the same lane accelerating & decelerating like the surrounding cars in order to prevent a collision, but then is stuck traveling at a low average speed, and is unable to slow down significantly and then switch over 2 lanes over where there is no obstruction.

Using a cost function based approach that also looks at potential trajectories two lanes over, should enable the ego_car to get out of situations when it is surrounded by slow cars on three sides and a road boundary on the other. I am looking at implementing that as well, but submitting the project as it meets the rubric criteria.
I would also like to evaluate the impact of slower computational hardware in order to make the system more robust; I ran the code on an i7-7700k cpu with a dedicated graphics card.
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

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

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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


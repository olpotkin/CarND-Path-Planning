# CarND-Path-Planning-Project

---

[image1]: ./imgs/img_01.png "case_01"
[image2]: ./imgs/img_02.png "case_02"

### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data, a sparse map list of waypoints around the highway were provided. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the **6946m** highway. Since the car is trying to go **50 MPH**, it should take **a little over 5 minutes** to complete 1 loop. Also the car should **not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3**.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains ```[x, y, s, dx, dy]``` values. ```x``` and ```y``` are the waypoint's map coordinate position, the ```s``` value is the distance along the road to get to that waypoint in meters, the ```dx``` and ```dy``` values define the unit normal vector pointing outward of the highway loop.

The track contains a total of 181 waypoints, with the last waypoint mapping back around to the first. The waypoints are in the middle of the double-yellow diving line in the center of the highway.

The highway's waypoints loop around so the frenet ```s``` value, distance along the road, goes from 0 to 6945.554.

The track is 6945.554 meters around (about 4.32 miles). If the car averages near 50 MPH, then it should take a little more than 5 minutes for it to go all the way around the highway.

The highway has 6 lanes total - 3 heading in each direction. Each lane is 4 m wide and the car should only ever be in one of the 3 lanes on the right-hand side.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program


## Reflection

The major task in this project - to evaluate the best trajectory case, that depends on traffic conditions on the road.

Car could be in 3 available states and from each state there are different lane change scenarios:
- Lane 1 (left lane)
	- keep lane
	- turn right
- Lane 2 (middle lane)
	- turn left
	- keep lane
	- turn right
- Lane 3 (right lane)
	- turn left
	- keep lane

To make a decision what to do when our car are near to some car in the traffic we need to evaluate the cost of each scenario.

Cost evaluation algorithm for each lane consists of following steps:

1. Evaluate Lane Cost: if we are going to change the lane the cost is 1000.
2. Evaluate Speed Cost: higher the speed - lower the cost (speed limit is 49.5 mph).
3. Evaluate Collision cost and 15m gap cost (high values)

Example 1: 

![alt text][image1]

Example 2 (more difficult case): 

![alt text][image2]



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

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ```ID```, car's ```x position``` in map coordinates, car's ```y position``` in map coordinates, car's ```x velocity``` in m/s, car's ```y velocity``` in m/s, car's ```s position``` in frenet coordinates, car's ```d position``` in frenet coordinates. 

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


## Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).


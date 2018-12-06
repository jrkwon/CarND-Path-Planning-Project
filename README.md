[//]: # (image references)
[screenshot]: ./image/Screenshot.png 

# CarND-Path-Planning-Project

## Introduction

The project goal is to design a path planner that can create smooth and safe paths for the ego car to follow along a three-lane highway with other cars. The path planner must keep inside its lane as the ego car does not exceed the maximum speed, does not collide other cars and can change lanes smoothly to pass slower traffic without having jerks or sudden acceleration/braking.

## Implementation

Here is how I address the rubric points.

### At least 4.32 miles without incident

With all describe below, the ego car can drive more than 4.32 miles without any incident.

![screen shot](image/Screenshot_result.png)

### The car drives according to the speed limit.

I set the maximum velocity 49.5 mph and increase the current speed by 0.224 mph when necessary only if the speed is not over the maximum velocity. By doing so, the ego car's speed cannot exceed the speed limit. Also, the ego car's speed is slowing down by 0.224 mph when traffic is obstructed. 

### Max Acceleration and Jerk are not Exceeded.

By applying a slow step up and step down the ego car's speed, the car does not exceed the max acceleration and jerk. 

### Car does not have collisions.

The ego car does not contact with any other cars by applying my lane change policy that is described below.

### The car stays in its lane, except for the time between changing lanes.

The car doesn't spend more than a 3-second length outside the lane lanes while changing lanes, and every other time the car stays inside one of the 3 lanes on the right-hand side of the road.

### The car is able to change lanes

The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic. 

Three boolean variables are introduced: ``is_car_ahead, is_car_left, is_car_right`` from line 275 ~ 279.

```
            // ---------------------------
            // Check other cars' positions
            bool is_car_ahead = false;
            bool is_car_left = false;
            bool is_car_right = false;
```

There are as many cars as the size of ``sensor_fusion`` data. I have to check all these cars. I get the positions of cars using Frenet's ``s`` coordinate value.

First, get a lane of each of other cars. See line 283 ~ 291.

```
                int check_lane = -1;

                // get a car's lane.
                for (int l = 0; l < NUM_LANES; l++) {
                    if (d >= l*LANE_WIDTH && d <= (l+1)*LANE_WIDTH) {
                        check_lane = l;
                        break;
                    }
                }
```


Once I find out the car's lane, I can tell the car is ahead, left, or right of the ego car based upon the difference between the ego car's lane and another car's lane. If they are in the same lane, I have to check if the ego car is in a certain range from the other car. The range that I used is 30 meters. If they are in a different lane, I can tell the other car is left or right based on the difference of the ego car's lane and the others. In either case, I need to check not only a car ahead but also behind since this can be used to determine the lane change behavior. Again, I used 30 meters range to decide that the other car is ahead of the ego car in left or right lane and behind of the ego car. See line 303 ~ 316.

```
                if (lane == check_lane) {
                    // if a car is in my lane,
                    is_car_ahead |= (car_s < check_car_s && ((check_car_s - car_s) < NUM_SPACED_POINTS));
                }
                else if ((check_lane - lane) == -1) {
                    // this car is one lane left from my car.
                    is_car_left |= (car_s < check_car_s + NUM_SPACED_POINTS) 
                                    && (car_s > check_car_s - NUM_SPACED_POINTS);
                }
                else if ((check_lane - lane) == 1) {
                    // this car is one lane right from my car
                    is_car_right |= (car_s < check_car_s + NUM_SPACED_POINTS) 
                                    && (car_s > check_car_s - NUM_SPACED_POINTS);
                }
```

Based upon these three boolean variables, I can slow down if the car is ahead and there is no way to change lanes. If a car is ahead of the ego car and there is enough room in the left or right lane, lane-change is happening. If there is no car ahead, then as I described earlier, the ego car's speed is increasing by 0.224 mph. See line 319 ~ 350.

```
            // -------------------------
            // Slow down or change lanes
            if (is_car_ahead) {
                if (lane > 0 && !is_car_left) {
                    // change lane to left
                    lane--;
                }
                else if (!is_car_right && lane != (NUM_LANES-1)) {
                    // change to right
                    lane++;
                }
                else {
                    // if you can't change lane, slow down
                    ref_vel -= DIFF_VEL;
                }
            }
            else {
                // no car is ahead in 30m range
                if (ref_vel < MAX_VEL) {
                    // then speed up
                    ref_vel += DIFF_VEL;
                }
                // // let's use the center line if possible
                // if (lane > 0 && !is_car_left) {
                //     // change lane to left
                //     lane--;
                // }
                // else if (!is_car_right && lane != (NUM_LANES-1)) {
                //     // change to right
                //     lane++;
                // }
            }
```

## Result

[![output](https://img.youtube.com/vi/ZtRIu6IAJF0/0.jpg)](https://youtu.be/ZtRIu6IAJF0)

---

The description below is Udacity's original README for this project repository.

----

# CarND-Path-Planning-Project

Self-Driving Car Engineer Nanodegree Program
   
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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).


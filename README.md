# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Implementation

### Behavior planner

Behavior planner first creates candidate trajectories with different speed and highway lanes [line 18-19](./src/behavior_planner.cpp#L18):
```c++
vector<double> candidate_velocities = generate_candidate_velocities();
vector<int> candidate_lanes = generate_candidate_lanes(ego_vehicle);
```

Cost is calculated for each candidate [line 28](./src/behavior_planner.cpp#L28):

```c++
double candidate_trajectory_cost = this->calculateCost(ego_vehicle, other_vehicles, 
		    previous_path[0].size(), candidate_lane, candidate_velocity);
```

Candidate trajectory with lowest cost is chosen and full trajectory with x and y coordinates is generated [line 37](./src/behavior_planner.cpp#L37):

```c++
this->best_trajectory = trajectory_generator.generateTrajectory(ego_vehicle.x, ego_vehicle.y, ego_vehicle.yaw, ego_vehicle.s, 
				candidate_lane_d, candidate_velocity, previous_path);	
```

Currently cost calculation does not need full trajectory coordinates, hence it is generated only in the end for best 
trajectory. For real life situation, full trajectory would be probably needed for cost calculation and would be 
generated for every candidate trajectory.

Sample of costs for different candidate trajectories:
```
Candidate trajectory: candidate_lane=0, candidate_velocity=50, cost=11049
Candidate trajectory: candidate_lane=0, candidate_velocity=49.75, cost=11001.5
Candidate trajectory: candidate_lane=0, candidate_velocity=49.5, cost=9953.95
Candidate trajectory: candidate_lane=0, candidate_velocity=49.25, cost=9906.45
Candidate trajectory: candidate_lane=0, candidate_velocity=49, cost=9858.95
Candidate trajectory: candidate_lane=1, candidate_velocity=50, cost=1000
Candidate trajectory: candidate_lane=1, candidate_velocity=49.75, cost=1002.5
Candidate trajectory: candidate_lane=1, candidate_velocity=49.5, cost=5
Candidate trajectory: candidate_lane=1, candidate_velocity=49.25, cost=7.5
Candidate trajectory: candidate_lane=1, candidate_velocity=49, cost=10
Candidate trajectory: candidate_lane=2, candidate_velocity=50, cost=2382.51
Candidate trajectory: candidate_lane=2, candidate_velocity=49.75, cost=2385.01
Candidate trajectory: candidate_lane=2, candidate_velocity=49.5, cost=1387.51
Candidate trajectory: candidate_lane=2, candidate_velocity=49.25, cost=1390.01
Candidate trajectory: candidate_lane=2, candidate_velocity=49, cost=1392.51
```

See chapter Trajectory generator for more information.

### Cost calculation

For every candidate trajectory, cost is calculated in `BehaviorPlanner::calculateCost` [lines 63-118](./src/behavior_planner.cpp#L63) by following rules:

1. **Faster speed is preferred** [line 68](./src/behavior_planner.cpp#L68)
2. **Speed limit is honoured**, with adding very high cost to speeds above the limit [lines 71-73](./src/behavior_planner.cpp#L71)
3. **Keep safety distance to a vehicle in front** on the same lane by giving higher cost to trajectory candidates with higher speed [lines 85-91](./src/behavior_planner.cpp#L85)
4. **Keep safety distance to vehicle in beside lanes**, higher cost is given to vehicles on beside lanes, than vehicle on the same lane, so that ego vehicle would not start to change lanes when surrounded with vehicles and would instead start to slow down (see rule 3). [lines 95-103](./src/behavior_planner.cpp#L93)
5. **Prefer empty space** by adding up a cost for every vehicle on candidate lane, very important when choosing whether to take right or left lane change [lines 105-108](./src/behavior_planner.cpp#L105)
6. **Middle lane is preferred** as it has more options (switch lane right and left) [line 113](./src/behavior_planner.cpp#L113)

### Trajectory generation

Trajectory is generated using spline library in `trajectory_generator.cpp` class by following approach described in 
Udacity project Q&A video.

Previous path coordinates (trajectory not consumed yet by simulator) are used as starting point for generating new trajectory,
this causes trajectory transitions to be smooth. See `trajectory_generator.cpp` [lines 30-55](./src/trajectory_generator.cpp#L55).

3 anchor points spaced by 30 in s and d coordinates are used to define a spline `trajectory_generator.cpp` [lines 57-74](./src/trajectory_generator.cpp#L57):

```c++
int anchor_points_count = 3;
int anchor_points_spacing = 30;
for (int i = 1; i <= anchor_points_count; i++) {
    int anchor_point_s = car_s + i*anchor_points_spacing;
    vector<double> anchor_point = getXY(anchor_point_s, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    anchor_points_x.push_back(anchor_point[0]);
    anchor_points_y.push_back(anchor_point[1]);  
}
```

Then all other points are interpolated with spline library with appropriate spacing needed for low jerk and 
accelerations `trajectory_generator.cpp` [lines 78-113](./src/trajectory_generator.cpp#L78):

```c++
tk::spline s;
s.set_points(anchor_points_x, anchor_points_y);

...

for (int i = 1; i <= 50-previous_path_x.size(); i++) {
    double N = target_dist / (.02*reference_velocity/2.24); // todo: rename step length, move outside of loop
    double x_point = x_add_on+target_x/N;
    double y_point = s(x_point);

    ...

    interpolated_points_x.push_back(x_point);
    interpolated_points_y.push_back(y_point);
}
```

For simplifying calculation, car reference angle is shifted to 0 and result is shifted back to original yaw.

### Rubric criterias

This chapter describes how different Rubric criterias are addressed.

#### The code compiles correctly.

New source files were added to `CMakeList.txt`, but project should still compile on any platform.

#### The car is able to drive at least 4.32 miles without incident.

Screenshot of car driving 10.26 miles without incidents:

![No incidents image](/images/no-incidents.png)

#### The car drives according to the speed limit.

The car drives according to speed limit, because exceeding speed limit is given highest cost in 
`trajectory_generator.cpp` [lines 71-73](./src/behavior_planner.cpp#L71):

```c++
if (candidate_velocity > (SPEED_LIMIT - 0.5)) {
	cost += 1000.0;
}	
```

#### Max Acceleration and Jerk are not Exceeded.

Max acceleration and jerk is not exceeded by changing reference speed by small increments 
`trajectory_generator.cpp` [lines 45-47](./src/behavior_planner.cpp#L45):

```c++
vector<double> BehaviorPlanner::generate_candidate_velocities() {
	return { ref_vel+0.50, ref_vel+0.25, ref_vel, ref_vel-0.25, ref_vel-0.50};	
}
```

and by adding appropriate amount of points onto trajectory `trajectory_generator.cpp` [lines 93-95](./src/trajectory_generator.cpp#L93):

```c++
double N = target_dist / (.02*reference_velocity/2.24); // todo: rename step length, move outside of loop
double x_point = x_add_on+target_x/N;
double y_point = s(x_point);
```


#### Car does not have collisions.

Car avoids collisions by car keeping safety distance with other vehicles. This is achieved by having high cost for
candidate trajectories that have ego vehicle close to other vehicles. 
See `behaviour_planner.cpp` [lines 85-103](./src/behavior_planner.cpp#L71).

#### The car stays in its lane, except for the time between changing lanes.

The car stays in its lane because generate trajectory target point in Frenet is defined to be in a middle of lane 
`behaviour_planner.cpp` [line 36](./src/behavior_planner.cpp#L36):

```c++
double candidate_lane_d = 4*LANE_WIDTH + LANE_WIDTH/2;
```

#### The car is able to change lanes

Candidate trajectories with lane changes is created and cost is calculate for them. When cost for trajectories with
lane changes is lower than current lane, vehicle changes lane. See `behaviour_planner.cpp` [line 49-61](./src/behavior_planner.cpp#L49).
   
## How to use

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Basic Build Instructions

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


## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---

### The goals of this project are:
1. Loading a 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].

---

### General structure of the code

The scripts `motion_planning.py` and `planning_utils.py` contain the basic planning implementation. Following 

#### `motion_planning.py`

 - connect to simulator
 - registering callbacks to messages returned from simulator
 - `local_position_callback()`:
 	 - when in state TAKEOFF: make sure the height is reached, then change to `waypoint_transition()`: change state to WAYPOINT, take next waypoint as target position, `cmd_position()`
 	 - when in state WAYPOINT: check if in vicinity (1m) of target location
 	 	- move to next waypoint or transition to landing if velocity is low enough (<1m/s), `landing_transition()`: change state to LANDING, `land()`
 - `velocity_callback()`:
 	 - when in state LANDING: check local and global position and transition to disarm, `disarming_transition()`: change state to DISARMING, `disarm()`, `release_control()`
 - state_callback():
 	 - MANUAL --> arming transition: `arm()`, `take_control()`
 	 - ARMING: if armed: `plan_path()`
 	 - PLANNING --> takeoff transition: change state to TAKEOFF, `takeoff()`
 	 - DISARMING: if not armed and guided --> `manual_transition()`: change state to MANUAL, `stop()`, set in_mission to False

- plan_path()
	- define target altitude and safety distance
	- set target position height to target altitude
	- read lattitude and longtitude from `colliders.csv` and set home position
	- convert global home position to relative local (north, east) position
	- read in obstacle map and convert to 2D grid at target altitude with safety distance using the method `create_grid()`
	- convert relative local position to grid local position
	- save specific global target positions in array and select one randomly
	- convert the global target lcoation to grid specific local position
	- use `a_star()` from `planning_utils.py` to find path from local start to goal
	- prune the path via `prune_path()` from `planning_utils.py`
	- convert path to waypoints, visualize them in simulator and follow them


#### `planning_utils.py`

 - contains the following methods:
 	- `create_grid()`
 	- `valid_actions()`
 	- `a_star()`
 	- `heuristic()`
 	- `collinearity_check()`
 	- `prune_path()`



And here's a lovely image of my results (ok this image has nothing to do with it, but it's a nice example of how to include images in your writeup!)
![Top Down View](./misc/high_up.png)

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

---

### Implementation of the Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.


And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.


Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)

![NESW Path](./misc/plot_path_nesw.png)

Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

![Diagonal Path](./misc/plot_path_diagonal.png)

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

![Pruned Path](./misc/plot_path_pruned.png)



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.



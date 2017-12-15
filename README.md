# Path Planning Project
This project is part of the Udacity Self-Driving Car Engineer Nanodegree Program. It is the 1st project of the final term 3. The project base code can be found under https://github.com/udacity/carnd-path-planning-project. The corresponding simulator, which interacts with this program through a websocket protocol, can be downloaded here https://github.com/udacity/self-driving-car-sim/releases -> Look for the term 3 simulator. As part of the base code provided by udacity, the main.cpp is given plus the highway map data.

## Goals

The target was to write an autonomous path planner which provides waypoints that the car can follow. The waypoints are designed to be followed by the simulator using a perfect controller and each is separated from the following by a 0.02 s interval. By choosing different locations and distances the path planner can control speed and acceleration of the vehicle.

Ultimately, the target of the path planner are as follows:
- Complete at least one lap (around 7000m) without any interruptions
- Do not collide with other vehicles
- Stay below the speed limit of 50 MPH
- Stay below the acceleration limit of 10m/s2 at any given time
- Stay below the limit for jerk (1st derivative of acceleration) of 50 m/s3 at any given time
- Indirect goal: In order to complete the lap at a speed close to the speed limit (around 5 minutes), overtake other vehicles if neighbouring lanes are empty. This is actually the real and most difficult task here.

## Explanation of major functions

#### class SDCPlanner

The src/SDCPlanner.cpp file contains the SDCPlanner class with its respective member functions. The reason why I have chosen a class to contain most of the data of the ego vehicle is that it interacts easily with the main.cpp and stores the many variables that are needed for the vehicle across each main loop.

#### SDCPlanner::update_own

This function simply guarantees that the current values from the simulator are stored as member variables in the class to which all functions have access to. On top of the simulator values, it stores a time value delta_t since the last update and determines the speed of the end of the previous waypoints (the simulator returns all "unused" waypoints which can be used as a starting point to extend the new path from).

#### SDCPlanner::predict

This member function takes sensor fusion data from the simulator (idealized data from, e.g. radar or lidar, but in perfect, clean form). For each other vehicle on the road, it provides (x,y) coordinates as well as the corresponding frenet (s,d) coordinates and speed in (x,y) components. Using this data, predict finds for each lane the vehicle closest ahead and behind and stores the results in two vectors, cars_ahead and cars_behind. I have found that each closest vehicle is sufficient for the purpose here, however, in real life you will want to track all vehicles on the road, especially when planning for more complex scenarios such as turning at an intersection. Actually, designing SDCPlanner as a class helps with this, because each other vehicle could then be stored as an individual class as well and have access to all of its member functions to calculate their respective trajectories as well. This is not done here, because it is not necessary and would increase computational requirements.

#### SDCPlanner::update_state

This is the central function to the state planner. In its current simple form, the state planner only uses three states: KL (keep lane), LCL (lane change left), LCR (lane change right). It could easily be extended to more states, though, if required. The function will only be called from main() if two requirements are met:
- Target lane is reached: This is to avoid to make a lane change only half way and turn back. For safety reasons, it may be desirable to change this later, but in this setting this requirement makes sense.
- Update cycle is reached: This is to avoid to make an update calculation in every loop. Usually, the state planner is a high-level function that is computationally expensive. It is not required to update the state every 20ms. Usually, for each one update of the state, several new trajectories are calculated and sensor data updated. The safety critical functions must run at a high update speed.

The function takes into account the current state and determines future possible states (finite state machine). Not all states can be reached from each state. Think of driving in the left-most lane in the keep-lane state. It is forbidden to do a lane change left, since that would be the upcoming lane (at least in this highway setting). Therefore, only keep lane and lane change right are permitted.

The state planner then calculates a simple (s,d) trajectory for each state and calculates the associated cost. The state with the lowest cost will be chosen as the next state for which a detailed trajectory is calculated.

#### SDCPlanner::plan_sd_trajectory

For each state, a simple trajectory in (s,d) coordinates is generated. This is done at a rough timestep of PREDICT_STEP, because then each trajectory contains only a few points which are easier to calculate. It would also alternatively be possible to calculate a detailed trajectory at 20ms intervals instead, but computationally more expensive (which may be a problem if you want to run the code on SOC based computers like the Raspberry Pi).

For the calculation of the trajectory, it takes into account the speed at each timestep, which accelerates until maximum speed at the permitted acceleration value. The maximum speed is determined in safe_speed(), which is either the speed limit if no vehicle is ahead within a safe distance of travelling two seconds at current speed, or an interpolation between the own speed and the other vehicle's speed. If a safety minimum is breached (usually only happens if another car changes onto the ego vehicle lane at close distance), the speed is decelerated to zero. Lowering the speed should quickly lead to an increase in the distance and therefore to an increase in the own speed as well. The lateral d component is calculated at a constant lateral change speed that corresponds to a turn angle of roughly 30° until the target lane is reached.

#### SDCPlanner::cost_for_trajectory

This function is central to evaluating which next state is the best with the least cost. For this, three parts are evaluated:
- Target cost calculates which trajectory gets ahead fastest. If a car ahead blocks our path and reduces to a speed below the speed limit, whereas the left or right lane is empty, enabling us to accelerate to the speed limit, the left or right lane trajectory get a lower cost assigned
- Safety cost calculates the minimum distance to another vehicle on the planned trajectory. A lower distance results in a higher score, encouraging paths that stay comfortably clear of other vehicles
- Lane change cost assigns a relatively small penalty to a lane change, which simply achieves that the car does not change lanes too easily and all the time, requiring a reasonably faster and safer trajectory for a lane change compared to keeping the current lane

#### SDCPlanner::spline_trajectory

Here, a detailed trajectory in (x,y) coordinates is calculated based on key waypoints which the spline needs to pass through. The first points are the last two points from the previously calculated path, together with the (s,d) points handed over to this function. These points are transformed relative to the current car position (x=y=yaw=0). Using a car centered coordinate system makes it easier to calculate the next x values (linear extrapolation of the current speed multiplied with the interval time). Based on the x coordinate, the y coordinate is calculated from the spline. These coordinates then only need to be transformed back to global (x,y) coordinate system and returned to the simulator to be followed.

#### SDCPlanner::trajectory

This is the function that feeds spline_trajectory with the key (s,d) waypoints. In order to do this, it simply extrapolates two timesteps into the future and calculates a corresponding (s,d) value similar to the plan_sd_trajectory function.

#include "SDCPlanner.h"
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <chrono>

/* Spline from https://github.com/ttk592/spline/
used to generate a spline interpolation for single waypoints
to make sure the generated fit passes exactly through each calculated waypoint
*/
#include "spline.h"

using namespace std;

/* Initiates a SDCPlanner class with the major variables it is holding
Inputs: Waypoint vectors with s, x and y coordinates. These vectors will be copied into class variables to avoid 
having to carry them over in every function call. They are constant over runtime
*/
SDCPlanner::SDCPlanner(const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	
	this->state = "KL";
	this->current_lane = floor(d/4); // 1st lane from d = 0-4 m, 2nd 4-8 m, 3rd 8-12 m. Middle lane at 2m, 6m and 10m respectively
	this->target_lane = this->current_lane; // First target lane where the car is starting
	this->state_update_time = 0;
	this->x = 0;
	this->y = 0;
	this->s = 0;
	this->d = 0;
	this->delta_t = 0;
	this->map_wp_x = maps_x;
	this->map_wp_y = maps_y;
	this->map_wp_s = maps_s;
	init_cars_aheadbehind();
}

/* Destructor */
SDCPlanner::~SDCPlanner() {}

/* Updates state variables with feedback values from the simulator in every loop from main() */
void SDCPlanner::update_own(double x, double y, double yaw, double s, double d, double car_speed, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d) {


	// Calculates delta t since last update to determine derivatives
	chrono::steady_clock::time_point current_time = chrono::steady_clock::now();
	chrono::duration<double> timelapse = current_time - this->previous_time;
	double delta_t = timelapse.count();
	this->delta_t = delta_t;
	this->previous_time = current_time;

	this->x = x;
	this->y = y;
	this->yaw = yaw;
	this->s = s;
	this->d = d;
	this->current_lane = floor(d/4);
	this->speed = car_speed;
	this->speed_end_path = car_speed;
	this->prev_path_x = previous_path_x;
	this->prev_path_y = previous_path_y;
	int prev_size = previous_path_x.size();
	// If a previous trajectory exists, then this calculates the speed between the 2nd to last and the last waypoint
	// This speed is important to determine at which speed waypoints extending from there should be planned
	if (prev_size > 1) {
		this->end_path_s = end_path_s;
		this->end_path_d = end_path_d;
		this->speed_end_path = distance(previous_path_x[prev_size-1], previous_path_y[prev_size-1], previous_path_x[prev_size-2], previous_path_y[prev_size-2]) / TIME_INTERVAL;
	}
	else {
		this->end_path_s = s;
		this->end_path_d = d;
	}

	// cout << "Current s, s_d, s_dd: " << s << ", " << s_d << ", " << s_dd << " and d, d_d, d_dd: " << d << ", " << d_d << ", " << d_dd << endl; // DEBUG
}


void SDCPlanner::predict(vector< vector<double> > sensor_data) {
	
	// Resets distances to vehicles closest ahead and behind to a standard high value
	// The following loop then sets this value to the real value if cars are found closer ahead or behind than this value
	reset_cars_aheadbehind(); 
	
	// Iterate through other vehicles on the road, one loop per vehicle
	for (int i = 0; i < sensor_data.size(); i++) {
		int id = sensor_data[i][0];
		double p_x = sensor_data[i][1]; // Position x
		double p_y = sensor_data[i][2]; // Position y
		double v_x = sensor_data[i][3]; // Speed x direction
		double v_y = sensor_data[i][4]; // Speed y direction
		double s = sensor_data[i][5];
		double d = sensor_data[i][6];

		int lane = floor(d/4); // Current lane of the other vehicle
		double speed = sqrt(v_x * v_x + v_y * v_y); // Current speed of other vehicle
		double delta_s = s - this->s; // Lateral distance towards other vehicle from ego vehicle POV. Delta s is positive if a car is ahead of us

		// Makes sure that delta s is only between "-pi and +pi". The simulator artificially resets s to zero after crossing the MAX_S variable
		// If a car ahead crosses this mark, the relative distance will be close to -MAX_S, whereas actually it is a small positive distance
		if (delta_s < (-MAX_S/2)) {
			delta_s += MAX_S;
		}
		else if (delta_s > (MAX_S / 2)) {
			delta_s -= MAX_S;
		}

		// First index: timestep (at interval PREDICT_STEP)
		// Second index: 0 = s, 1 = d
		vector<vector<double>> predictions;

		double s_i = s;
		double d_i = d;
		// Prediction vector of other vehicles position every PREDICT_STEP seconds, starting from position at t=0
		for (int j = 0; j < (PREDICT_HORIZON / PREDICT_STEP); j++) {
			predictions.push_back({s_i, d_i});
			s_i += speed * PREDICT_STEP;
			d_i = d;
		}

		// DEBUG: Prints out each found vehicle with its most important sensor data
		// cout << "Vehicle ID: " << id << ", d: " << d << " , lane: " << lane << " , speed: " << speed << " , relative distance: " << delta_s << endl;

		// Vehicle will only be added if it's on the ego vehicle side of the road (d between 0m and 12m)
		// This is to prevent buggy cars (high negative d values) causing errors in the algorithm before the simulator is properly initiated
		if ( (d > 0) && (d < (4 * NR_LANES)) ) {
			// Cars behind with a delta_s < 0, the smallest distance will be added to cars_behind vector (one for each lane)
			if ((delta_s < 0) && (delta_s > this->cars_behind[lane].delta_s)) {
				vehicle_track current_vehicle = {lane, speed, delta_s, predictions};
				this->cars_behind[lane] = current_vehicle;
			}
			// Cars ahead, respectively
			else if ((delta_s > 0) && (delta_s < this->cars_ahead[lane].delta_s)) {
				vehicle_track current_vehicle = {lane, speed, delta_s, predictions};
				this->cars_ahead[lane] = current_vehicle;
			}
		}
	}
	// DEBUG: Prints out the closest vehicle ahead and behind for each lane
	/*
	for (int i=0; i< this->cars_ahead.size(); i++) {
		cout << "Car ahead in lane " << this->cars_ahead[i].lane << ", relative distance: " << this->cars_ahead[i].delta_s << endl;
		cout << "Car behind in lane " << this->cars_behind[i].lane << ", relative distance: " << this->cars_behind[i].delta_s << endl;
	}*/
}

/* State planner: Determines depending on current state which future states are possible and calculates the best one out of these
depending on a cost function, then selects the best one and exectutes the according trajectory
*/
void SDCPlanner::update_state() {

	vector<string> possible_states;

	// Determines which next states are possible based on the current state in the Finite State Machine
	if ((this->state == "KL") && (this->current_lane == 1)) {
		possible_states = {"KL", "LCL", "LCR"};
	}

	if ((this->state == "KL") && (this->current_lane == 0)) {
		possible_states = {"KL", "LCR"};
	}

	if ((this->state == "KL") && (this->current_lane == 2)) {
		possible_states = {"KL", "LCL"};
	}

	if (this->state == "LCR") {
		if (target_lane_reached(this->target_lane, this->d)) {
			possible_states = {"KL"};
		}
		else {
			possible_states = {"LCR"};
		}
	}

	if (this->state == "LCL") {
		if (target_lane_reached(this->target_lane, this->d)) {
			possible_states = {"KL"};
		}
		else {
			possible_states = {"LCL"};
		}
	}

	int nr_states = possible_states.size();

	vector<vector<double>> sd_trajectory;
	double state_cost = 0;

	// DEBUG: Prints out all possible next states that are planned for
	/*
	cout << "List of possible states: ";
	for (int i= 0; i < nr_states; i++) {
		cout << possible_states[i] << ", ";
	}
	cout << endl;
	*/

	// Update only when not in the process of changing lanes
	if (target_lane_reached(this->target_lane, this->d)) {

		double cost_min = 10000.0;

		// For each possible state, calculate a trajectory
		for (int i=0; i < nr_states; i++) {

			sd_trajectory = plan_sd_trajectory(possible_states[i]);
			state_cost = cost_for_trajectory(sd_trajectory);

			// State with minimum cost will be chosen next
			if (state_cost < cost_min) {
				this->state = possible_states[i];
				cost_min = state_cost;
			}
			// cout << "Cost for state " << possible_states[i] << ": " << state_cost << endl;
		}
		// cout << "New state: " << this->state << endl;

		if (this->state == "PLCL" || this->state == "LCL") {
			this->target_lane = this->current_lane - 1;
		}
		else if (this->state == "PLCR" || this->state == "LCR") {
			this->target_lane = this->current_lane + 1;
		}
		else if (this->state == "KL") {
			this->target_lane = this->current_lane;
		}
	}
}	

/* Calculates and returns trajectory in (x,y) coordinates for the current state
*/
vector< vector<double> > SDCPlanner::trajectory() {

	vector<vector<double>> trajectory;

	// cout << "Trajectory in planning... for state: " << this->state << endl;

	/* plan_time is used to calculate the key spline points for the planned path. It either takes a part of the total length
	or the "remaining" points from the last planned trajectory into account, whichever is larger.
	Usually, if the system is fast to update regularly, the remaining points should be very few and the first option will always
	be chosen except for when the first path vector is calculated, with no previous values available */
	double plan_time = max(0.5 * PATH_LENGTH * TIME_INTERVAL, (PATH_LENGTH - this->prev_path_x.size()) * TIME_INTERVAL);
	double d_target = this->target_lane * 4 + 2; // Target d value, e.g. 2 for lane 0 (middle of the lane)
	double dist_ahead = this->cars_ahead[target_lane].delta_s; // Distance to closest vehicle ahead in the target lane
	double speed_ahead = this->cars_ahead[target_lane].speed; // Speed to closest vehicle ahead in the target lane
	double max_speed = safe_speed(dist_ahead, this->speed, speed_ahead); // Based on above dist. and speed ahead, calculates a safe speed for ego vehicle

	// The s and d_ahead points are used as key points for the planned trajectory where the spline passes through
	double delta_s_ahead = min(this->speed_end_path * plan_time + 0.5 * MAX_ACCELERATION * plan_time * plan_time, max_speed * plan_time);
	double s_ahead1 = this->end_path_s + delta_s_ahead;
	double s_ahead2 = this->end_path_s + 2 * delta_s_ahead;

	double d_deviation = d_target - this->end_path_d; // Current lateral distance towards target location
	int d_direction = d_deviation / abs(d_deviation); // -1 if right of target, +1 if left of target
	double d_delta = this->speed_end_path * 0.083 * plan_time * d_direction; // Total lateral distance during plan_time (to s_ahead1) at a vehicle turn angle of about 15° (sin 15° ~= 0.083)
	double d_ahead1, d_ahead2;
	// cout << "d deviation = " << d_deviation << ", d_direction = " << d_direction << ", d_delta = " << d_delta << ", d_target = " << d_target << endl;

	// d waypoints are determined by turning at an angle of about 20° left or right.
	// If a constant turn at this angle overshoots the target, the waypoint is simply set towards the precise target d
	if (abs(d_delta) > abs(d_deviation)) {
		d_ahead1 = d_target;
	}
	else {
		d_ahead1 = this->end_path_d + d_delta;
	}
	// d_ahead2 is calculated like d_ahead1, but at twice the distance, corresponding to s_ahead2
	if ((2 * abs(d_delta)) > abs(d_deviation)) {
		d_ahead2 = d_target;
	}
	else {
		d_ahead2 = this->end_path_d + 2 * d_delta;
	}

	// cout << "Last waypoint s = " << this->end_path_s << ", s1 = " << s_ahead1 << ", s2 = " << s_ahead2 << ". Last waypoint d = " << this->end_path_d << ", d1 = " << d_ahead1 << ", d2 = " << d_ahead2 << endl;

	vector<double> s_ahead;
	vector<double> d_ahead;
	s_ahead.push_back(s_ahead1);
	s_ahead.push_back(s_ahead2);
	d_ahead.push_back(d_ahead1);
	d_ahead.push_back(d_ahead2);

	// Calculates and returns a splined trajectory based on the (s,d) waypoints
	trajectory = spline_trajectory(s_ahead, d_ahead, max_speed);
	return trajectory;
}

/* Plans a detailed (interpolated) trajectory based on a spline.
Inputs: s_ahead: Vector of s values for path waypoints ahead of the last trajectory where the new spline passes through.
		d_ahead: Vector of d values for waypoints corresponding, together with s_ahead forms (s,d) waypoints
		max_speed: maximum speed that can be driven by the ego vehicle in the planned trajectory
*/
vector<vector<double>> SDCPlanner::spline_trajectory(vector<double> s_ahead, vector<double> d_ahead, double max_speed) {

	// First index: index of trajectory point (x,y)
	// Second index: x - 0, y - 1
	vector< vector<double> > trajectory;
	double ref_x, ref_y, ref_yaw, prev_x, prev_y;

	int prev_size = this->prev_path_x.size();

	// Stores (x,y) values of key points which the spline has to pass through
	vector<double> ipolate_x, ipolate_y;

	// Determines previous point from last timestep for spline calculation to make sure the spline is smooth.
	// If no previous path is available (prev_size < 2) then calculate a point backwards based on current heading
	if (prev_size < 2) {
		ref_x = this->x;
		ref_y = this->y;
		ref_yaw = this->yaw;
		prev_x = ref_x - cos(ref_yaw);
		prev_y = ref_y - sin(ref_yaw);
	}

	else {
		ref_x = this->prev_path_x[prev_size-1];
		ref_y = this->prev_path_y[prev_size-1];
		prev_x = this->prev_path_x[prev_size-2];
		prev_y = this->prev_path_y[prev_size-2];
		ref_yaw = atan2(ref_y - prev_y, ref_x - prev_x);
	}

	// First two points for the spline are the last two points from previous path
	ipolate_x.push_back(prev_x);
	ipolate_x.push_back(ref_x);
	ipolate_y.push_back(prev_y);
	ipolate_y.push_back(ref_y);

	// The next waypoints are generated from the (s,d) waypoints by transforming them to (x,y) coordinates
	for (int i = 0; i < s_ahead.size(); i++) {
		vector<double> xy_ahead = getXY(s_ahead[i], d_ahead[i]);
		ipolate_x.push_back(xy_ahead[0]);
		ipolate_y.push_back(xy_ahead[1]);
	}

	// Shift all points in the interpolation vector to a car-centric coordinate system
	for (int i = 0; i < ipolate_x.size(); i++) {
		double shift_x = ipolate_x[i] - ref_x;
		double shift_y = ipolate_y[i] - ref_y;

		ipolate_x[i] = (shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw));
		ipolate_y[i] = (shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw));
	}

	/* DEBUG: Prints out key spline points to make sure they are in ascending order
	for (int i=0; i < ipolate_x.size(); i++) {
		cout << "Spline X: " << ipolate_x[i] << endl;
	}
	*/

	// Calculates the spline based on key (x,y) pairs ipolate_x/y
	tk::spline s;
	s.set_points(ipolate_x, ipolate_y);

	// Add all remaining points of the last path to the current path to make sure it's smooth and less new points need to be generated
	for (int i=0; i < prev_size; i++) {
		trajectory.push_back({this->prev_path_x[i], this->prev_path_y[i]});
	}

	// cout << "Previous trajectory contained " << prev_size << " data points. Remaining ones will be newly created." << endl; // DEBUG

	// Path continues from the last planned point from previous path or from current point if no previous path exists
	double last_x = ref_x;
	double last_y = ref_y;
	double last_speed = this->speed_end_path;

	// Generate each "remaining" point in the trajectory, generate a new one.
	// For example, the last calculation was 100ms ago, and 5 waypoints were "used" up (5*20ms). 
	// Then, 5 new waypoints are added to the end of the previous path.
	for (int i = prev_size; i < (PATH_LENGTH); i++) {

		double cell_speed; // For every cell (from one waypoint to the next = 0.02 seconds) calculate the speed

		// If the maximum allowed speed is exceeded, decelerate with the maximum allowed acceleration
		if (last_speed > max_speed) {
			cell_speed = last_speed - MAX_ACCELERATION * TIME_INTERVAL;
		}
		// Otherwise, travel at max speed or, if not yet reached, accelerate until max speed is reached
		else {
			cell_speed = min(max_speed, last_speed + MAX_ACCELERATION * TIME_INTERVAL);
		}

		last_speed = cell_speed;

		// cout << "i = " << i << ". Current cell speed: " << cell_speed;

		// Since we are already in car-centric coordinates, the new delta x can be calculated by the distance that is traveled at the current speed
		// The corresponding y coordinate lies exactly on the interpolated spline, y = f(x)
		double x_rel = cell_speed * TIME_INTERVAL;
		double y_rel = s(x_rel);

		// cout << ", x_rel = " << x_rel << ", y_rel = " << y_rel;

		// Transform back to global coordinates
		double x_new = (x_rel * cos(ref_yaw) - y_rel * sin(ref_yaw));
		double y_new = (x_rel * sin(ref_yaw) + y_rel * cos(ref_yaw));

		last_x = last_x + x_new;
		last_y = last_y + y_new;

		// cout << ", x_new = " << last_x << ", y_new = " << last_y << endl;

		trajectory.push_back({last_x, last_y});
	}

	return trajectory;
}

/* Plans a very rough trajectory in s,d coordinates (less computationally expensive), to determine 
which next state is the most cost-efficient, which will then be planned in detail */
vector<vector<double>> SDCPlanner::plan_sd_trajectory(string state) {

	vector<vector<double>> sd_trajectory;

	// cout << "------ Calculating SD trajectory for state " << state << endl;

	double last_speed = this->speed;
	double max_speed;
	double last_s = this->s;
	double last_d = this->d;
	// Target lane determines the target d that the path should achieve
	int target_lane = this->current_lane;
	if (state == "LCL") {target_lane -= 1;}
	if (state == "LCR") {target_lane += 1;}

	double target_d = target_lane * 4 + 2;
	double deviation_d = target_d - last_d; // Relative distance from current d to the target
	int change_direction = deviation_d / abs(deviation_d); // -1 if right of target, +1 if left of target

	// cout << "Target d: " << target_d << ", deviation d: " << deviation_d << endl;

	// A trajectory of total duration of PREDICT_HORIZON is created, at an interval between path points of PREDICT_STEP
	for (int i = 0; i < (PREDICT_HORIZON / PREDICT_STEP); i++) {

		sd_trajectory.push_back({last_s, last_d});

		double cell_speed;
		double ov_dist_ahead = this->cars_ahead[target_lane].predictions[i][0] - last_s; // Relative distance to closest vehicle ahead in target lane
		double ov_speed_ahead = this->cars_ahead[target_lane].speed; // Speed of vehicle ahead in target lane
		max_speed = safe_speed(ov_dist_ahead, last_speed, ov_speed_ahead); // Maximum speed that is safe to drive so as to keep a safe distance to the vehicle ahead

		// cout << "Target lane reached " << target_lane_reached(target_lane, last_d) << ", distance ahead: " << ov_dist_ahead << ", max speed: " << max_speed << endl;

		// Accelerate or decelerate only when in target lane
		if ( (last_speed > max_speed) && target_lane_reached(target_lane, last_d) ) {
			cell_speed = last_speed - MAX_ACCELERATION * PREDICT_STEP;
		}
		else if ((last_speed < max_speed) && target_lane_reached(target_lane, last_d) ) {
			cell_speed = min(max_speed, last_speed + MAX_ACCELERATION * PREDICT_STEP);
		}
		else {
			cell_speed = last_speed;
		}

		last_speed = cell_speed;

		// delta_d is the change in lateral direction per timestep, equal to a turn angle of about 30° (sin 30° ~= 0.16)
		// If the distance to the target is smaller than delta_d, set this to delta_d to not overshoot the target d
		double delta_d = change_direction * min(cell_speed * 0.16 * PREDICT_STEP, abs(deviation_d));
		last_d = last_d + delta_d;
		deviation_d = target_d - last_d;

		// If a lateral movement happens, the longitudinal speed is reduced to keep the total vehicle speed at cell_speed
		double delta_s = PREDICT_STEP * sqrt( (cell_speed * cell_speed) - (delta_d * delta_d) );
		// cout << "Delta d = " << delta_d << ", cell speed = " << cell_speed << endl;
		last_s = last_s + delta_s;
	}

	return sd_trajectory;
}

// Calculates a cost for each (s,d) trajectory.
double SDCPlanner::cost_for_trajectory(vector<vector<double>> sd_trajectory) {

	double total_cost;

	// Cost for getting to a goal faster
	int t_len = sd_trajectory.size();
	double s_end = sd_trajectory[t_len-1][0];
	double delta_s = s_end - this->s; // total distance travelled

	// cout << "Current s: " << this->s << ", end s of trajectory: " << s_end << " => Delta s = " << delta_s << endl;

	// Linear interpolation between optimum value (move at speed limit) and zero, the faster, the lower the cost
	double cost_target = COST_MUL_DISTANCE * (1 - (delta_s / (SPEED_LIMIT * PREDICT_HORIZON)));
	total_cost += cost_target;

	// cout << "Target cost: " << cost_target;

	// Cost for staying away from other vehicles (safety)
	double d_target = sd_trajectory[t_len-1][1];
	int lane_target = floor(d_target / 4);
	double dist_min = 1000.0;

	// Loop through all forecast timepoints, in each timepoint calculate minimum distance
	// to the vehicles closest behind or ahead. Overall minimum distance will be stored
	// and used to calculate a safety score
	for (int i=0; i < t_len; i++) {
		double d_own = sd_trajectory[i][1];
		int lane_own = floor(d_own / 4);
		double s_own = sd_trajectory[i][0];
				
		// If target lane has not yet been reached, take also target lane into account, otherwise
		// check only current lane for collisions.
		// This avoids too small safety margins when overtaking (lateral distance one lane width)
		if (lane_own != lane_target) {
			double s_ahead = this->cars_ahead[lane_target].predictions[i][0]; // s of closest vehicle ahead at time i
			double d_ahead = this->cars_ahead[lane_target].predictions[i][1]; // d of closest vehicle ahead at time i
			double s_behind = this->cars_behind[lane_target].predictions[i][0]; // s of closest vehicle behind at time i
			double d_behind = this->cars_behind[lane_target].predictions[i][1]; // s of closest vehicle behind at time i			
			double dist_ahead = distance(s_own, d_own, s_ahead, d_ahead);
			double dist_behind = distance(s_own, d_own, s_behind, d_behind);
			if (dist_ahead < dist_min) {
				dist_min = dist_ahead;
			}
			if (dist_behind < dist_min) {
				dist_min = dist_behind;
			}
		}
		double s_ahead = this->cars_ahead[lane_own].predictions[i][0]; // s of closest vehicle ahead at time i
		double d_ahead = this->cars_ahead[lane_own].predictions[i][1]; // d of closest vehicle ahead at time i
		double dist_ahead = distance(s_own, d_own, s_ahead, d_ahead);
		if (dist_ahead < dist_min) {
			dist_min = dist_ahead;
		}
	}	

	// cout << ". Minimum distance during prediction forecast = " << dist_min;

	double safety_cost;
	// If safety minimum is breached, cost is 1, if the maximum exceeded, cost is 0
	if (dist_min < SAFETY_MINIMUM) { safety_cost = 1; }
	else if (dist_min > SAFETY_MAXIMUM) { safety_cost = 0; }
	else {
		// linear interpolation between (dist_min = MIN) => Cost 1, and (dist_min = MAX) => Cost 0
		safety_cost = (SAFETY_MAXIMUM - dist_min) / (SAFETY_MAXIMUM - SAFETY_MINIMUM);
	}
	safety_cost = safety_cost * COST_MUL_SAFETY;
	total_cost += safety_cost;

	// cout << ". Safety cost: " << safety_cost;

	// Cost for lane change: If lane is changed, cost is applied, else not
	double lane_change_cost = 0;
	if (abs(d_target - sd_trajectory[0][1]) > 2) {
		lane_change_cost += COST_LANE_CHANGE;
	}
	total_cost += lane_change_cost;

	// cout << ". Lane change cost: " << lane_change_cost << endl;

	return total_cost;
}

// Returns true if the last state update was at least STATE_UPDATE_CYCLE seconds ago, otherwise false
bool SDCPlanner::state_update_cycle() {
	bool update;
	this->state_update_time += this->delta_t;
	if (this->state_update_time > STATE_UPDATE_CYCLE) {
		update = true;
		this->state_update_time = 0;
	}
	else {
		update = false;
	}
	return update;
}

void SDCPlanner::init_cars_aheadbehind() {
	for (int i=0; i < NR_LANES; i++) {
		SDCPlanner::vehicle_track no_vehicle;
		vector< vector<double> > predictions;
		int lane = i;
		double speed = SPEED_LIMIT;
		double delta_s = 1000;
		double d = lane * 4 + 2;
		for (int i = 0; i < (PREDICT_HORIZON / PREDICT_STEP); i++) {
			predictions.push_back({this->s+1000, d});
		}
		this->cars_ahead.push_back({lane, speed, delta_s, predictions});
		this->cars_behind.push_back({lane, speed, delta_s, predictions});
	}
}

void SDCPlanner::reset_cars_aheadbehind() {
	for (int i=0; i < NR_LANES; i++) {
		SDCPlanner::vehicle_track no_vehicle;
		vector< vector<double> > predictions;
		int lane = i;
		double speed = SPEED_LIMIT;
		double delta_s = 1000;
		double d = lane * 4 + 2;
		for (int i = 0; i < (PREDICT_HORIZON / PREDICT_STEP); i++) {
			predictions.push_back({this->s+1000, d});
		}
		this->cars_ahead[i] = {lane, speed, delta_s, predictions};
		this->cars_behind[i] = {lane, speed, delta_s, predictions};
	}
}

bool SDCPlanner::target_lane_reached(int target_lane, double d) {
	double distance = abs(d - (target_lane * 4 + 2));
	if (distance < LANE_MID_DEVIATE) {
		return true;
	}
	else {
		return false;
	}
}

// Returns a safe speed given a vehicle's speed ahead, own current speed and the distance between own and other vehicle
// Target is to begin slowdown from a distance of 3 seconds of own speed behind other vehicle and to match other vehicle's speed
// at a distance of 2 seconds behind
double SDCPlanner::safe_speed(double distance_ahead, double own_speed, double other_speed) {
	double safe_distance = 1.5 * own_speed;
	double slow_down_distance = 2 * own_speed;
	double safe_speed;
	if (distance_ahead > slow_down_distance) {
		safe_speed = SPEED_LIMIT;
	}
	else if (distance_ahead < SAFETY_MINIMUM) {
		safe_speed = 0; // Emergency braking
	}
	else if (distance_ahead < safe_distance) {
		safe_speed = other_speed;
	}
	else {
		safe_speed = (SPEED_LIMIT - other_speed) / (slow_down_distance - safe_distance) * (distance_ahead - safe_distance) + other_speed;
	}
	return safe_speed;
}

// **************************************************************************************************************
// Calculates a Jerk Minimizing Trajectory given 
//	Input: a 3dim start vector [s, s_dot, s_dot_dot] or [d, d_dot, d_dot_dot] alternatively
//			as well as the corresponding end vector for s or d. Also, time T between start and end must be given
//	Output is a 6 dimensional vector of a0 to a5, as in s(t) = a0 + a1 * t + a2 * t² + ... + a5 * t⁵
// **************************************************************************************************************
vector<double> SDCPlanner::JMT(vector<double> start, vector<double> end, double T) {

    // Calculate right-hand side b of Ax = b equation
    Eigen::VectorXd b(3);
    b[0] = end[0] - (start[0] + start[1] * T + 1/2 * start[2] * T * T);
    b[1] = end[1] - (start[1] + start[2] * T);
    b[2] = end[2] - start[2];
    
    // Define matrix A
    Eigen::MatrixXd A(3, 3);
    double T_2 = T * T;
    double T_3 = T * T_2;
    double T_4 = T * T_3;
    double T_5 = T * T_4;
    
    A << T_3, T_4, T_5,
        3 * T_2, 4 * T_3, 5 * T_4,
        6 * T, 12 * T_2, 20 * T_3;
        
    // Solve A x = b equation system
    Eigen::VectorXd x(3);
    x = A.fullPivHouseholderQr().solve(b);
    // Alternative methods for solving the equation system below
    // x = A.llt().solve(b);
    // x = A.lldt().solve(b);
    // x = A.colPivHouseholderQr().solve(b);
    
    vector<double> a(6);
    a = {start[0], start[1], 0.5 * start[2], x[0], x[1], x[2]};
    
    return a;
}

int SDCPlanner::ClosestWaypoint(double x, double y) {

	double closestLen = 100000;
	int closestWaypoint = 0;

	for (int i = 0; i < this->map_wp_x.size(); i++)
	{
		double map_x = this->map_wp_x[i];
		double map_y = this->map_wp_y[i];
		double dist = distance(x, y, map_x, map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;
}

int SDCPlanner::NextWaypoint(double x, double y, double theta) {

	int closestWaypoint = ClosestWaypoint(x,y);

	double map_x = this->map_wp_x[closestWaypoint];
	double map_y = this->map_wp_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	// If closest waypoint is not within a reasonable range ahead
	// of the vehicle (< -45°, > +45°), take the next waypoint
	if (angle > (M_PI/4)) {
		closestWaypoint++;
	}

	return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> SDCPlanner::getFrenet(double x, double y, double theta) {

	int next_wp = NextWaypoint(x, y, theta);
	int prev_wp = next_wp-1;

	if(next_wp == 0) {
		prev_wp  = this->map_wp_x.size()-1;
	}

	double n_x = this->map_wp_x[next_wp] - this->map_wp_x[prev_wp];
	double n_y = this->map_wp_y[next_wp] - this->map_wp_y[prev_wp];
	double x_x = x - this->map_wp_x[prev_wp];
	double x_y = y - this->map_wp_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - this->map_wp_x[prev_wp];
	double center_y = 2000 - this->map_wp_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(this->map_wp_x[i], this->map_wp_y[i], this->map_wp_x[i+1], this->map_wp_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> SDCPlanner::getXY(double s, double d) {

	int prev_wp = -1;
	int wp_s_length = this->map_wp_s.size();

	while ( (s > this->map_wp_s[prev_wp+1]) && (prev_wp < (wp_s_length - 1)) )
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1) % wp_s_length;

	double heading = atan2((this->map_wp_y[wp2]-this->map_wp_y[prev_wp]), (this->map_wp_x[wp2]-this->map_wp_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - this->map_wp_s[prev_wp]);

	double seg_x = this->map_wp_x[prev_wp] + seg_s * cos(heading);
	double seg_y = this->map_wp_y[prev_wp] + seg_s * sin(heading);

	double perp_heading = heading - ( M_PI / 2.0 );

	double x = seg_x + d * cos(perp_heading);
	double y = seg_y + d * sin(perp_heading);

	return {x,y};
}

double SDCPlanner::deg2rad(double x) { return x * M_PI / 180; }
double SDCPlanner::rad2deg(double x) { return x * 180 / M_PI; }

double SDCPlanner::distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
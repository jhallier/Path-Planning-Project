#include <vector>
#include <string>
#include <chrono>
#include "constants.h"

class SDCPlanner {

public:

	double x, y, yaw, s, d;
	std::vector<double> map_wp_x, map_wp_y, map_wp_s, map_wp_dx, map_wp_dy;
	std::vector<double> prev_path_x, prev_path_y;
	double end_path_s, end_path_d;
	double speed, speed_end_path;
	std::chrono::steady_clock::time_point previous_time;
	double delta_t, state_update_time;
	int current_lane, target_lane;
	std::string state;

	struct vehicle_track {
		int lane;
		double speed;
		double delta_s;
		std::vector< std::vector<double> > predictions;
	};

	std::vector<SDCPlanner::vehicle_track> cars_ahead, cars_behind;

	SDCPlanner(const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

	virtual ~SDCPlanner();

	void update_own(double x, double y, double yaw, double s, double d, double car_speed, std::vector<double> previous_path_x, std::vector<double> previous_path_y, double end_path_s, double end_path_d);

	void predict(std::vector< std::vector<double> > sensor_data);

	void update_state();

	std::vector<std::vector<double>> trajectory();

	std::vector<std::vector<double>> plan_sd_trajectory(std::string state);

	std::vector<std::vector<double>> plan_trajectory(std::vector<double> s_ahead, 
						std::vector<double> d_ahead, double max_speed);
	
	std::vector<std::vector<double>> spline_trajectory(std::vector<double> s_ahead, 
						std::vector<double> d_ahead, double max_speed);

	double cost_for_trajectory(std::vector<std::vector<double>> sd_trajectory);

	bool state_update_cycle();

	void init_cars_aheadbehind();
	void reset_cars_aheadbehind();

	bool target_lane_reached(int target_lane, double d);

	double safe_speed(double distance_ahead, double own_speed, double other_speed);

	std::vector<double> JMT(std::vector<double> start, std::vector<double> end, double T);

	int NextWaypoint(double x, double y, double theta);
	int ClosestWaypoint(double x, double y);
	std::vector<double> getFrenet(double x, double y, double theta);
	std::vector<double> getXY(double s, double d);
	double deg2rad(double x);
	double rad2deg(double x);
	double distance(double x1, double y1, double x2, double y2);
};

const double SPEED_LIMIT = 22.0;     // In meters/second, equals 49.21 mph
const double MAX_ACCELERATION = 9.0; // in meters/second**2, about 1g
const double MAX_JERK = 50.0;         // in meters/second**3

const double PREDICT_HORIZON = 10; // How many second to predict other vehicle's movement in the future
const double PREDICT_STEP = 0.5; // Interval in seconds between prediction steps
const double TIME_INTERVAL = 0.02; // Time between each waypoint

const double COST_MUL_DISTANCE = 100; // Multiplier for distance travelled (normalized 0-1)
const double COST_MUL_SAFETY = 50; // Multiplier for closest distance to other vehicles (safety)
const double COST_LANE_CHANGE = 10; // Added punishment for changing a lane (slightly discouraged behavior)

const double SAFETY_MINIMUM = 6; // Minimum safe distance, below will incur safety cost of 100%
const double SAFETY_MAXIMUM = 30; // Maximum safe distance, above this safety cost will be 0%

const int PATH_LENGTH = 50; // Number of points per planned path

const int NR_LANES = 3; // Should usually be a dynamic number depending on the road. In this specific setting it's constant though
const double LANE_MID_DEVIATE = 0.2; // Lateral deviation in meters from middle of the lane to determine when lane change is successful
const double CHANGE_RATE_D = 0.8; // How many meters per second should d maximally change when changing lane?

const double STATE_UPDATE_CYCLE = 0.2; // How often is the state updated (in seconds)

const double MAX_S = 6945.554; // Maximum s before restarting at 0 (circular course)
// TrajectoryCalc.h - helper class to calculate trajectories for the shooter

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include "trajectory/model.h"
#include "trajectory/trajectory_table_defs.h"

#define ELEVATION_ANGLE_MAX 75_deg
#define ELEVATION_ANGLE_MIN 55_deg
#define ELEVATION_ANGLE_MID 65_deg

#define ROBOT_SPEED_THRESHOLD 0.01_fps  // minimum speed to enable speed compensation
#define ROBOT_MAX_SPEED 19.9_fps // worst case based on Kraken X60, no FOC, 6000 RPM, R3

#define HUB_DIAMETER 41.7_in
#define FUEL_DIAMETER 6_in
#define SUCCESSFUL_SHOT_RADIUS ((HUB_DIAMETER - FUEL_DIAMETER)/2.0)

#define WHEEL_DIAMETER 4_in

enum return_value_t {
	TRAJECTORY_SUCCESS = 0,
	TRAJECTORY_FAILURE_SHOT_SHORT = 1,
	TRAJECTORY_FAILURE_SHOT_LONG = 2,
	TRAJECTORY_FAILURE_SHOT_LEFT = 3,
	TRAJECTORY_FAILURE_SHOT_RIGHT = 4,
} ;

struct TrajectoryInfo {
	units::degree_t elevation_angle;
	units::revolutions_per_minute_t wheel_rpm;
	units::foot_t distance; // from shooter to the center of the hub
	units::feet_per_second_t vx; // field relative
	units::feet_per_second_t vy; // field relative
	units::degree_t turret_angle; // field relative
	units::degree_t hub_angle; // field relative
	units::second_t t; // flight time of fuel
	enum return_value_t return_value;
};

class TrajectoryCalc {

 public:

	TrajectoryCalc() {}
	~TrajectoryCalc() {}

	void init();
	enum model_t set_model(enum model_t model);
	enum model_t get_model();
	void set_shoot_on_the_move_enabled(bool enable);
	void set_constant_shooter_elevation(bool enable);
	double set_ball_compression(double compression);
	double get_ball_compression();
	units::degree_t get_angle(units::foot_t distance, units::revolutions_per_minute_t rpm);
	TrajectoryInfo compute_trajectory(TrajectoryInfo inputs);
	TrajectoryInfo find_best_launch_speed(TrajectoryInfo inputs, int theta_index);
	int get_v_launch_index(units::feet_per_second_t v_ball);
	int get_v_launch_index(double v_launch);
	units::feet_per_second_t v_launch_index_to_value(int v_launch_index);
	units::feet_per_second_t wheel_rpm_to_v_launch(units::revolutions_per_minute_t rpm);
	int wheel_rpm_to_v_launch_index(units::revolutions_per_minute_t rpm);
	units::revolutions_per_minute_t v_launch_index_to_wheel_rpm(int v_launch_index);

 private:
	enum model_t m_model;
	bool m_shoot_on_the_move_enabled;
	bool m_constant_shooter_elevation_enabled;
	double m_ball_compression;
	// bool debug_found_it;
} ;

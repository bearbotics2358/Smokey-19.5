// TrjectoryTable.h - class to hold the table of angle versus speed (actually wheel RPM) for the shooter

#include <units/velocity.h>
#include <units/angular_velocity.h>
#include "trajectory/trajectory_table_defs.h"

#define GRAVITY 32.17

class TrajectoryTable {

 public:

	TrajectoryTable(double wheel_diameter_in = 4.0, double shooter_height_in = 18.2817, double hub_height_in = 75.0);
	~TrajectoryTable() {}

	void init();
	int load_file();
	void set_wheel_diameter(double wheel_diameter_in);
	void set_shooter_height_in(double shooter_height_in);
	void set_hub_height(double hub_height_in);
	double get_wheel_diameter();
	double get_shooter_height_in();
	double get_hub_height();
	int get_rpm_min();
	int get_rpm_max();
	int get_rpm_inc();
	units::feet_per_second_t fuel_velocity(units::revolutions_per_minute_t rpm);

	double data[TRAJECTORY_TABLE_ANGLES][TRAJECTORY_TABLE_SPEEDS];

 private:

	double m_wheel_diameter_in;
	double m_shooter_height_in;
	double m_hub_height_in;

	int m_theta_min;
	int m_theta_max;
	int m_theta_inc;

	int m_rpm_min;
	int m_rpm_max;
	int m_rpm_inc;

} ;

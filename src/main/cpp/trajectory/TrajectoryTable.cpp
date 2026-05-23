// TrjectoryTable.cpp - class to hold the table of angle versus speed (actually wheel RPM) for the shooter

#if defined(_WIN32) || defined(_WIN64)
#define _USE_MATH_DEFINES
#endif

#include <math.h>
#include "trajectory/TrajectoryTable.h"

TrajectoryTable::TrajectoryTable(double wheel_diameter_in, double shooter_height_in, double hub_height_in) :
	m_wheel_diameter_in(wheel_diameter_in),
	m_shooter_height_in(shooter_height_in),
	m_hub_height_in(hub_height_in)
{

	m_theta_min = 0;
	m_theta_max = 90;
	m_theta_inc = 1;

	m_rpm_min = 1000;
	m_rpm_max = 4000;
	m_rpm_inc = 25;

}

// build the table based on physics of the trajectory, ignoring air resistance, etc.
void TrajectoryTable::init() {
	double theta = 0;
	int theta_indx = 0;
	double rpm = 0;
	int rpm_indx;
	double v = 0;
	double vh = 0;
	double vv = 0;
	double h;
	double d;
	double d1;
	double d2;
	double t;
	double t1;
	double t2;
	double b;
	
	for(theta_indx = 0; theta_indx < TRAJECTORY_TABLE_ANGLES; theta_indx++) {
		theta = m_theta_min + theta_indx * m_theta_inc;
		for(rpm_indx = 0; rpm_indx < TRAJECTORY_TABLE_SPEEDS; rpm_indx++) {
			rpm = m_rpm_min + rpm_indx * m_rpm_inc;
			v = M_PI * (m_wheel_diameter_in /12.0) * rpm / 60.0 / 2.0;
			vh = v * cos(theta / 180.0 * M_PI);
			vv = v * sin(theta / 180.0 * M_PI);
			h = vv * vv / (2 * GRAVITY);
			d1 = vh * vv / GRAVITY;
			t1 = vv / GRAVITY;
			
			// don't try to take the square root of a negative number
			b = 2.0 * (h - ((m_hub_height_in - m_shooter_height_in) / 12.0)) / GRAVITY;
			if(b > 0) {
				t2 = sqrt(b);
				d2 = vh * t2;
				t = t1 + t2;
				d = d1 + d2;
			} else {
				t2 = 0;
				d2 = 0;
				t = 0;
				d = 0;
			}
			data[theta_indx][rpm_indx] = d;
		}
	}
}

// placeholder for reading the table from a file
int TrajectoryTable::load_file() {
	return 0;
}

void TrajectoryTable::set_wheel_diameter(double wheel_diameter_in) {
	m_wheel_diameter_in = wheel_diameter_in;
}

void TrajectoryTable::set_shooter_height_in(double shooter_height_in) {
	m_shooter_height_in = shooter_height_in;
}

void TrajectoryTable::set_hub_height(double hub_height_in) {
	m_hub_height_in = hub_height_in;
}

double TrajectoryTable::get_wheel_diameter() {
	return m_wheel_diameter_in;
}

double TrajectoryTable::get_shooter_height_in() {
	return m_shooter_height_in;
}

double TrajectoryTable::get_hub_height() {
	return m_hub_height_in;
}

int TrajectoryTable::get_rpm_min() {
	return m_rpm_min;
}

int TrajectoryTable::get_rpm_max() {
	return m_rpm_max;
}

int TrajectoryTable::get_rpm_inc() {
	return m_rpm_inc;
}

units::feet_per_second_t TrajectoryTable::fuel_velocity(units::revolutions_per_minute_t rpm) {
	units::feet_per_second_t v = (units::feet_per_second_t)(M_PI * (m_wheel_diameter_in / 12.0) * rpm.value() / 60.0 / 2.0);	

	return v;
}

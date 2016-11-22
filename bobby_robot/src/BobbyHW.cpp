#include "BobbyHW.hpp"
#include <iostream>

using namespace std;

#define	PWM0_PIN	26

#define DEBUG(fmt, args...) fprintf(stdout, "D: %s - " fmt, __func__, ##args)
#define ERROR(fmt, args...) fprintf(stderr, "E: %s - " fmt, __func__, ##args)
#define PI	3.141592653589793f

BobbyHW::BobbyHW(ros::NodeHandle &node, double wheel_radius, double width,
		int32_t ticks_per_rotation) :
	L(width), R(wheel_radius)
{
	DEBUG("Constructing our RobotHW with wheels of radius of" \
		" %f meter and base width of %f meter\n", R, L);
	rtick = 0;
	ltick = 0;
	
	double rotation_length = 2 * PI * R;
	ticks_per_meter = ticks_per_rotation / rotation_length;
	
	tick_timer = node.createTimer(ros::Duration(0.1),
			&BobbyHW::tick_timer_callback, &(*this));
	
	try {
		pwm = new mraa::Pwm(PWM0_PIN);
		if (pwm == NULL) {
			ERROR("Cannot create the mraa::pwm object\n");
		} else pwm->enable(true);
	} catch (std::invalid_argument ia) {
		ERROR("Cannot create the mraa::pwm object: %s\n", ia.what());
		pwm = NULL;
	}
}

BobbyHW::~BobbyHW()
{
	DEBUG("Destructing our RobotHW\n");
	if (pwm) delete pwm;
	pwm = NULL;
}

void BobbyHW::tick_timer_callback(const ros::TimerEvent& event)
{
	//DEBUG("Update the ticks...\n");
	current_time = event.current_real;
	
	read_sensors();
	
	double dt = old_time.toSec() - current_time.toSec();
	vr = (old_rtick - rtick) * ticks_per_meter / dt;
	vl = (old_ltick - ltick) * ticks_per_meter / dt;
	
	old_ltick = ltick;
	old_rtick = rtick;
	old_time = current_time;
}

void BobbyHW::read_sensors()
{
	//DEBUG("Read the sensors...\n");
	/* update ticks */	
	ltick = ltick + 1;
	rtick = rtick + 1;
}

void BobbyHW::write_left_wheel_velocity(double vl)
{
	DEBUG("Setting left wheel velocity to %f\n", vl);
	if (pwm) pwm->write(vl);
}

void BobbyHW::write_right_wheel_velocity(double vr)
{
	DEBUG("Setting right wheel velocity to %f\n", vr);
	if (pwm) pwm->write(vr);
}

void BobbyHW::update(double linear_vel, double angular_vel) {
	
	DEBUG("Updating linear and angular velocities to (%f, %f)\n",
		linear_vel, angular_vel);
	float vr = (2.0f * linear_vel + angular_vel * L) / (2.0f * R);
	float vl = (2.0f * linear_vel - angular_vel * L) / (2.0f * R);
	write_right_wheel_velocity(vr);
	write_left_wheel_velocity(vl);
}

double BobbyHW::get_linear_velocity() {

	return R * (vr + vl) / 2.0f;
}

double BobbyHW::get_angular_velocity_degrees() {

	return get_angular_velocity_radians() * PI / 180.0f;
}

double BobbyHW::get_angular_velocity_radians() {

	double dt = old_time.toSec() - current_time.toSec();
	double vr = (old_rtick - rtick) * ticks_per_meter / dt;
	double vl = (old_ltick - ltick) * ticks_per_meter / dt;
	return R * (vr - vl) / L;
}

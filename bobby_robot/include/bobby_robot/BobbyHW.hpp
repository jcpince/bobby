#ifndef __BOBBYHW_HPP__
#define __BOBBYHW_HPP__

#include <stdint.h>
#include <stdio.h>

#include "ros/ros.h"

#include "mraa.hpp"

using namespace ros;

class BobbyHW {
	
	public:
		BobbyHW(ros::NodeHandle &node, double wheel_radius, double width,
			int32_t ticks_per_rotation);
		~BobbyHW();
		
		void update(double linear_vel, double angular_vel);
		void read_sensors();
		double get_linear_velocity();
		double get_angular_velocity_radians();
		double get_angular_velocity_degrees();
	
	private:
		mraa::Pwm* pwm;
		double	R;					/* Wheels radius in meters */
		double	L;					/* Cart width in meters */
		double	ticks_per_meter;	/* Encoder ticks per meter */
		
		int64_t	rtick;				/* Right tick measure */
		int64_t	ltick;				/* Left tick measure */
		int64_t	old_rtick;			/* Right tick measure */
		int64_t	old_ltick;			/* Left tick measure */
		Time	old_time;			/* time of previous ticks measure */
		Time	current_time;		/* time of current ticks measure */
		double  vr;
		double  vl;
		
		Timer tick_timer;			/* timer used to update the ticks */

		void tick_timer_callback(const ros::TimerEvent& event);
		void write_left_wheel_velocity(double vl);
		void write_right_wheel_velocity(double vr);
};

#endif /* __BOBBYHW_HPP__ */

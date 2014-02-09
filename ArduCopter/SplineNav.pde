/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//////////////////////////////////////////////////////////
// SplineNav Spline Navigation Controller for ArduCopter 3.1
// Version 0.4
// Created by David Dewey <david@mavbot.com>
// 
// License: GNU 2.1 or later (See COPYING.txt)
//////////////////////////////////////////////////////////

#include "SplineNav.h"

// start -- begin the spline WP navigation
void SplineNav::start(const Vector3f &current_position, float heading_in_radians)
{
    spline_t = spline_t_sq = 0.0f;
    spline_timer = spline_slow_timer = hal.scheduler->micros();
    spline_dt = 0.0f;
    spline_step = 0;
    spline_next_cmd_index = 0;
    spline_end = false;

    // set spline p0, p1, p2, and p0 prime
    spline_p0 = spline_start_point = current_position;
    spline_p1 = next_spline_waypoint();
    spline_p2 = next_spline_waypoint();
    
    // allow for a smooth transition into spline navigation
    Vector3f diff = spline_p1 - spline_p0;
    diff.z = 0.0f;
    float xy_dist = diff.length();
    spline_p0_prime.x = xy_dist * cosf(heading_in_radians);
    spline_p0_prime.y = xy_dist * sinf(heading_in_radians);
    spline_p0_prime.z = 0.0f;

    // initialise other variables
    spline_target_speed = requested_target_speed = spline_acceleration = 0.0;
    t_speed = 0.0f;
    spline_max_speed = wp_nav.get_horizontal_velocity();
    spline_max_climb = wp_nav.get_climb_velocity();
    spline_max_descent = -wp_nav.get_descent_velocity();
    spline_max_acceleration = wp_nav.get_waypoint_acceleration();
    spline_max_curve_accel = spline_max_acceleration * SPLINE_CURVE_ACCEL_MULTIPLE;
    
    // make the derivative at the first waypoint what it would be if the
    // copter was actually starting exactly on the first waypoint, so the curve after
    // the first waypoint is always the same, no matter where the copter actually starts from
    spline_p1_prime = (spline_p2 - spline_p1) * 2.0f / SPLINE_TENSION;
    initialize_spline_segment(false); // false to not use default p1_prime calculation
    
    // initialize yaw settings
    spline_nav_yaw = wrap_360_cd(heading_in_radians * DEGX100);
    spline_heading_new = spline_heading_radians = heading_in_radians;
    spline_yaw_rate = 0.0f;
    
    // initialise loiter target.  Note: feed forward velocity set to zero
    wp_nav.init_loiter_target(current_position, Vector3f(0,0,0));
}

// update spline_nav, setting xy derivative, yaw heading
// return spline yaw rate in centidegrees per second
// to be called at 100 Hz
void SplineNav::update_yaw() {
    float dt = update_spline_t(); // moving forward on the spline curve
    evaluate_spline_derivative_xy();
    int32_t new_nav_yaw = spline_nav_yaw;
    if (has_horizontal_component(spline_derivative)) { // ignore for zero length or vertical derivative
        spline_heading_new = fast_atan2(spline_derivative.y, spline_derivative.x);
        new_nav_yaw = spline_heading_new * DEGX100;
    }
    

    // yaw rate based simply on difference in successive headings; use spline second derivative when we get more CPU power
    float yaw_rate = (float)wrap_180_cd(new_nav_yaw - spline_nav_yaw)
            * SPLINE_YAW_RATE_BOOST / dt;
    
    // remove yaw rate spikes at waypoints
    yaw_rate = constrain_float(yaw_rate, -SPLINE_MAX_YAW_RATE, SPLINE_MAX_YAW_RATE);
    
    // smooth yaw rate
    spline_yaw_rate = 0.93f * spline_yaw_rate + 0.07f * yaw_rate;
    
    spline_nav_yaw = wrap_360_cd(new_nav_yaw);
}

// spline position controller's main call which in turn
// calls loiter controller with updated target position
// should be called at 100 Hz
void SplineNav::update()
{
    // for smooth video yaw update needs to be fast, every cycle, at
    // least until we have a 3-axis gimbal to work with.
    // No need to update fast if we're in manual yaw control.
    if (yaw_mode==YAW_SPLINE) update_yaw(); 

    switch (spline_step) {
        case 0:
            // compute next target for loiter controller
            evaluate_spline();
            break;
        case 1:
            // compute curve derivative z since fast update skips this
            evaluate_spline_derivative_z();
            break;
        case 2:
            get_t_speed(); // compute t_speed scale factor based on derivative length
            break;
        case 3:
            constrain_speed();
            break;
        case 4:
            // apply final segment slowdown speed request
            if (spline_end) spline_final_slowdown();
            // take care of the various speed requests
            apply_requested_speed();
            break;
        case 5:
            // call loiter controller
            spline_chase_speed = wp_nav.update_spline_velocity(spline_target, spline_dt);
            break;
        case 6:
            // call loiter controller
            wp_nav.update_spline_acceleration(spline_dt);
            break;
        case 7:
            // call loiter controller
            wp_nav.update_spline_lean_angles();
        case 8:
            {
                // accelerate or deccelerate target speed based on how far we're lagging loiter target position/altitude
                float requested_spline_acceleration =
                        spline_max_acceleration *
                        (1.0f - spline_chase_speed / spline_max_speed);
                request_target_speed(
                        spline_target_speed + requested_spline_acceleration * spline_dt);
            }
            break;
        case 9:
            {   
                uint32_t slow_timer = hal.scheduler->micros();
                spline_dt = (float)(slow_timer - spline_slow_timer) / 1000000.f;
                spline_slow_timer = slow_timer;
                check_pilot_yaw_control();
            }
            break;
    }
    spline_step++;
    spline_step %= 10;
}

// check pilot manual yaw control to switch back to YAW_SPLINE
void SplineNav::check_pilot_yaw_control() {
    // special case for when pilot is controlling heading manually
    if (yaw_mode==YAW_HOLD) {
        
        // update yaw called here at the slow rate since the
        // YAW_HOLD mode doesn't need the fast yaw
        // control and doesn't call fast_update every cycle.
        update_yaw();
        
        // SplineNav takes back yaw control if stick yaw input is neutral AND pilot has directed yaw
        // back to the approx. direction of SplineNav curve tangent.
        if (g.rc_4.control_in == 0 &&
                labs(wrap_180_cd(spline_nav_yaw - ahrs.yaw_sensor)) <
                SPLINE_YAW_TAKEBACK_THRESHOLD)
        {
            set_yaw_mode(YAW_SPLINE);
            spline_yaw_rate = 0.0f;
        }
    }
}

// contrain speed along curve according to max climb/decent and accel limits
void SplineNav::constrain_speed() {
    // calculate our climb/decent rate at next point
    float climb_rate = spline_derivative.z * t_speed;
    
    // constrain speed to not exceed max configured climb speed 
    if (climb_rate > spline_max_climb)
        request_target_speed(spline_target_speed * (spline_max_climb/climb_rate));
    // constrain speed to not exceed max configured decent speed
    else if (climb_rate < spline_max_descent)
        request_target_speed(spline_target_speed * (spline_max_descent/climb_rate));
 
    // Constrain speed so that cetripetal acceleration in the horizontal does not exceed max acceleration.
    // When we get more CPU power, change this to use spline second derivative to derive angular velocity,
    // instead of using difference of sucessive headings.
    float spline_angular_velocity = spline_dt > 0.0f ?
            wrap_PI(spline_heading_new - spline_heading_radians) / spline_dt : 0.0f;
    float angular_speed = fabsf(spline_angular_velocity);
    if (angular_speed * spline_target_speed > spline_max_curve_accel) {
        request_target_speed(spline_max_curve_accel / angular_speed);
    }
    spline_heading_radians = spline_heading_new;
}

// update spline_t; returns time since previous update.
float SplineNav::update_spline_t() {
    uint32_t timer = hal.scheduler->micros();
    float dt = (float)(timer - spline_timer) / 1000000.f;
    spline_timer = timer;
    spline_t += t_speed * dt;
    if (spline_t >= 1.0f && !spline_end) next_spline_segment();
    spline_t_sq = spline_t * spline_t;
    return dt;
}

//constrain the requested target speed and set target speed
void SplineNav::apply_requested_speed() {

    // constrain spline acceleration to a magnitude no larger than SPLINE_ACCELERATION,
    // and a change no greater than SPLINE_JERK * dt
    float requested_spline_acceleration = spline_dt > 0.0f ?
            (requested_target_speed - spline_target_speed) / spline_dt : 0.0f;
    requested_spline_acceleration = constrain_float(
            requested_spline_acceleration, -spline_max_acceleration, spline_max_acceleration);
    spline_acceleration = constrain_float(
            requested_spline_acceleration,
            spline_acceleration - SPLINE_JERK * spline_dt,
            spline_acceleration + SPLINE_JERK * spline_dt
    );
    // apply acceleration
    spline_target_speed += spline_dt * spline_acceleration;
    spline_target_speed = constrain_float(spline_target_speed, 0.0f, spline_max_speed);
    get_t_speed();
    // reset requested_target_speed
    requested_target_speed = spline_max_speed;
}

// perform initialization in preparation for the new spline segment
void SplineNav::initialize_spline_segment(bool set_p1_prime) {

    // derivative of spline at p1 is based on difference of previous and next points
    if (set_p1_prime) spline_p1_prime = (spline_p2 - spline_p0) / SPLINE_TENSION;

    // compute a and b vectors used in spline formula
    spline_a = spline_p0*2.0f - spline_p1*2.0f + spline_p0_prime + spline_p1_prime;
    spline_b = spline_p1*3.0f - spline_p0*3.0f - spline_p0_prime*2.0f - spline_p1_prime;
}

// continue to the next spline segment
void SplineNav::next_spline_segment() {
    // start t back at near the beginning of the new segment
    spline_t -= 1.0f;

    // p1 becomes the next p01, p2 the next p1, etc.
    spline_p0 = spline_p1;
    spline_p1 = spline_p2;
    spline_p0_prime = spline_p1_prime;
    spline_p2 = next_spline_waypoint();

    initialize_spline_segment();
}

// get the next spline waypoint
Vector3f SplineNav::next_spline_waypoint() {
    struct Location command;
    // search for the next WAYPOINT command
    // todo: recognize some other commands as well, for example ROI would be nice
    
    int next_cmd_index = spline_next_cmd_index;
    int n = g.command_total.get()-1; // total number of commands
    for (int i=0; i<n; i++) { // loop for a max of total number of commands
        if (next_cmd_index < n) next_cmd_index++;
        else if (SPLINE_LOOP) {
            next_cmd_index=1;
        }
        else { // out of commands
            spline_end = true; // spline curve is ending
            
            // reset to previous index to return the last waypoint again
            next_cmd_index = spline_next_cmd_index;
        }
        command = get_cmd_with_index(next_cmd_index);
        if (command.id == MAV_CMD_NAV_WAYPOINT) {
            spline_next_cmd_index = next_cmd_index;
            return pv_location_to_vector(command);
        }
    }

    // give up and just return spline start point if we don't have any waypoints
    return spline_start_point;
}

#ifndef PIBY2_FLOAT
#define PIBY2_FLOAT  1.5707963f
#endif
// from https://gist.github.com/volkansalma/2972237/raw/
// |error| < 0.005
float fast_atan2( float y, float x )
{
	if ( x == 0.0f )
	{
		if ( y > 0.0f ) return PIBY2_FLOAT;
		if ( y == 0.0f ) return 0.0f;
		return -PIBY2_FLOAT;
	}
	float atan;
	float z = y/x;
	if ( fabs( z ) < 1.0f )
	{
		atan = z/(1.0f + 0.28f*z*z);
		if ( x < 0.0f )
		{
			if ( y < 0.0f ) return atan - PI;
			return atan + PI;
		}
	}
	else
	{
		atan = PIBY2_FLOAT - z/(z*z + 0.28f);
		if ( y < 0.0f ) return atan - PI;
	}
	return atan;
}

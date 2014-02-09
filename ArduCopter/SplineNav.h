/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//////////////////////////////////////////////////////////
// SplineNav Spline Navigation Controller for ArduCopter 3.1
// Version 0.4
// Created by David Dewey <david@mavbot.com>
// 
// License: GNU 2.1 or later (See COPYING.txt)
//////////////////////////////////////////////////////////

#ifndef SPLINENAV_H
#define SPLINENAV_H

// higher tension makes SplineNav curve more tightly at waypoints, but straighter in between waypoints.
// 2.0 makes it a Catmull-Rom Spline. 1.5 to 1.7 gives nice loose curves for smooth video.
#define SPLINE_TENSION 1.6f

// use this multiple of waypoint nav acceleration as the maximum curve cetripetal acceleration
// SplineNav attempts to maintain.
#define SPLINE_CURVE_ACCEL_MULTIPLE 2.0f

// how fast SplineNav increases or descreases acceleration as it flies the curve in cm/s^3
#define SPLINE_JERK 200.0f

// set to true to make SplineNav fly a closed loop
#define SPLINE_LOOP false

// max yaw overshoot in cd for spline yaw controller
#define SPLINE_MAX_YAW_OVERSHOOT 4000

// scale factor for yaw angle error curve
#define SPLINE_ANGLE_ERROR_SCALE 1000

// spline max yaw rate to reject yaw jerks when passing a waypoint, cd/s
#define SPLINE_MAX_YAW_RATE 12000.0f

// yaw rate boost factor to avoid small yaw jitter
#define SPLINE_YAW_RATE_BOOST 1.04f

// threshold of yaw alignment with SplineNav's curve tangent
// after which SplineNav takes back yaw control if there is no
// yaw stick input from user
#define SPLINE_YAW_TAKEBACK_THRESHOLD 1200

class SplineNav
{
public:

    // start the spline WP navigation
    void start(const Vector3f &current_position, float heading_in_radians);

    // spline position controller's main call which in turn
    // calls loiter controller with updated target position
    void update();
    
    // current rate of yaw for yaw controller
    int32_t get_yaw_rate() {
        return (int32_t)spline_yaw_rate;
    }
    
    int32_t get_yaw() {
        return spline_nav_yaw;
    }

protected:
    
    // update spline_nav, setting derivative, yaw heading
    void update_yaw();
    
    // time of last update to spline_t
    uint32_t spline_timer;
    
    // time of last slow update
    uint32_t spline_slow_timer;

    // which portion of splinenav controller to run during this cycle
    uint8_t spline_step;

    // time between spline nav updates
    float spline_dt;
    
    // required speed to keep up with target
    float spline_chase_speed;
    
    // current target speed for motion along spline curve
    float spline_target_speed;
    
    // proposed new target speed according to accel, climb, decend, and target/altitude lag constraints
    float requested_target_speed;
    
    // acceleration used to increase target speed each cycle
    float spline_acceleration;
    
    // max acceleration for WP navigation
    float spline_max_acceleration;
    
    // max centripetal acceleration
    float spline_max_curve_accel;

    // current t parameter on spline segment from 0.0 to 1.0
    float spline_t;
    
    // store spline t squared, stored for faster processing
    float spline_t_sq;
    
    // factor to scale spline_t progression as we follow the curve
    // to get the correct vehicle speed
    float t_speed;
    
    // spline heading in radians
    float spline_heading_radians;
    
    // latest spline heading to compare with spline_heading_radians
    float spline_heading_new;
    
    // spline nav yaw
    int32_t spline_nav_yaw;
    
    // spline yaw rate
    float spline_yaw_rate;
    
    // max climb rate
    float spline_max_climb;
    
    // max descent rate (always a negative number)
    float spline_max_descent;
    
    // max wp speed
    float spline_max_speed;

    // index of next command to read from storage
    int spline_next_cmd_index;
    
    // true when we've run out of additional waypoints in our track
    bool spline_end;
    
    // some helpful vectors
    Vector3f spline_start_point;
    Vector3f spline_p0;
    Vector3f spline_p1;
    Vector3f spline_p2;
    Vector3f spline_p0_prime;
    Vector3f spline_p1_prime;
    Vector3f spline_a;
    Vector3f spline_b;
    Vector3f spline_target;
    Vector3f spline_derivative;
    
    // check for pilot manual yaw control
    void check_pilot_yaw_control();

    // contrain speed along curve according to max climb/decent and accel limits
    void constrain_speed();
    
    // update spline_t; returns time since previous update.
    float update_spline_t();
    
    // perform initialization in preparation for the new spline segment
    void initialize_spline_segment(bool set_p1_prime=true);

    // continue to the next spline segment
    void next_spline_segment();

    // get the next spline waypoint
    Vector3f next_spline_waypoint();

    //evaluate spline formula at point t
    void evaluate_spline() {
        // evaluate spline t cubed
        float t_cu = spline_t_sq * spline_t;
        spline_target = spline_a * t_cu + spline_b * spline_t_sq + spline_p0_prime * spline_t + spline_p0;
    }

    // evaluate spline derivative at point t
    // not used for now, but useful in the future when we have a faster processor
    /*void evaluate_spline_derivative() {
        float t_sq = spline_t * spline_t;
        spline_derivative = spline_a*3.0f*t_sq + spline_b*2.0f*spline_t + spline_p0_prime;
    }*/
    
    // evaluate xy components of spline derivative
    // ignore z for higher processing speed, as z doesn't need to be calculated as often as xy
    void evaluate_spline_derivative_xy() {
        spline_derivative.x = spline_a.x*3.0f*spline_t_sq + spline_b.x*2.0f*spline_t + spline_p0_prime.x;
        spline_derivative.y = spline_a.y*3.0f*spline_t_sq + spline_b.y*2.0f*spline_t + spline_p0_prime.y;
    }
    
    // evaluate z component of spline derivative
    // ignore xy for higher processing speed, as xy is already computed in the fast update
    void evaluate_spline_derivative_z() {
        spline_derivative.z = spline_a.z*3.0f*spline_t_sq + spline_b.z*2.0f*spline_t + spline_p0_prime.z;
    }
    
    // evaluate spline second derivative at point t
    // not used for now, but useful in the future when we have a faster processor
    /*void evaluate_spline_2nd_derivative() {
        spline_2nd_derivative = spline_a*6.0*spline_t + spline_b*2.0;
    }*/
    
    //compute scale factor for spline t speed of increase as we move along the curve
    void get_t_speed() {
        float spline_derivative_length = spline_derivative.length();
        if (spline_derivative_length > 0.0f) // avoid divide by zero
            // factor to scale the velocity as we move along the spline curve
            t_speed = spline_target_speed / spline_derivative_length;
    }
    
    // evaluate magnitude of centripetal acceleration at current t on spline curve
    // heading must be a tangential vector
    // not used for now, but useful in the future when we have a faster processor
    /*float spline_centripetal_acceleration(float t, const Vector3f &heading) const {
        Vector3f spline_2nd_derivative = evaluate_spline_2nd_derivative(t);
        return (spline_2nd_derivative - spline_2nd_derivative.projected(heading)).length();
    }*/
    
    // if a speed constraint results in a reduced target speed, set the target speed request,
    // only if the new request is lower than the existing one
    void request_target_speed(float speed) {
        if (speed < requested_target_speed) requested_target_speed = speed;
    }
    
    // constrain the requested target speed and set target speed
    void apply_requested_speed();
    
    // slow down on the last segment
    void spline_final_slowdown() {
        float speed_factor = 1.0f - spline_t;
        speed_factor = constrain_float(speed_factor, 0.0f, 1.0f);
        request_target_speed(speed_factor * spline_max_speed);
    }
    
    // see if a vector has any non-z component
    static bool has_horizontal_component(const Vector3f &v) {
        return v.x != 0.0 || v.y != 0.0;
    }

};
#endif  // SPLINENAV_H

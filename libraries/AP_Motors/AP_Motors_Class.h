// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_MOTORS_CLASS_H__
#define __AP_MOTORS_CLASS_H__

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <AP_Notify.h>      // Notify library
#include <AP_Curve.h>       // Curve used to linearlise throttle pwm to thrust
#include <RC_Channel.h>     // RC Channel Library

// offsets for motors in motor_out, _motor_filtered and _motor_to_channel_map arrays
#define AP_MOTORS_MOT_1 0
#define AP_MOTORS_MOT_2 1
#define AP_MOTORS_MOT_3 2
#define AP_MOTORS_MOT_4 3
#define AP_MOTORS_MOT_5 4
#define AP_MOTORS_MOT_6 5
#define AP_MOTORS_MOT_7 6
#define AP_MOTORS_MOT_8 7

#define APM1_MOTOR_TO_CHANNEL_MAP CH_1,CH_2,CH_3,CH_4,CH_7,CH_8,CH_10,CH_11
#define APM2_MOTOR_TO_CHANNEL_MAP CH_1,CH_2,CH_3,CH_4,CH_5,CH_6,CH_7,CH_8

#define AP_MOTORS_MAX_NUM_MOTORS 8

#define AP_MOTORS_DEFAULT_MIN_THROTTLE  130
#define AP_MOTORS_DEFAULT_MID_THROTTLE  500
#define AP_MOTORS_DEFAULT_MAX_THROTTLE  1000

// APM board definitions
#define AP_MOTORS_APM1  1
#define AP_MOTORS_APM2  2

// frame definitions
#define AP_MOTORS_PLUS_FRAME        0
#define AP_MOTORS_X_FRAME           1
#define AP_MOTORS_V_FRAME           2
#define AP_MOTORS_H_FRAME           3   // same as X frame but motors spin in opposite direction
#define AP_MOTORS_NEW_PLUS_FRAME    10  // NEW frames are same as original 4 but with motor orders changed to be clockwise from the front
#define AP_MOTORS_NEW_X_FRAME       11
#define AP_MOTORS_NEW_V_FRAME       12
#define AP_MOTORS_NEW_H_FRAME       13   // same as X frame but motors spin in opposite direction

// motor update rate
#define AP_MOTORS_SPEED_DEFAULT     490 // default output rate to the motors

#define THROTTLE_CURVE_ENABLED      1   // throttle curve disabled by default
#define THROTTLE_CURVE_MID_THRUST   52  // throttle which produces 1/2 the maximum thrust.  expressed as a percentage of the full throttle range (i.e 0 ~ 100)
#define THROTTLE_CURVE_MAX_THRUST   93  // throttle which produces the maximum thrust.  expressed as a percentage of the full throttle range (i.e 0 ~ 100)

#define AP_MOTORS_SPIN_WHEN_ARMED   70  // spin motors at this PWM value when armed

// bit mask for recording which limits we have reached when outputting to motors
#define AP_MOTOR_NO_LIMITS_REACHED  0x00
#define AP_MOTOR_ROLLPITCH_LIMIT    0x01
#define AP_MOTOR_YAW_LIMIT          0x02
#define AP_MOTOR_THROTTLE_LIMIT     0x04
#define AP_MOTOR_ANY_LIMIT          0xFF

// slow start increments - throttle increase per (100hz) iteration.  i.e. 5 = full speed in 2 seconds
#define AP_MOTOR_SLOW_START_INCREMENT           10      // max throttle ramp speed (i.e. motors can reach full throttle in 2 seconds)
#define AP_MOTOR_SLOW_START_LOW_END_INCREMENT   2       // min throttle ramp speed (i.e. motors will speed up from zero to _spin_when_armed speed in about 1 second)

/// @class      AP_Motors
class AP_Motors {
public:

    // Constructor
    AP_Motors( RC_Channel* rc_roll, RC_Channel* rc_pitch, RC_Channel* rc_throttle, RC_Channel* rc_yaw, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT);

    // init
    virtual void        Init();

    // set mapping from motor number to RC channel
    void                set_motor_to_channel_map( uint8_t mot_1, uint8_t mot_2, uint8_t mot_3, uint8_t mot_4, uint8_t mot_5, uint8_t mot_6, uint8_t mot_7, uint8_t mot_8 ) {
        _motor_to_channel_map[AP_MOTORS_MOT_1] = mot_1;
        _motor_to_channel_map[AP_MOTORS_MOT_2] = mot_2;
        _motor_to_channel_map[AP_MOTORS_MOT_3] = mot_3;
        _motor_to_channel_map[AP_MOTORS_MOT_4] = mot_4;
        _motor_to_channel_map[AP_MOTORS_MOT_5] = mot_5;
        _motor_to_channel_map[AP_MOTORS_MOT_6] = mot_6;
        _motor_to_channel_map[AP_MOTORS_MOT_7] = mot_7;
        _motor_to_channel_map[AP_MOTORS_MOT_8] = mot_8;
    }

    // set update rate to motors - a value in hertz
    virtual void        set_update_rate( uint16_t speed_hz ) { _speed_hz = speed_hz; };

    // set frame orientation (normally + or X)
    virtual void        set_frame_orientation( uint8_t new_orientation ) { _flags.frame_orientation = new_orientation; };

    // enable - starts allowing signals to be sent to motors
    virtual void        enable() = 0;

    // arm, disarm or check status status of motors
    bool                armed() { return _flags.armed; };
    void                armed(bool arm);

    // set_min_throttle - sets the minimum throttle that will be sent to the engines when they're not off (i.e. to prevents issues with some motors spinning and some not at very low throttle)
    void                set_min_throttle(uint16_t min_throttle);
    // set_mid_throttle - sets the mid throttle which is close to the hover throttle of the copter
    // this is used to limit the amount that the stability patch will increase the throttle to give more room for roll, pitch and yaw control
    void                set_mid_throttle(uint16_t mid_throttle);

    // output - sends commands to the motors
    void                output();

    // output_min - sends minimum values out to the motors
    virtual void        output_min() = 0;

    // motor test
    virtual void        output_test() = 0;

    // throttle_pass_through - passes pilot's throttle input directly to all motors - dangerous but used for initialising ESCs
    virtual void        throttle_pass_through();

	// setup_throttle_curve - used to linearlise thrust output by motors
    //      returns true if curve is created successfully
	bool                setup_throttle_curve();

    // 1 if motor is enabled, 0 otherwise
    bool                motor_enabled[AP_MOTORS_MAX_NUM_MOTORS];

    // slow_start - set to true to slew motors from current speed to maximum
    // Note: this must be set immediately before a step up in throttle
    void                slow_start(bool true_false);

    // final output values sent to the motors.  public (for now) so that they can be access for logging
    int16_t             motor_out[AP_MOTORS_MAX_NUM_MOTORS];

    // structure for holding motor limit flags
    struct AP_Motors_limit {
        uint8_t roll_pitch      : 1; // we have reached roll or pitch limit
        uint8_t yaw             : 1; // we have reached yaw limit
        uint8_t throttle_lower  : 1; // we have reached throttle's lower limit
        uint8_t throttle_upper  : 1; // we have reached throttle's upper limit
    } limit;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo        var_info[];

protected:

    // output functions that should be overloaded by child classes
    virtual void        output_armed() {};
    virtual void        output_disarmed() {};

    // update_max_throttle - updates the limits on _max_throttle if necessary taking into account slow_start_throttle flag
    void                update_max_throttle();

    // flag bitmask
    struct AP_Motors_flags {
        uint8_t armed               : 1;    // 1 if the motors are armed, 0 if disarmed
        uint8_t frame_orientation   : 4;    // PLUS_FRAME 0, X_FRAME 1, V_FRAME 2, H_FRAME 3, NEW_PLUS_FRAME 10, NEW_X_FRAME, NEW_V_FRAME, NEW_H_FRAME
        uint8_t slow_start          : 1;    // 1 if slow start is active
        uint8_t slow_start_low_end  : 1;    // 1 just after arming so we can ramp up the spin_when_armed value
    } _flags;

    // parameters
    AP_CurveInt16_Size4 _throttle_curve;        // curve used to linearize the pwm->thrust
    AP_Int8             _throttle_curve_enabled;        // enable throttle curve
    AP_Int8             _throttle_curve_mid;    // throttle which produces 1/2 the maximum thrust.  expressed as a percentage (i.e. 0 ~ 100 ) of the full throttle range
    AP_Int8             _throttle_curve_max;    // throttle which produces the maximum thrust.  expressed as a percentage (i.e. 0 ~ 100 ) of the full throttle range
    AP_Int16            _spin_when_armed;       // used to control whether the motors always spin when armed.  pwm value above radio_min 

    // internal variables
    RC_Channel*         _rc_roll, *_rc_pitch, *_rc_throttle, *_rc_yaw;  // input in from users
    uint8_t             _motor_to_channel_map[AP_MOTORS_MAX_NUM_MOTORS];        // mapping of motor number (as received from upper APM code) to RC channel output - used to account for differences between APM1 and APM2
    uint16_t            _speed_hz;              // speed in hz to send updates to motors
    int16_t             _min_throttle;          // the minimum throttle to be sent to the motors when they're on (prevents motors stalling while flying)
    int16_t             _max_throttle;          // the maximum throttle to be sent to the motors (sometimes limited by slow start)
    int16_t             _hover_out;             // the estimated hover throttle in pwm (i.e. 1000 ~ 2000).  calculated from the THR_MID parameter
    int16_t             _spin_when_armed_ramped;// equal to _spin_when_armed parameter but slowly ramped up from zero
};
#endif  // __AP_MOTORS_CLASS_H__

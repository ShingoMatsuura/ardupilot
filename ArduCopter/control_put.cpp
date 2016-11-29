#include "Copter.h"

static bool put_with_gps;

static uint32_t put_start_time;
static uint32_t put_reached_desirealt_time;
static uint32_t put_release_time;

static bool put_pause;
static PutStateType put_state;

#define PUT_GRIP_RELEASE_ALT 150

// put_init - initialise land controller
bool Copter::put_init(bool ignore_checks)
{
    // check if we have GPS and decide which PUT we're going to do
    put_with_gps = position_ok();
    if (put_with_gps) {
        // set target to stopping point
        Vector3f stopping_point;
        wp_nav.get_loiter_stopping_point_xy(stopping_point);
        wp_nav.init_loiter_target(stopping_point);
    }

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(wp_nav.get_speed_down(), wp_nav.get_speed_up());
    pos_control.set_accel_z(wp_nav.get_accel_z());

    // initialise position and desired velocity
    if (!pos_control.is_active_z()) {
        pos_control.set_alt_target_to_current_alt();
        pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());
    }
    
    put_start_time = millis();

    put_pause = false;

    // reset flag indicating if pilot has applied roll or pitch inputs during landing
    ap.land_repo_active = false;

    put_state = PutStateType_Descending;

    return true;
}

// put_run - runs the put controller
// should be called at 100hz or more
void Copter::put_run()
{
	switch (put_state) {
	case PutStateType_Descending:
	    if (put_with_gps) {
	        put_gps_run();
	    }else{
	        put_nogps_run();
	    }
		break;

	case PutStateType_ReachedDesireAlt:
	    put_reached_desirealt_time = millis();
	    put_state = PutStateType_ControlServo;
		// FALL THUROUGH

	case PutStateType_ControlServo:
	    put_release_time = millis();
	    // wait 3 seconds before release gripper
	    if (millis() - put_reached_desirealt_time > 1000) {
	        put_control_servo_at_desire_alt();
	        put_state = PutstateType_PutComplete;
	    }
	    // FALL THUROUGH

	case PutstateType_PutComplete:
	    // wait 3 seconds before RTL
	    if (millis() - put_release_time > 1000) {
	        g2.gripper.grab();
	        set_mode(RTL, MODE_REASON_PUT_COMPLETE);
	    }
		break;
	}

}

// put_run - runs the put controller
//      horizontal position controlled with loiter controller
//      should be called at 100hz or more
void Copter::put_gps_run()
{
    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    
    // pause before beginning land descent
    if(put_pause && millis()-put_start_time >= LAND_WITH_DELAY_MS) {
        put_pause = false;
    }
    
    put_run_horizontal_control();
    put_run_vertical_control(put_pause);
}

// put_nogps_run - runs the land controller
//      pilot controls roll and pitch angles
//      should be called at 100hz or more
void Copter::put_nogps_run()
{
    float target_roll = 0.0f, target_pitch = 0.0f;
    float target_yaw_rate = 0;

    // process pilot inputs
    if (!failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            Log_Write_Event(DATA_LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            set_mode(ALT_HOLD, MODE_REASON_THROTTLE_LAND_ESCAPE);
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // get pilot desired lean angles
            get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // pause before beginning land descent
    if(put_pause && millis()-put_start_time >= LAND_WITH_DELAY_MS) {
        put_pause = false;
    }

    put_run_vertical_control(put_pause);
}

/*
  get a height above ground estimate for putting
 */
int32_t Copter::put_get_alt_above_ground(void)
{
    int32_t alt_above_ground;
    if (rangefinder_alt_ok()) {
        alt_above_ground = rangefinder_state.alt_cm_filt.get();
    } else {
        bool navigating = pos_control.is_active_xy();
        if (!navigating || !current_loc.get_alt_cm(Location_Class::ALT_FRAME_ABOVE_TERRAIN, alt_above_ground)) {
            alt_above_ground = current_loc.alt;
        }
    }
    return alt_above_ground;
}

void Copter::put_run_vertical_control(bool pause_descent)
{
    bool navigating = pos_control.is_active_xy();

    // compute desired velocity
    const float precput_acceptable_error = 15.0f;
    const float precput_min_descent_speed = 10.0f;
    int32_t alt_above_ground = put_get_alt_above_ground();

    if (alt_above_ground <= PUT_GRIP_RELEASE_ALT) {
        desired_climb_rate = 0.0f;

        // update altitude target and call position controller
        pos_control.set_alt_target_from_climb_rate_ff(0.0f, G_Dt, true);
        pos_control.update_z_controller();

    	put_state = PutStateType_ReachedDesireAlt;
    	return;
    }

    float cmb_rate = 0;
    if (!pause_descent) {
        float max_put_descent_velocity;
        if (g.land_speed_high > 0) {
            max_put_descent_velocity = -g.land_speed_high;
        } else {
            max_put_descent_velocity = pos_control.get_speed_down();
        }

        // Don't speed up for landing.
        max_put_descent_velocity = MIN(max_put_descent_velocity, -abs(g.land_speed));

        // Compute a vertical velocity demand such that the vehicle approaches put_START_ALT. Without the below constraint, this would cause the vehicle to hover at put_START_ALT.
        cmb_rate = AC_AttitudeControl::sqrt_controller(LAND_START_ALT-alt_above_ground, g.p_alt_hold.kP(), pos_control.get_accel_z());

        // Constrain the demanded vertical velocity so that it is between the configured maximum descent speed and the configured minimum descent speed.
        cmb_rate = constrain_float(cmb_rate, max_put_descent_velocity, -abs(g.land_speed));
    }

    // record desired climb rate for logging
    desired_climb_rate = cmb_rate;

    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate_ff(cmb_rate, G_Dt, true);
    pos_control.update_z_controller();
}

void Copter::put_run_horizontal_control()
{
    int16_t roll_control = 0, pitch_control = 0;
    float target_yaw_rate = 0;
    
    // process pilot inputs
    if (!failsafe.radio) {
        if ((g.throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0 && rc_throttle_control_in_filter.get() > LAND_CANCEL_TRIGGER_THR){
            Log_Write_Event(DATA_LAND_CANCELLED_BY_PILOT);
            // exit land if throttle is high
            if (!set_mode(LOITER, MODE_REASON_THROTTLE_LAND_ESCAPE)) {
                set_mode(ALT_HOLD, MODE_REASON_THROTTLE_LAND_ESCAPE);
            }
        }

        if (g.land_repositioning) {
            // apply SIMPLE mode transform to pilot inputs
            update_simple_mode();

            // process pilot's roll and pitch input
            roll_control = channel_roll->get_control_in();
            pitch_control = channel_pitch->get_control_in();

            // record if pilot has overriden roll or pitch
            if (roll_control != 0 || pitch_control != 0) {
                ap.land_repo_active = true;
            }
        }

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }
    
    // process roll, pitch inputs
    wp_nav.set_pilot_desired_acceleration(roll_control, pitch_control);

    // run loiter controller
    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

    int32_t nav_roll  = wp_nav.get_roll();
    int32_t nav_pitch = wp_nav.get_pitch();

    if (g2.wp_navalt_min > 0) {
        // user has requested an altitude below which navigation
        // attitude is limited. This is used to prevent commanded roll
        // over on landing, which particularly affects helicopters if
        // there is any position estimate drift after touchdown. We
        // limit attitude to 7 degrees below this limit and linearly
        // interpolate for 1m above that
        int alt_above_ground = put_get_alt_above_ground();
        float attitude_limit_cd = linear_interpolate(700, aparm.angle_max, alt_above_ground,
                                                     g2.wp_navalt_min*100U, (g2.wp_navalt_min+1)*100U);
        float total_angle_cd = norm(nav_roll, nav_pitch);
        if (total_angle_cd > attitude_limit_cd) {
            float ratio = attitude_limit_cd / total_angle_cd;
            nav_roll *= ratio;
            nav_pitch *= ratio;

            // tell position controller we are applying an external limit
            pos_control.set_limit_accel_xy();
        }
    }

    
    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(nav_roll, nav_pitch, target_yaw_rate, get_smoothing_gain());
}

// control servo when reached to desire alt.
void Copter::put_control_servo_at_desire_alt()
{
	g2.gripper.release();
}

// put_do_not_use_GPS - forces put-mode to not use the GPS but instead rely on pilot input for roll and pitch
//  called during GPS failsafe to ensure that if we were already in PUT mode that we do not use the GPS
//  has no effect if we are not already in PUT mode
void Copter::put_do_not_use_GPS()
{
    put_with_gps = false;
}

// set_mode_put_with_pause - sets mode to PUT and triggers 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Copter::set_mode_put_with_pause(mode_reason_t reason)
{
    set_mode(PUT, reason);
    put_pause = true;

    // alert pilot to mode change
    AP_Notify::events.failsafe_mode_change = 1;
}

// putting_with_GPS - returns true if vehicle is putting using GPS
bool Copter::putting_with_GPS()
{
    return (control_mode == PUT && put_with_gps);
}

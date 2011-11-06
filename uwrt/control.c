/******************************************************************************
 * control.c
 * Copyright 2011 University of Waterloo Rocketry Team
 *
 * Main control state machine.
 ******************************************************************************
 * This program is distributed under the of the GNU Lesser Public License. 
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *****************************************************************************/

#include <Compiler.h>

#include "control.h"
#include "est_apogee.h"
#include "kalman.h"
#include "igniters.h"
#include "core/error.h"
#include "core/timing.h"
#include "io/spi_press.h"
#include "io/gps.h"

// Maximum time to wait for GPS fix before going to ignition-ready state [ms]
#define GPS_MAX_FIX_TIME 120000

// Time to wait after igniter firing to open NO2 [ms]
#define NO2_OPEN_DELAY 500

// Time to leave igniter on
#define IGNITER_FIRE_TIME 1000

// 10000 feet, in meters
#define TARGET_ALT (10000*0.3048)

// How long to wait after apogee before deploying drogue [ms]
#define DROGUE_DELAY 2000

// At what altitude the main chute should be deployted [m]
#define PARACHUTE_ALT (1000*0.3048)

enum _control_states {
    NO_GPS_FIX,
    WAIT_IGN,
    IGN_DEBOUNCE,
    IGN_FIRING,
    NO2_ON_FIRING,
    THRUSTING,
    MAYBE_SHUTOFF,
    NO2_OFF_FIRING,
    WAIT_APOGEE,
    WAIT_DROGUE,
    DROGUE_FIRING,
    WAIT_PARACHUTE,
    PARACHUTE_FIRING,
    END
};

// The current control state
static enum _control_states _control_state = NO_GPS_FIX;

// The time of the last state change
static long _last_event_time = 0;

// The estimated altitude at launch.
static double _launch_alt = 2000;

static void _set_state(enum _control_states new_state) {
    _control_state = new_state;
    _last_event_time = sys_time();
}

// Prints state periodically, only if USB is connected.
void _maybe_print_state(); 

// One control loop iteration
void control(void * data __attribute__((unused))) {
    /* Actions to take always */
    if (gps_data() && gps_data()->gps_mode) {
        PORTCSET = 1 << 2;  // GPS fix LED
    } else {
        PORTCCLR = 1 << 2;
    }

    /* Kalman update logic */
    int kalman_updated = 0;
    switch (_control_state) {
    case NO_GPS_FIX:
    case WAIT_IGN:
    case IGN_DEBOUNCE:
    case IGN_FIRING:
    case NO2_ON_FIRING:
        /* Not yet in flight */
        if (gps_has_new() && gps_data()->gps_mode) {
            kalman_setup_gps(gps_data()->altitude, gps_data()->tstamp);
            _launch_alt = gps_data()->altitude;
            kalman_updated = 1;
        }
        if (press_has_new()) {
            kalman_setup_press(press_data()->alt_calc, press_data()->tstamp);
            _launch_alt = press_data()->alt_calc;
            kalman_updated = 1;
        }

    case THRUSTING:
    case MAYBE_SHUTOFF:
    case NO2_OFF_FIRING:
        /* Engine operating */
        if (gps_has_new() && gps_data()->gps_mode) {
            update_gps(gps_data()->altitude, gps_data()->tstamp, 1);
            kalman_updated = 1;
        }
        if (press_has_new()) {
            update_press(press_data()->alt_calc, press_data()->tstamp, 1);
            kalman_updated = 1;
        }

    case WAIT_APOGEE:
    case WAIT_DROGUE:
    case DROGUE_FIRING:
    case WAIT_PARACHUTE:
    case PARACHUTE_FIRING:
    case END:
        /* Engine off */
        if (gps_has_new()) {
            update_gps(gps_data()->altitude, gps_data()->tstamp, 0);
            kalman_updated = 1;
        }
        if (press_has_new()) {
            update_press(press_data()->alt_calc, press_data()->tstamp, 0);
            kalman_updated = 1;
        }
    }

    /* State change / control logic */
    switch (_control_state) {
    case NO_GPS_FIX:
        if ((sys_time() - _last_event_time) > GPS_MAX_FIX_TIME) {
            // Give up waiting for fix
            _set_state(WAIT_IGN);
        }
        if (gps_data() && gps_data()->gps_mode) {
            // Have fix!!
            _set_state(WAIT_IGN);
        }
        break;

    case WAIT_IGN:
        PORTCSET = 1 << 3;  // Ignition ready LED
        if (PORTC & (1 << 4)) {
            _set_state(IGN_DEBOUNCE);
        }
        break;

    case IGN_DEBOUNCE:
        if ((sys_time() - _last_event_time) > 100) {
            if (PORTC & (1 << 4)) {
                set_igniter(IGNITER, IGN_FIRE);
                _set_state(IGN_FIRING);
            }
        }
        break;

    case IGN_FIRING:
        if ((sys_time() - _last_event_time) > NO2_OPEN_DELAY ) {
            set_igniter(NO2_OPEN, IGN_FIRE);
            _set_state(NO2_ON_FIRING);
        }
        break;

    case NO2_ON_FIRING:
        if ((sys_time() - _last_event_time) > IGNITER_FIRE_TIME) {
            set_igniter(IGNITER, IGN_OFF);
            set_igniter(NO2_OPEN, IGN_OFF);
            _set_state(THRUSTING);
        }
        if ((get_mu_est())[1] < 0) {
            _set_state(WAIT_DROGUE);
        }
        break;

    case THRUSTING:
        if (kalman_updated) {
            double apogee = est_apogee(get_mu_est(), get_last_t());
            if (apogee > TARGET_ALT ) {
                _set_state(MAYBE_SHUTOFF);
            }
        }
        break;

    case MAYBE_SHUTOFF:
        if (kalman_updated) {
            double apogee = est_apogee(get_mu_est(), get_last_t());
            if (apogee > TARGET_ALT ) {
                set_igniter(NO2_CLOSE, IGN_FIRE);
                _set_state(NO2_OFF_FIRING);
            } else {
                _set_state(THRUSTING);
            }
        }

    case NO2_OFF_FIRING:
        if ((sys_time() - _last_event_time) > IGNITER_FIRE_TIME) {
            set_igniter(NO2_CLOSE, IGN_OFF);
            _set_state(WAIT_APOGEE);
        }
        break;

    case WAIT_APOGEE:
        if ((get_mu_est())[1] < 0) {
            _set_state(WAIT_DROGUE);
        }
        break;

    case WAIT_DROGUE:
        if ((sys_time() - _last_event_time) > DROGUE_DELAY) {
            set_igniter(DROGUE, IGN_FIRE);
            _set_state(DROGUE_FIRING);
        }
        break;

    case DROGUE_FIRING:
        if ((sys_time() - _last_event_time) > IGNITER_FIRE_TIME) {
            set_igniter(DROGUE, IGN_OFF);
            _set_state(WAIT_PARACHUTE);
        }
        break;

    case WAIT_PARACHUTE:
        if ((get_mu_est())[0] < (PARACHUTE_ALT + _launch_alt)) {
            set_igniter(PARACHUTE, IGN_FIRE);
            _set_state(PARACHUTE_FIRING);
        }
        break;

    case PARACHUTE_FIRING:
        if ((sys_time() - _last_event_time) > IGNITER_FIRE_TIME) {
            set_igniter(PARACHUTE, IGN_OFF);
            _set_state(END);
        }
        break;

    case END:
        break;
    };
}

// Set up control loop
void init_control() {
    add_poll(control, 0);

    /* Ign ready LED */
    PORTCCLR = 1 << 3;
    TRISCCLR = 1 << 3;

    /* GPS fix LED */
    PORTCCLR = 1 << 2;
    TRISCCLR = 1 << 2;

    /* Fire command pin */
    PORTCCLR = 1 << 4;
    TRISCSET = 1 << 4;
}

void print_control_state() {
    static long last_print = 0;
    if((sys_time() - last_print) > 250) {
        last_print = sys_time();
        printf("%d: %lf, %lf, %lf, %lf\n", _control_state, 
                gps_data() ? gps_data()->altitude : 0,
                press_data() ? press_data()->alt_calc : 0,
                (get_mu_est())[0],
                (get_mu_est())[1]);
    }
}


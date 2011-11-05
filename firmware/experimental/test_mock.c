/******************************************************************************
 * test_mock.c
 * Copyright 2011 Iain Peet
 *
 * Implements mock sensor functions, allowing us to replay fake data for
 * systems tests.
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

#include "test_mock.h"

#include "error.h"
#include "kalman.h"
#include "gps.h"
#include "spi_press.h"
#include "test_data.h"
#include "timing.h"

/* Poll the mock replay, to advance */
static void _poll_mock(void * data);

void init_mock() {
    add_poll(_poll_mock, 0);
}

/* Mock gps state */
static int _gps_inited = 0;
static int _gps_new_data = 0;
static gps_data_t _gps_data;

void init_gps(int fd) {
    _gps_inited = 1;
}

int gps_has_new() {
    if (!_gps_inited) {
        err_add(ERROR, UWRT_SYSTEST, "GPS not inited");
    }

    return _gps_new_data;
}

const gps_data_t* gps_data() {
    if (!_gps_inited) {
        err_add(ERROR, UWRT_SYSTEST, "GPS not inited");
    }

    _gps_new_data = 0;
    return &_gps_data;
}

int gps_send_cmd(const char* payload, size_t len, int timeout) {
    return 0;
}

/* Mock pressure state */
static int _press_inited = 0;
static int _press_new_data = 0;
static press_msmt_t _press_data;

void init_pressure() {
    _press_inited = 1;
}

int press_has_new() {
    if (!_press_inited) {
        err_add(ERROR, UWRT_SYSTEST, "Press not inited");
    }

    return _press_new_data;
}

const press_msmt_t* press_data() {
    if (!_press_inited) {
        err_add(ERROR, UWRT_SYSTEST, "Press not inited");
    }

    _press_new_data = 0;
    return &_press_data;
}

/* Mock state management */

enum mock_states_e {
    GPS_WAIT_FIX,
    WAIT_IGN,
    FLIGHT,
    END
};

/* Fixed time events */
#define GPS_FIX_TIME 10000

static enum mock_states_e _mock_state = GPS_WAIT_FIX;

static long _ignition_time = 0;

void mock_ignite() {
    _ignition_time = sys_time();
    _mock_state = FLIGHT;
}

static void _update_gps(int inx, long time_ofst) {
    if( inx < 0 ) {
        /* i.e. no fix */
        _gps_data.tstamp = time_ofst;
        _gps_data.latitude = 0;
        _gps_data.longitude = 0;
        _gps_data.gps_mode = 0;
        _gps_data.satellites = 0;
        _gps_data.altitude = 0;
    } else {
        /* Fix, replaying data */
        _gps_data.tstamp = time_ofst + test_data[inx].t;
        _gps_data.latitude = 0;
        _gps_data.longitude = 0;
        _gps_data.gps_mode = 1;
        _gps_data.satellites = 4;
        _gps_data.altitude = test_data[inx].gps_alt;
    }
}

static void _update_press(int inx, long time_ofst) {
    _press_data.tstamp = test_data[inx].t + time_ofst;
    _press_data.temp = 0;
    _press_data.press = 0;
    _press_data.alt_calc = test_data[inx].press_alt;
}

static void _poll_mock(void * data) {
    static int data_inx = 0;
    static long last_update = 0;
    static int sens_sel = 0;
    int time_ofst;

    switch (_mock_state) {
    case GPS_WAIT_FIX:
        if ((sys_time() - last_update) >= 100) {
            last_update = sys_time();
            if (sens_sel) {
                _update_gps(-1, last_update);
                _gps_new_data = 1;
            } else {
                _update_press(0, last_update);
                _press_new_data = 1;
            }
            sens_sel = !sens_sel;
        }
        if (sys_time() < GPS_FIX_TIME) break;
        _mock_state = WAIT_IGN;
        break;

    case WAIT_IGN:
        if ((sys_time() - last_update) >= 100) {
            last_update = sys_time();
            if (sens_sel) {
                _update_gps(0, last_update);
                _gps_new_data = 1;
            } else {
                _update_press(0, last_update);
                _press_new_data = 1;
            }
            sens_sel = !sens_sel;
        }
        break;

    case FLIGHT:
        if (test_data[data_inx+1].t == -1) {
            _mock_state = END;
        }
        time_ofst = sys_time() - _ignition_time;
        while( (test_data[data_inx+1].t >= 0) && 
               (test_data[data_inx+1].t <= time_ofst) )
        {
            data_inx++;
            if (sens_sel) {
                _update_gps(data_inx, _ignition_time);
                _gps_new_data = 1;
            } else {
                _update_press(data_inx, _ignition_time);
                _press_new_data = 1;
            }
            sens_sel = !sens_sel;
        }
        break;

    case END:
        break;
    }

    /* busy-wait a bit, to simulate long-running sensor operations */
    int i;
    for(i=0; i<8000; ++i);
}


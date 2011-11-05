/******************************************************************************
 * gps.h
 * Copyright 2010 Iain Peet
 *
 * Provides gps control logic and output parsing.
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

#ifndef gps_h_
#define gps_h_

#include <stddef.h>

/** Contains data extracted from GPS */
typedef struct _gps_data {
    unsigned int tstamp;  // System time when this data was received
    int utc_h;  // hh:mm:ss.ms UTC time from satellites
    int utc_m;
    int utc_s;
    int utc_ms;
    double latitude;  // + indicates N, - indicates S
    double longitude; // + indicates E, - indicates W
    int gps_mode; // 0 - no fix, 1 - valid fix.  etc...
    int satellites;
    float hdop;
    float altitude;
} gps_data_t;

/** Initializes the gps system to communicate through a particular file
 *  @param fd  The file to use to communicate with the GPS. This file should
 *             be opened with O_NONBLOCK. */
void init_gps(int fd);

/** @return the last message received from the gps */
const char* gps_last_msg();

/** @return 1 if new GPS data has arrived since last check, 0 if not */
int gps_has_new();

/** Get current GPS data.
 *  Note that, if new data has become available, it will be opportunistically
 *  parsed at this time.
 *  @return Latest GPS data.  0 if no GPS data has ever been obtained. */
const gps_data_t* gps_data();

/** Send a binary command to the GPS.  This function takes care of framing logic.
 *  @param payload  The command payload to send
 *  @param len      Length of the payload, in bytes.
 *  @param timeout  How long to wait for an ACK, in ms.  If 0, there is no
 *                  timeout.  If negative, ACK will not be awaited.
 *  @return 0 if transmision is successful.  nonzero on error, eg. write error
 *          or device NACK'd */
int gps_send_cmd(const char* payload, size_t len, int timeout); 

#endif //gps_h_


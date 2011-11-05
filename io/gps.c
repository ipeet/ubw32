/******************************************************************************
 * gps.c
 * Copyright 2010 Iain Peet
 *
 * Implements gps control logic and output parsing.
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
#include <GenericTypeDefs.h>
#include "HardwareProfile.h"
#include <string.h>

#include "gps.h"
#include "core/error.h"
#include "core/timing.h"
#include "fileio/file_io_drv.h" // for CR/LF defines
#include "unistd.h"

static int        _gps_fd = -1;

#define MAX_GPS_LEN  128

// Buffer for reading in data from non-blocking file
static unsigned char _gps_buf[MAX_GPS_LEN];
static int           _gps_buf_len = 0;
static unsigned int  _gps_buf_ts = 0; // Timestamp.  Set when message begins.

// Copy of the last interesting message from GPS
// Note: we use _gps_msg[0]=='\0' to check for 'no data ever' state
static unsigned char _gps_msg[MAX_GPS_LEN] = {'\0'}; 
static int           _gps_new_msg = 0;
static unsigned int  _gps_msg_ts = 0; // System time when this message was received

// Last binary command message received from gps
static unsigned char _gps_cmd[MAX_GPS_LEN];
static int           _gps_cmd_len = 0;
// Whether we're in the process of reading a command
static int           _gps_cmd_reading = 0;
// Whether there is a new, unhandled cmd from GPS.  Cannot receive another
// cmd until this is cleared by handler.
static int           _gps_new_cmd = 0;

// Set when to sys_time() we are expecting to see a GPS reset.  If a reset
// occurs, we can check timing to see if it's normal.  (Otherwise, major error)
static int           _gps_rst_tm = 0;
// Time after _gps_rst_tim where a reset is 'normal' (in ms)
#define GPS_RST_WINDOW  2000  

// Data parsed from latest _gps_msg
static gps_data_t    _gps_data;

/** Checks for and parses new data from the GPS
 *  @param data  provided for polling interface, but not used here */
static void _pump_gps(void* data);
//! Handles binary command messages from the GPS
static void _gps_cmd_putc(char ch);
//! Handles normal NMEA position messages
static void _gps_nmea_putc(char ch);
//! Parse curreng _gps_msg
static void _gps_parse();

//! GPS binary command beginning of frame 
static struct _gps_cmd_bof {
    unsigned char start[2];
    unsigned char length[2];
} __attribute__ ((packed)) _gps_bof = {.start={0xa0,0xa1}};

//! GPS binary command end of frame
static struct _gps_cmd_eof {
    unsigned char checksum;
    unsigned char end[2];
} __attribute__((packed)) _gps_eof = {.end={0x0d,0x0a}};

//! Initialize GPS
void init_gps(int fd) {
    _gps_fd = fd;
    add_poll(_pump_gps,0);

    /* Sanity checks (make sure the structs didn't get any padding)*/
    if( sizeof(struct _gps_cmd_bof) != 4 ) 
        err_add(ERROR,UWRT_ASSERT,"GPS cmd beginning of frame not 4 bytes\n");
    if( sizeof(struct _gps_cmd_eof) != 3 )
        err_add(ERROR,UWRT_ASSERT,"GPS cmd end of frame not 3 bytes\n");

    // Select only GPGGA messages
    char sel_data[] = {0x08,1,0,0,0,0,0,0,0};
    if( gps_send_cmd(sel_data,9,1000) ) {
        // Failed to select message type.
        // Since we're on 9600 baud, we don't want to increase rate
        // when all messages are being sent
        return;
    }

    // Increase send rate
    char rate_data[] = {0x0e,10,0};
    _gps_rst_tm = sys_time();
    // The GPS seems to reset when rate is changed, and doesn't seem to ever send an
    // ACK.  This is found empirically, and contradicts the lovely documentation.
    gps_send_cmd(rate_data,3,-1);
    
}

//! Get the last GPS message received
const char* gps_last_msg() {
    return _gps_msg;
}

//! Check if new gps data is available
int gps_has_new() {
    return _gps_new_msg;
}

//! Get current GPS data
const gps_data_t* gps_data() {
    // Return nul if no data has been received yet
    if(_gps_msg[0]=='\0') return 0;

    // If there's a new message, we need to parse it first
    if(_gps_new_msg) _gps_parse();

    // Mark message as old
    _gps_new_msg = 0; 

    return &_gps_data;
}

//! Send a binary command message
int gps_send_cmd(const char* payload, size_t len, int timeout) {
    /* Write length
     * Docs LIED when they said this was little-endian!!! */
    _gps_bof.length[0] = len >> 8;
    _gps_bof.length[1] = len & 0xff;

    /* Calculate and write checksum */
    unsigned char cs = 0;
    unsigned int i;
    for(i=0; i<len; ++i) cs = cs ^ payload[i];
    _gps_eof.checksum = cs;

    /* Clear any commands already received */
    _gps_new_cmd = 0;

    /* Write the command packet */
    if( write(_gps_fd,&_gps_bof,sizeof(struct _gps_cmd_bof))
        != sizeof(struct _gps_cmd_bof) ) 
    {
        err_add(ERROR,UWRT_GPS_WRITE,0);
        return -1;
    }
    if( write(_gps_fd,payload,len) != (ssize_t)len ) {
        err_add(ERROR,UWRT_GPS_WRITE,0);
        return -1;
    }
    if( write(_gps_fd,&_gps_eof,sizeof(struct _gps_cmd_eof))
        != sizeof(struct _gps_cmd_eof) ) 
    {
        err_add(ERROR,UWRT_GPS_WRITE,0);
        return -1;
    }

    if(timeout<0) return 0;

    /* Wait for the acknowledge from the GPS */
    unsigned int wait_start = sys_time();
    while( (!timeout) || ((sys_time()-wait_start) < (unsigned)timeout) ) {
        // Wait for ack, either indefinately, or until timeout
        if(_gps_new_cmd) {
            // Got some sort of response
            if(_gps_cmd[4]==0x83) {
                // message id is ACK, we're good
                return 0;
            } else {
                static char str[16];
                sprintf(str, "ID: %02x", (unsigned int)_gps_cmd[4] );
                // not an ACK, something's fishy
                err_add(ERROR, UWRT_GPS_NOACK, str);
                return 1;
            }
        }
        delay();
    }

    // Timeout
    err_add(ERROR, UWRT_GPS_NOACK, 0);
    return 1;
}

//! Read messages from the GPS
static void _pump_gps(void* data) {
    /* RX chars until we get a full line 
     * This is a non-blocking implementation, so no fdgets */
    unsigned char curch;
    int status = read(_gps_fd, &curch, 1);
    while( status > 0 ) {
        /* Watch for beginning of command */
        if( (!_gps_cmd_len) && (curch==0xa0) ) {
            // a0 is the first byte of the start-of-cmd sequence
            _gps_cmd[0] = curch;
            _gps_cmd_len=1;
        } else if( (_gps_cmd_len==1) && (curch==0xa1) ) {
            // start-of-cmd sequence follows a0 with a1
            _gps_cmd[1] = curch;
            _gps_cmd_len=2;
            _gps_cmd_reading = 1;
        } else if( (_gps_cmd_len == 1) ) {
            // a0 start of sequence was spurious, since there's no a1
            _gps_nmea_putc(0xa0);
            _gps_nmea_putc(curch);
            _gps_cmd_len = 0;
        } else if( _gps_cmd_reading ) {
            _gps_cmd_putc(curch);
        }
        /* If it's not a command, its a NMEA message */
        else {
            _gps_nmea_putc(curch);
        }

        status = read(_gps_fd, &curch, 1);
    }
    if( status == -1 ) {
        // Broke because of read error
        err_add(ERROR, UWRT_GPS_READ, 0);
    }
}

//! RX Position message
static void _gps_nmea_putc(char ch) {
    if( _gps_buf_len >= (MAX_GPS_LEN-1) ) {
        // Run out of buffer.  Drop buffer.
        _gps_buf_len = 0;
        err_add(ERROR,UWRT_GPS_OF,"NMEA message buffer");
    }

    // First character in message, record current sys time.
    if(_gps_buf_len == 0 ) _gps_buf_ts = sys_time();

    _gps_buf[_gps_buf_len] = ch;
    ++_gps_buf_len;
    _gps_buf[_gps_buf_len] = '\0';

    // Check for end-of-line
    if( (ch!=CR) && (ch!=LF) && (ch!='\0') )
    {
        // Not end of message, nothing more to do
        return;
    }

    /* We've reached an end-of-message char */
    /* Handle the message */
    if( _gps_buf_len == 1 ) {
        // Line is empty (just a terminating char), so ignore it. 
    } else if( _gps_buf[0] == '$') {
        // Have message, is it interesting?
        if( !strncmp("$GPGGA",_gps_buf,6) ) {
            // Interested in gps location data
            strcpy(_gps_msg,_gps_buf);
            _gps_new_msg = 1;
            _gps_msg_ts = _gps_buf_ts;
        } else if ( !strncmp("$SkyTraq",_gps_buf,8) ||
                    !strncmp("$Kernel",_gps_buf,7) ||
                    !strncmp("$ver",_gps_buf,4) )
           {
            // These messages are sent when the GPS resets
            fprintf(stderr, "GPS: %s\n", _gps_buf);
            if( (sys_time() - _gps_rst_tm) > GPS_RST_WINDOW ) {
                err_add(ERROR,UWRT_GPS_RESET,0);
            }
        } else {
            fprintf(stderr, "Unexpected NMEA: %s\n", _gps_buf);
            // Should only get GPGGA, except after reset. 
            err_add(WARNING,UWRT_GPS_UNEXPECTED,0);
        }
    } else if( (sys_time() - _gps_rst_tm) > GPS_RST_WINDOW ) {
        // Totally invalid data, which can happen at gps startup
        fprintf(stderr, "BAD GPS: %s\n", _gps_buf);
        err_add(ERROR, UWRT_GPS_PARSE, "Invalid NMEA message");
    }

    _gps_buf_len = 0;
}

//! RX a GPS binary command
static void _gps_cmd_putc(char ch) {
    if(_gps_cmd_len >= (MAX_GPS_LEN-1)) {
        // Run out of buffer.  Drop buffer
        _gps_cmd_len = 0;
        _gps_cmd_reading = 0;
        err_add(ERROR,UWRT_GPS_OF,"GPS binary command buffer");
        return;
    }

    _gps_cmd[_gps_cmd_len] = ch;
    ++_gps_cmd_len;
    _gps_cmd[_gps_cmd_len] = '\0';

    // Check for end-of-command
    if( ch != 0x0a ) return;  // not at end yet
    if( _gps_cmd[_gps_cmd_len-2] != 0x0d ) return; // 0x0a is part of data

    _gps_cmd_reading = 0;
    _gps_new_cmd = 1;
}

//! Parse a GPS message out into the data struct
static void _gps_parse() {
    #define GPGGA_FIELDS 15
    char *fields[GPGGA_FIELDS];
    char stmp[MAX_GPS_LEN];
    strcpy(stmp,_gps_msg);

    /* Run through the data string, replacing ,'s with nulls and
     * recording pointers. */
    int i;
    int field = 0;
    fields[0] = stmp;
    for(i=0; stmp[i]!='\0'; ++i) {
        if(stmp[i]==',') {
            /* Begin new field */
            field++;
            if(field >= GPGGA_FIELDS) {
                // Found another field, but didn't expect it!
                break;
            }
            stmp[i]='\0';
            if(field>=GPGGA_FIELDS) break;
            fields[field] = stmp + i + 1;
        }
    }

    if(field != (GPGGA_FIELDS-1)) {
        err_add(ERROR,UWRT_GPS_PARSE,"Not enough fields found");
        fprintf(stderr,"Too few GPS fields: %s\n", _gps_msg);
        return;
    }
    if(field >= GPGGA_FIELDS) {
        err_add(ERROR,UWRT_GPS_PARSE,"Too many fields found");
        fprintf(stderr,"Too many GPS fields: %s\n", _gps_msg);
        return;
    }

    char tmp[4] = {0,0,0,0};

    _gps_data.tstamp = _gps_msg_ts;

    /* Parse out UTC */
    strncpy(tmp,fields[1],2);
    _gps_data.utc_h = atoi(tmp);
    strncpy(tmp,fields[1]+2,2);
    _gps_data.utc_m = atoi(tmp);
    strncpy(tmp,fields[1]+4,2);
    _gps_data.utc_s = atoi(tmp);
    strncpy(tmp,fields[1]+7,3);
    _gps_data.utc_ms = atoi(tmp);
    
    /* Parse out latitude */
    tmp[2] = '\0';
    strncpy(tmp,fields[2],2);
    _gps_data.latitude = atoi(tmp);
    double minutes = atof(fields[2]+2);
    _gps_data.latitude += minutes / 60.0;
    if(fields[3][0]=='S') _gps_data.latitude *= -1.0;

    /* Parse out longitude */
    strncpy(tmp,fields[4],3);
    _gps_data.longitude = atoi(tmp);
    minutes = atof(fields[4]+2);
    _gps_data.longitude += minutes / 60.0;
    if(fields[5][0]=='W') _gps_data.longitude *= -1.0;

    /* Remaining fields (easy! :D ) */
    _gps_data.gps_mode = atoi(fields[6]);
    _gps_data.satellites = atoi(fields[7]);
    _gps_data.hdop = atof(fields[8]);
    _gps_data.altitude = atof(fields[9]);
}


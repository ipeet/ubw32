/******************************************************************************
 * error.c
 * Copyright 2010 Iain Peet
 *
 * Implements basic error-reporting capabilities
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
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "HardwareProfile.h"

#include "error.h"
#include "fileio/file_io.h"

//! Maximum number of errors to track
#define MAX_ERRS   20

//! Error messages corresponding to enum uwrt_err stauses.  May be null 
static const char * _err_msgs[] = {
    "Success",                           //UWRT_SUCCESS
    "See errno",                         //UWRT_ERRNO
    "Error",                             //UWRT_ERROR
    "Assertion failed",                  //UWRT_ASSERT
    "Out of memory",                     //UWRT_NOMEM
    "UART signalling error",             //UWRT_UART_COMM
    "UART TX buffer overflow",           //UWRT_UART_TX_OF
    "UART RX buffer overflow",           //UWRT_UART_RX_OF
    "USB TX buffer overflow",            //UWRT_USB_TX_OF
    "USB RX buffer overflow",            //UWRT_USB_RX_OF
    "GPS read error",                    //UWRT_GPS_READ
    "GPS write error",                   //UWRT_GPS_WRITE
    "GPS buffer overflow",               //UWRT_GPS_OF
    "GPS parse failure",                 //UWRT_GPS_PARSE
    "GPS command not acknowledged",      //UWRT_GPS_NOACK
    "Unexpected GPS NMEA message",       //UWRT_GPS_UNEXPECTED
    "Unexpected GPS reset",              //UWRT_GPS_RESET
    "Pressure sensor startup failure",   //UWRT_PRESS_START
    "Failed to mount device",            //UWRT_NOMOUNT
    "Failed system test"                 //UWRT_SYSTEST

};
//! Number of times each error has occurred since last call
static unsigned int _err_counts[NUM_ERRORS];
//! Whether we've zeroed out _err_counts yet
static int _err_initd = 0;

//! (linked) list of errors encountered:
static err_desc_t* _err_list = 0;
//! Tail of _err_list:
static err_desc_t* _err_tail = 0;

/** Number of errors and warnings listed.  If either of these are nonzero,
 *  the red (error) or yellow (warning) LEDs will be lit */
static int _num_errs = 0;
static int _num_warns = 0;

//! Number of out-of-memory errors encountered allocating error descriptors: 
static int _nomem_count = 0;
//! Always-allocated error descriptor for when we run out of memory 
err_desc_t _nomem_desc;
//! Record out-of-memory occurrence.
static void _handle_nomem();

//! Initialize error handling.  Implicitly called with first err_add
static void _init_err() {
    int i;
    for(i=0; i<NUM_ERRORS; ++i) 
        _err_counts[i] = 0;

    _err_initd = 1;
}

//! Add a new error
void err_add(enum err_type type, enum uwrt_err status, const char* details) {
    if(!_err_initd) _init_err();

    /* Make sure we're not exceeding the active error count limit */
    if( _num_errs + _num_warns >= MAX_ERRS ) {
        // Handle this as an out-of-memory
        _handle_nomem();
        return;
    }

    // Out-of-memory errors use a special statically-allocated descriptor 
    if( status == UWRT_NOMEM ) _handle_nomem();

    /* Allocate an error descriptor */
    err_desc_t * desc = 0;
    if( (status!=UWRT_SUCCESS) && (status!=UWRT_ERRNO) && (_err_counts[status]) ) {
        /* If this has a uwrt_err that isn't ERRNO or SUCCESS, and we've seen this
         * error before, we just need to record the additional occurrence, since
         * we don't store replicas */
        if( _err_counts[status]+1 != 0 ) ++_err_counts[status];
        return;
    } else {
        desc = malloc(sizeof(err_desc_t));
        if( !desc ) {
            _handle_nomem();
            return;
        }
    }

    /* Trip appropriate LED, update appropriate count */
    switch(type) {
        case ERROR:
            mLED_3_On();
            ++_num_errs;
            break;
        case WARNING:
        default:  
            mLED_4_On();
            ++_num_warns;
            break;
    }

    /* Fill descriptor */
    desc->errno = errno;
    desc->type = type;
    desc->status = status;
    desc->details = details;
    desc->next = 0;

    // Record occurrence
    if(_err_counts[status]+1 != 0) ++_err_counts[status];

    /* Add descriptor to error list */
    if( !_err_list ) {
        // Special case: empty list
        _err_list = desc;
        _err_tail = desc;
    } else {
        _err_tail->next = desc;
        _err_tail = desc;
    }
}

//! Record an out-of-memory
static void _handle_nomem() {
    if(!_err_counts[UWRT_NOMEM]) {
        // All this only needs to be done the first time 
        mLED_3_On();
        ++_num_errs;  
        /* Fill struct */
        _nomem_desc.errno = 0;
        _nomem_desc.type = ERROR;
        _nomem_desc.status = UWRT_NOMEM;
        _nomem_desc.details = 0;
        /* Add to error list */
        if( !_err_list ) {
            // Special case: empty list
            _err_list = &_nomem_desc;
            _err_tail = _err_list;
        } else {
            _err_tail->next = &_nomem_desc;
            _err_tail = &_nomem_desc;
        }
    }
    if( _err_counts[UWRT_NOMEM]+1 != 0 ) ++_err_counts[UWRT_NOMEM];
}

//! Peek at the head of the error list
err_desc_t* err_peek() {
    return _err_list;
}

//! Print the current error
void err_print(int fd, err_desc_t* err) {
    char pbuf[32];
    fdputs(fd,"Error Description:\n");

    /* Print internal error into */
    if( err->status ) {
        fdputs(fd,"Internal Error: ");
        sprintf(pbuf,"%d ",err->status);
        fdputs(fd,pbuf);
        sprintf(pbuf,"(%dx)\n",_err_counts[err->status]);
        fdputs(fd,pbuf);
        if(_err_msgs[err->status]) {
                   fdputs(fd,_err_msgs[err->status]);
            fdputs(fd,"\n");
        }
        if(err->details) {
            fdputs(fd,err->details);
            fdputs(fd,"\n");
        }
    }

    /* Print errno */
    if( err->errno ) {
        // If errno is recorded:
        fdputs(fd,"Errno: ");
        sprintf(pbuf,"%d\n",err->errno);
        fdputs(fd,pbuf);
        fdputs(fd,strerror(err->errno));
        fdputs(fd,"\n");
    }
}

//! Remove an error from the list
void err_remove() {
    if( !_err_list ) return; // No error to remove

    err_desc_t *removed = _err_list;

    _err_list = _err_list->next;
    if( !_err_list ) {
        // Reached end
        _err_tail = 0;
    }

    _err_counts[removed->status] = 0;

    /* Decrement count, possibly turn off the respective LED */
    switch(removed->type) {
        case ERROR:
            _num_errs--;
            if(_num_errs<=0) mLED_3_Off();
            break;
        case WARNING:
        default:
            _num_warns--;
            if(_num_warns<=0) mLED_4_Off();
            break;
    }

    // Don't free the special nomem descriptor, it's static
    if( removed == &_nomem_desc ) {
        // Clear the counter, since we've removed the error.
        _nomem_count = 0;
        return;
    }

    free(removed);
}

//! Remove all errors
void err_clear() {
    while(err_peek()) err_remove();
    
    /* Make sure counts and LEDs are reset */
    _num_errs = 0;
    _num_warns = 0;
    mLED_3_Off();
    mLED_4_Off();
}


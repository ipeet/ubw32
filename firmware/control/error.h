/******************************************************************************
 * error.h
 * Copyright 2010 Iain Peet
 *
 * Provides basic error-reporting capabilities
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

#ifndef ERROR_H_
#define ERROR_H_

/** Internal error statuses */
enum uwrt_err {
    UWRT_SUCCESS = 0,
    UWRT_ERRNO,  // Error is in errno
    UWRT_ERROR,  // Generic error
    UWRT_ASSERT, // Assertion failed
    UWRT_NOMEM,  // Out of memory
    UWRT_UART_COMM, // UART communication error
    UWRT_UART_TX_OF, //TX buffer overflow
    UWRT_UART_RX_OF, //RX buffer overflow
    UWRT_USB_TX_OF,
    UWRT_USB_RX_OF,
    UWRT_GPS_READ, //Read error in GPS
    UWRT_GPS_WRITE, //Write error in GPS
    UWRT_GPS_OF, //Buffer overflow in GPS
    UWRT_GPS_PARSE, // GPS Parse error
    UWRT_GPS_NOACK, // GPS command was not acked
    UWRT_GPS_UNEXPECTED, // Unexpected NMEA message
    UWRT_GPS_RESET, // Unexpected GPS reset
    UWRT_PRESS_START, // Pressure sensor failed to start up
    UWRT_NOMOUNT, // Failed to mount a file handle
    UWRT_SYSTEST, // Failed a systems test.
    NUM_ERRORS
};

/** Error types.  (Control which led is triggered) */
enum err_type {
    WARNING,
    ERROR
};

/** An error descriptor */
typedef struct _err_desc {
    int              errno;   // Errno when this error was pushed
    enum err_type    type;    // Type of error
    enum uwrt_err    status;  // Our own internal error status
    const char*      details; // A string with any special details.  May be null.
    struct _err_desc *next;   // Next err descriptor in a list.  PRIVATE.
} err_desc_t;

/** Add an error to the error list.  Always records the current value of errno.
 *  Multiple cases of uwrt_err errors will be folded into one, with an instance
 *  count maintained.  (This is not done for errno errors)
 *  @param type     Whether this is a warning or error.
 *  @param status   The internal error status
 *  @param details  Any specific error information.  May be null */
void err_add(enum err_type type, enum uwrt_err status, const char* details); 

/** @return the current head of the error list.  0 if no errors are queued */
err_desc_t *err_peek();

/** Prints an error description. 
 *  @param fd  IO system file descriptor to print to 
 *  @param err The error to print */
void err_print(int fd, err_desc_t* err);

/** Remove the current head of the error list.  The corresponding error
 *  descriptor is dynamically freed, so a pointer obtained from err_peek() will
 *  become invalid */
void err_remove();

/** Clear the entire error list */
void err_clear();

#endif // ERROR_H_


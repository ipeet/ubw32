/******************************************************************************
 * timing.h
 * Copyright 2010 Iain Peet
 *
 * Provides functions for timed delays and events.  Also provides means of
 * servicing polling driven systems during busy delays.
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

#ifndef timing_h_
#define timing_h_

#include <unistd.h>
// timing.h provides sleep, usleep.  Both will service polling systems
//          while waiting.
// NB:  currently usleep can only provide 1000us resolution

/** Initialize the timing system */
void init_timing();

/** Service polling systems while waiting for an event.
 *  This should be called in all busy-wait loops (or any
 *  other long-running / non-deterministic loop) */
void delay();

/** Waits for a certain number of seconds. */
unsigned int sleep(unsigned int seconds);

/** Waits for a certain number of microseconds */
unsigned int usleep(unsigned int us);

/** Registers a poll function to be pumped during delays 
 *  @param pump  The function to poll
 *  @param data  Pointer to pass to pump with each call */
void add_poll( void (*pump)(void *data), void *data );

/** Removes a polling function previously registered with add_poll.
 *  A poll entry matching both the function and data pointers is sought.
 *  @param pump  The function to poll
 *  @param data  Pointer to pass to pump with each call */
void del_poll( void (*pump)(void *data), void *data );

/** Get current system time, in ms since startup */
unsigned int sys_time();

/** Current system time in nanoseconds.  */
unsigned long long sys_time_ns();

/** Add a timer event, which runs after a certain amount of time has passed.
 *  @param cb   The event callback function.  NB: This runs in IRQ context.
 *  @param data Data pointer to pass to cb
 *  @param ms   How long, after now, should the event occur? */
void add_timer( void (*cb)(void *data), void *data, unsigned int ms ); 

/** Print system load information to stdout */
void print_load();

#endif //timing_h_


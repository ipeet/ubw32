/******************************************************************************
 * timing.c
 * Copyright 2010 Iain Peet
 *
 * Implements timing and polling related logic.
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

#include "timing.h"
#include "error.h"

#define SYS_FREQ                (80000000L)
#define TOGGLES_PER_SEC               (1000)
#define CORE_TICK_RATE               (SYS_FREQ/2/TOGGLES_PER_SEC)
#define CORE_TICK_NS           (25)

/* Node in the circular linked list of polling functions. */
struct _poll_n {
    void           (*pump)(void *data);
    void           *data;
    struct _poll_n *next;
};
//! Circular linked list of polling functions
static struct _poll_n *_poll_list = 0;

/* Node in the sorted list of upcoming timer events */
struct _timer_n {
    void            (*cb)(void *data);
    void            *data;
    struct _timer_n *next;
};
//! Sorted linked list of timer events
static struct _timer_n *_timer_wait= 0;

//! Current system time.  Updated in IRQ context
volatile int _sys_time = 0;

//! System load tracking
int _poll_second = 0; // system time, in seconds, of last delay() call
int _poll_count = 0; // number of polls so far this second
long long _idle_ns = 0;  // 'idle' ns this second
int _last_count = 0; // total polls last second.
long long _last_idle_ns = 0; // total idle ns last second

//! Initialize the timing sytsem
void init_timing() {
    // Open up the core timer at our 1ms rate
    OpenCoreTimer(CORE_TICK_RATE);

    // set up the core timer interrupt with a prioirty of 2 and zero sub-priority
    mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_2 | CT_INT_SUB_PRIOR_0));
}

//! Add a function to be polled
void add_poll( void (*pump)(void* data), void* data ) {
    /* Allocate a node for the poll function */
    struct _poll_n * n = malloc(sizeof(struct _poll_n));
    if( !n ) {
        // Fail to allocate node 
        err_add(ERROR,UWRT_NOMEM,"Add polling event");
        return;
    }

    /* Fill node */
    n->pump = pump;
    n->data = data;

    /* Add to circular poll list */
    if( !_poll_list ) {
        // Special case: empty list
        n->next = n;
        _poll_list = n;
    } else {
        n->next = _poll_list->next;
        _poll_list->next = n;
    }
}

//! Remove a polling function from the list
void del_poll( void (*pump)(void* data), void* data) {
    if(!_poll_list) return;  // No nodes to remove from.

    /* Search the circular list for the specified node */
    // List is singly-linked, so we need to keep record of prev node 
    struct _poll_n *prev_n = 0;
    struct _poll_n *cur_n = _poll_list;
    int foundn = 0;
    // We end up starting on the node *after* the head, so the search
    // wraps around and looks at the head last, ending after checking head
    do {
        prev_n = cur_n;
        cur_n = cur_n->next;
        if( (cur_n->pump = pump) && (cur_n->data == data) ) {
            foundn = 1;
            break;
        }
    } while( cur_n != _poll_list);
    if( !foundn ) return;  // Node wasn't found

    /* Have node to remove, unlink */
    if( prev_n == cur_n ) {
        // Special case: single item list
        _poll_list = 0;
        free(cur_n);
    } else { 
        if( cur_n == _poll_list ) {
            // Special case:  removing current head
            _poll_list = cur_n->next;
        }
        prev_n->next = cur_n->next;
        free(cur_n);
    }
}

//! Pump the polling list
void delay() {
    //unsigned long long start_ns = sys_time_ns();
    int cur_second = _sys_time / 1000;
    if(cur_second > _poll_second) {
        _last_count = _poll_count;
        _last_idle_ns = _idle_ns;
        _poll_count = 0;
//        _idle_ns = 0;
        _poll_second = cur_second;
    }
    ++_poll_count;

    if(_poll_list) {
        _poll_list->pump(_poll_list->data);
        _poll_list = _poll_list->next;
    }
//    _idle_ns += (sys_time_ns()-start_ns);
}

//! Delay a certain number of seconds
unsigned int sleep(unsigned int seconds) {
    unsigned int start = sys_time();
    unsigned int ms = seconds*1000;
    while( start + ms > sys_time() ) delay();
    return 0;
}

//! Delay a certain number of microseconds.
unsigned int usleep(unsigned int us) {
   unsigned int start = sys_time();
   unsigned int ms = us / 1000;
   while( start + ms > sys_time() ) delay();
   return 0;
}

//! Get current system time
unsigned int sys_time() {
    return _sys_time;
}

//! Get sys time in ns
unsigned long long sys_time_ns() {
    unsigned long long ns = (unsigned long long)(_sys_time) * 1000000L; // ns of current sys_time
    unsigned long long ticks = CORE_TICK_RATE - ReadCoreTimer(); // timer ticks since last irq
    ns += ticks / CORE_TICK_NS;  // ticks to ns
    return ns;
}

//! Print system load info
void print_load() {
    printf("System Load:\n");
    printf("Average Poll Frequency: %dHz\n",_last_count);
}

//! Keep 1ms resolution system time
void __ISR(_CORE_TIMER_VECTOR, ipl2) CoreTimerHandler(void)
{
    // clear the interrupt flag
    mCTClearIntFlag();

    // update the period
    UpdateCoreTimer(CORE_TICK_RATE);

    ++_sys_time;
}


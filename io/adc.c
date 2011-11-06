/******************************************************************************
 * adc.c
 * Copyright 2010 Iain Peet
 *
 * Provides functions for interaction with the Analog to Digital Converter
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

#include <stdio.h>

#include "adc.h"

#include <Compiler.h>
#include <fcntl.h>
#include "core/timing.h"

static unsigned int _print_period_ms = 0;
static unsigned int _print_mask = 0;
static unsigned int _print_last = 0;

void init_adc() 
{
    // Set A0 ~ A7 as digital inputs
    TRISA = 0xff; 

    /* Configure ADC */
    AD1PCFG = 0xf0;  // enable input 0-3
    // Reference VRails, alternate muxes
    AD1CON2 = 0x0001;
    // Use peripheral clock, conversion clock 1/32 multiplier
    AD1CON3 = 0x000f;
    // turn on ADC, unsigned 16 bit output, convert when SAMP cleared:
    AD1CON1 = 0x8000;
    // Use input 0
    AD1CHS  = 0x0;
}

int adc_sample(int channel) {
    // Check channel is in [0,15]
    if( channel & 0xf0 ) return 0;
    // Select the channel for both muxes ( although only A should be used )
    AD1CHS = (channel << 16) | (channel << 24);
    // Begin sample-and-hold
    AD1CON1SET = 0x02;
    // Wait a bit for settle
    int i;
    for(i=0; i<16; ++i) Nop();
    // End sample-and-hold (triggers conversion)
    AD1CON1CLR = 0x02;
    // Wait for conversion to complete
    while( ! (AD1CON1 & 0x1) ) Nop();
    // Fetch result
    return ADC1BUF0;
}

//! Returns -1 if not all data can be written to stdout
static int _adc_print_reading(unsigned int mask)
{
    /* We want to tight poll on ADC, without waiting for
     * sprintf overhead, so we can get samples as close together as
     * possible.  So we buffer inputs before printing */
    int adc_val[16];
    int i = 0;
    
    /* Shift through the bit mask, reading channels which
     * have set bits */
    for(i=0; i<16; ++i) {
        if( mask & 0x1 ) {
            // Channel i is selected in bitmask
            adc_val[i] = adc_sample(i);
        } else {
            // Mark channel as unselected
            adc_val[i] = -1;
        }
       
        mask >>= 1;
    }

    /* We print into a string, then write using write(), rather
     * than using printf.  This is because this function can get
     * called a lot, and printf hangs the system when the usb
     * write buffer is filled.  write() will just fail to send
     * all chars.  */
    char pr_buf[128];  // more than enough for what we're doing
    char pr_len = 0;   // chars printed so far
    pr_len += sprintf(pr_buf+pr_len, "ADC:%d: ",sys_time());
    if( adc_val[0] >= 0 ) {
        pr_len += sprintf(pr_buf+pr_len, "%04d", adc_val[0]);
    }
    for(i=1;i<16;++i) {
        pr_len += sprintf(pr_buf+pr_len, ",");
        if( adc_val[i] >= 0 ) {
            pr_len += sprintf(pr_buf+pr_len, "%04d", adc_val[i]);
        }
    }
    pr_len += sprintf(pr_buf+pr_len, "\n");
   
    // Write out the string
    return pr_len == write(1, pr_buf, pr_len) ? 0 : -1;
}

static void _adc_print_pump(void* data __attribute__((unused)))
{
    // Is it time yet?
    if( (sys_time() - _print_last) < _print_period_ms ) return;

    // It is!
    _print_last = sys_time();
    if( _adc_print_reading(_print_mask) ) {
        // We overflowed the output buffer.  Increase send period
        // So we don't do it again.
        _print_period_ms *= 2;
    }
}

void adc_print(unsigned int mask, int frequency)
{
    // We always need to print one reading:
    if(mask) _adc_print_reading(mask);

    /* Now, handle print subscription rate */
    if( frequency && mask) { // want subscription
        if( !_print_period_ms ) {
            /* If no subscription exists, we need to hook the poll.
             * if one does exist, we just modify state, and shouldn't
             * add a duplicate poll hook */
            add_poll(_adc_print_pump, 0);
        }
        _print_period_ms = 1000 / frequency;
        if(!_print_period_ms) _print_period_ms = 1;  // (maximum freq 1000Hz)
        _print_mask = mask;
        _print_last = sys_time();
    } else if( _print_period_ms ) {
        /* There is a subscription, but frequency=0 is a request to
         * stop.  Remove the hook and clear state so ^^ works properly */
        del_poll(_adc_print_pump, 0);
        _print_period_ms = 0;
        _print_mask = 0;
        _print_last = 0;
    }
}


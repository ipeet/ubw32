/******************************************************************************
 * spi_pres.c
 * Copyright 2010 Iain Peet
 *
 * Implements functions used to interface with the SCP1000 pressure sensor over
 * the SPI bus
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

#include "spi_press.h"
#include "core/timing.h"
#include "core/error.h"
#include <math.h>

// Outputs are all on port G
#define CLK_BIT   15
#define MOSI_BIT  13
#define CSB_BIT   14
#define OUT_MASK  ((1<<CLK_BIT) | (1<<MOSI_BIT) | (1<<CSB_BIT))
// MISO is also on port G
#define MISO_BIT  12

// Idle state for bus
#define m_idle_bus() do { PORTGSET = (1<<CSB_BIT); \
                     PORTGCLR = (1<<CLK_BIT) | (1<<MOSI_BIT); } while(0)

/* Registers.  LSB is always 0, Bit 1 is 0 for read, 1 for write */
#define STATUS_RD    (0x7<<2)
#define OPSTATUS_RD  (0x4<<2)
#define OPERATION_WR ((0x03<<2) | 0x2)
#define TEMP_RD      (0x21<<2)
#define PRESS_8_RD   (0x1f<<2)
#define PRESS_16_RD  (0x20<<2)

/** Write and read one byte.  CSB is lowered on first bit if not already low,
 *  and is not reset at end of transaction */
static unsigned char _spi_byte( unsigned char data_out );
//! Write/Read a 8 bit register
static unsigned char _spi_reg8( unsigned char addr, unsigned char data_out );
//! Write/Read a 16 bit register
static int _spi_reg16( unsigned char addr, int data_out );

//! Check for new data from pressure sensor
static void _pump_press(void *data); // Used as poll function, *data unused

//! Latest pressure data
static press_msmt_t _press_data;
static int _press_new_data = 0; // Whether data has changed since last check
static int _press_has_data = 0; // Whether data has ever been received.

//! Set up bit-banged SPI and pressure sensor
void init_pressure() {
    // Enable outputs
    TRISGCLR = OUT_MASK;
    // Enable inputs
    TRISGSET = (1<<MISO_BIT);
    // Idle bus: low clock, low data, high CSB
    m_idle_bus();

    /* Check if pressure sensor is started */
    unsigned char status = _spi_reg8(STATUS_RD,0);
    if( status & 0x1 ) {
        // It hasn't, wait half a moment 
        usleep(200000);
    }
    // Check again
    status = _spi_reg8(STATUS_RD,0);
    if( status & 0x1 ) {
        err_add(ERROR,UWRT_PRESS_START,0);
    }
    
    /* Start continuous acquisition */
    _spi_reg8(OPERATION_WR,0); // cancel any running operations
    _spi_reg8(OPERATION_WR,0x09); // start high-speed mode

    add_poll(_pump_press,0);
}

//! Obain latest pressure data
const press_msmt_t *press_data() {
    if( !_press_has_data ) return 0;
    _press_new_data = 0;
    return &_press_data;
}

//! Check if there's new pressure data
int press_has_new() {
    return _press_new_data;
}

//! Get a pressure measurement 
static void _pump_press(void * data) {
    // check status
    unsigned char status = _spi_reg8(STATUS_RD,0);
    if( (status & 0x20) && (status != 0xff) ) { // Is data ready?
        /* Retrieve data */
        _press_data.tstamp = sys_time();
        _press_data.temp = _spi_reg16(TEMP_RD, 0) & 0x3fff;
        _press_data.press = _spi_reg8(PRESS_8_RD, 0) & 0x70; // get MSBs (19 bit value)
        _press_data.press = (_press_data.press) << 16; 
        _press_data.press |= _spi_reg16(PRESS_16_RD, 0);
		double ACcoef1 = 1.1379e-6;
        _press_data.alt_calc = ACcoef1*powf(0.25 * _press_data.press, 2.);
        _press_data.alt_calc -= (0.30271 * (0.25 * _press_data.press));
        _press_data.alt_calc += 18260.; 
        _press_new_data = 1;
        _press_has_data = 1;
    }
}

//! 8 data bit register transaction
static unsigned char _spi_reg8( unsigned char addr, unsigned char data_out ) {
    // Place bus in idle, to ensure clean beginning of transaction
    m_idle_bus();
   
    /* Execute transaction */
    _spi_byte(addr);
    unsigned char incoming = _spi_byte(data_out);

    // Return to idle
    m_idle_bus();

    return incoming;
}

//! 16 bit register transaction
static int _spi_reg16( unsigned char addr, int data_out ) {
    // Place bus in idle, to ensure clean beginning of transaction
    m_idle_bus();

    /* Execute transaction */
    int incoming = 0;
    _spi_byte(addr);
    incoming = (_spi_byte(data_out>>8)) << 8; // High byte first
    incoming |= _spi_byte(data_out); // Then low byte

    // Return to idle
    m_idle_bus();

    return incoming;
}

//! Write/read a byte
static unsigned char _spi_byte( unsigned char data_out ) {
    unsigned char incoming = 0;
    int i;
    // record values of porta pins we're not using
    int portg_state = PORTG;
    // clear out bits we'll be setting
    portg_state &= ~OUT_MASK;

    for(i=0; i<8; ++i) {
        // Clock to idle, place data bit on line, CSB low
        PORTG = portg_state | ((data_out>>7) << MOSI_BIT);
        Nop();
        Nop();
        
        // Clock active, keep data out on line
        PORTG = portg_state | ((data_out>>7) << MOSI_BIT) | (1<<CLK_BIT) ;
        // Rotate data in, then read
        incoming = (incoming<<1) | ((PORTG>>MISO_BIT) & 0x1);

        // Rotate data out
        data_out = data_out << 1; 
    }

    return incoming;
}


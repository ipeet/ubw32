/******************************************************************************
 * spi_sw.c
 * Copyright 2010 Iain Peet
 *
 * Implements a bit-banged SPI bus.
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

#include "spi_sw.h"

#include <Compiler.h>

//! SPI idle: clk low, data low, cs high
static void _idle_bus(spi_sw_ctl_t *ctl);
//! Send / Receive a single byte
static unsigned char _spi_byte( spi_sw_ctl_t* ctl, unsigned char data_out);

//! Initialize a sw bus
void init_spi_sw(spi_sw_ctl_t *ctl) {
    /* Set tristate */
    unsigned int trisbits = *(ctl->tris);
    // Clear tris bits for outputs
    trisbits &= 
        ~( (1<<(ctl->clk_bit)) | (1<<(ctl->mosi_bit)) | (1<<(ctl->cs_bit)) );
    // Set tris bit for input
    trisbits |= (1<<(ctl->miso_bit));
    *(ctl->tris) = trisbits;

    _idle_bus(ctl);
}

//! Perform a SPI bus transaction
void spi_sw_trans(spi_sw_ctl_t *ctl, const char *out, char *in, size_t bytes) {
    _idle_bus(ctl);

    int i;
    for(i=0; i<bytes; ++i) {
        in[i] = _spi_byte(ctl, out[i]);
    }

    _idle_bus(ctl);
}

//! Idle the spi bus
static void _idle_bus(spi_sw_ctl_t *ctl) {
    unsigned int portbits = *(ctl->port);
    portbits |= (1<<(ctl->cs_bit));
    portbits &= ~( (1<<(ctl->mosi_bit)) | (1<<(ctl->miso_bit)) );
    *(ctl->port) = portbits;
}

//! Send/receive a single byte
static unsigned char _spi_byte( spi_sw_ctl_t* ctl, unsigned char data_out ) {
    unsigned char incoming = 0;
    int i;
    // record value of port bits we're not using
    int port_state = *(ctl->port);
    // Clear bits we want to set
    port_state &=
        ~( (1<<(ctl->clk_bit)) | (1<<(ctl->mosi_bit)) | (1<<(ctl->miso_bit)) );

    for(i=0; i<8; ++i) {
        // Clock to idle, put data bit on line, cs goes low
        *(ctl->port) = port_state | 
                       ((data_out>>7)<<(ctl->mosi_bit)) | (1<<(ctl->clk_bit));
        Nop(); Nop();

        // Clock active, keep data on line
        *(ctl->port) = port_state |
                       ((data_out>>7)<<(ctl->mosi_bit)) | (1<<(ctl->clk_bit));
        // Rotate data in, then read
        incoming = (incoming<<1) | ((*(ctl->port) >> ctl->miso_bit) & 0x1);

        // Rotate data out
        data_out = data_out << 1;
    }

    return incoming;
}


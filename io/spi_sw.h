/******************************************************************************
 * spi_sw.h
 * Copyright 2010 Iain Peet
 *
 * Provides a software (bit-banged) SPI bus.
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

#ifndef SPI_SW_H_
#define SPI_SW_H_

#include <GenericTypeDefs.h>

/* Control data for a specific sw spi buffer */
typedef struct _spi_sw_ctl {
    /* The port to use for sw SPI */
    unsigned int *port;
    unsigned int *tris;
    /* SPI bus line bits, in port */
    unsigned int clk_bit;
    unsigned int mosi_bit;
    unsigned int miso_bit;
    unsigned int cs_bit;
} spi_sw_ctl_t;

/** Initializes a software spi bus.
 *  @param bus_ctl  The control data for the bus to initialize. */
void init_spi_sw(spi_sw_ctl_t *bus_ctl);

/** Execute a SPI bus transaction.
 *  @param ctl   Control block for bus to communicate over 
 *  @param out   Bytes to transmit on bus
 *  @param in    Bytes received in transaction
 *  @param bytes Number of bytes to send / receive */
void spi_sw_trans(spi_sw_ctl_t *ctl, const char *out, char *in, size_t bytes);

#endif //SPI_SW_H_


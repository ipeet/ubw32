/******************************************************************************
 * spi_press.h
 * Copyright 2010 Iain Peet
 *
 * Provides functions used to interface with the SCP1000 pressure sensor over
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

#ifndef spi_press_h_
#define spi_press_h_

#include <Compiler.h>
#include <GenericTypeDefs.h>

/* SCP1000 provides both temperature and pressure at once */
typedef struct {
    unsigned int tstamp;  // System time when measurement was made
    unsigned int temp;
    unsigned int press;
    double alt_calc;
} press_msmt_t;

/** Initializes the SPI bus and the pressure sensor */
void init_pressure();

/** Check if new pressure/temperature data is available */
int press_has_new();

/** Obtain latest temperature/pressure data.
 *  @return Pointer to latest data.  0 if no data has ever been received.
 *          Pointed data will change when new data arrives (through polling). */
const press_msmt_t *press_data();

#endif // spi_press_h_


/******************************************************************************
 * uart_api.h
 * Copyright 2010 Iain Peet
 *
 * Provides an interface to the PIC32 uart modules
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

#ifndef uart_api_h_
#define uart_api_h_

#include "file_io.h"

//! The hardware uart modules available to the UBW
typedef enum uart_modules_e {
        // these are used for array indices
	UART_MOD_1 = 0,  
	UART_MOD_2,
        UART_MOD_CNT
} uart_t;

//! uart parity selection
typedef enum parity_e {
	// these are control register bits
	NO_PARITY   = 0x0,
	EVEN_PARITY = 0x2,
	ODD_PARITY  = 0x4
} parity_t;

//! uart stop bit selection
typedef enum stopbits_e {
	// these are control register bits
	ONE_STOPBIT  = 0x0,
	TWO_STOPBITS = 0x1
} stopbits_t;

/** Configure uart module hardware.
 *  @param module   Which module should be configured?
 *  @param name	    The device name, for mounting
 *  @param baud     The deisired baud rate
 *  @param parity 	The desired parity
 *  @param stopbits The desired stop bits.
 *  @return 0 on success, nonzero otherwise */
int init_uart(const char *name, uart_t module, int baud, parity_t parity, stopbits_t stopbits); 

/** Finalize a uart module.
 *  @param module   The module to finalize. 
 *  @return 0 on success.  
 *          -1 if the uart is currently open */
int finalize_uart(uart_t module);

#endif //uart_api_h_

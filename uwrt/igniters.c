/******************************************************************************
 * igniters.c
 * Copyright 2011 Iain Peet
 *
 * Implements igniter controls
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

#include "igniters.h"

// Set up igniters
void init_igniters() {
    /* Set pins to output 0.  Before setting to outputs, to avoid
     * possible pulse while initializing */
    PORTDCLR = 1 << 11; // igniter
    PORTDCLR = 1 << 2;  // NO2 open
    PORTDCLR = 1 << 5;  // NO2 close
    PORTGCLR = 1 << 1;  // drogue
    PORTCCLR = 1 << 1;  // main

    /* Set output modes */
    TRISDCLR = 1 << 11; // igniter
    TRISDCLR = 1 << 2;  // NO2 open
    TRISDCLR = 1 << 5;  // NO2 close
    TRISGCLR = 1 << 1;  // drogue
    TRISCCLR = 1 << 1;  // main

}

// Set iginiter mode
void set_igniter(enum igniters_e igniter, int mode) {
    if (mode) {
        /* IGN_FIRE */
        switch (igniter) {
        case IGNITER:
            PORTDSET = 1 << 11; 
            break;
        case NO2_OPEN:
            PORTDSET = 1 << 2;
            break;
        case NO2_CLOSE:
            PORTDSET = 1 << 5; 
            break;
        case DROGUE:
            PORTGSET = 1 << 1; 
            break;
        case PARACHUTE:
            PORTCSET = 1 << 1; 
            break;
        default: break;
        }
    } else {
        /* IGN_OFF */
        switch (igniter) {
        case IGNITER:
            PORTDCLR = 1 << 11; 
            break;
        case NO2_OPEN:
            PORTDCLR = 1 << 2;
            break;
        case NO2_CLOSE:
            PORTDCLR = 1 << 5; 
            break;
        case DROGUE:
            PORTGCLR = 1 << 1; 
            break;
        case PARACHUTE:
            PORTCCLR = 1 << 1; 
            break;
        default: break;
        }
    }
}


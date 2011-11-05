/******************************************************************************
 * igniters.h
 * Copyright 2011 Iain Peet
 *
 * Provides igniter controls.
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

#ifndef IGNITERS_H_
#define IGNITERS_H_ 

enum igniters_e {
    IGNITER,
    NO2_OPEN,
    NO2_CLOSE,
    DROGUE,
    PARACHUTE
};

#define IGN_FIRE 1
#define IGN_OFF  0

/** Sets ports for igniters */
void init_igniters();

/**
 * Set the mode of a given igniter.
 * @param igniter The igniter to set
 * @param mode IGN_FIRE to ignite.  IGN_OFF to turn off.
 */
void set_igniter(enum igniters_e igniter, int mode);

#endif //IGNITERS_H_ 


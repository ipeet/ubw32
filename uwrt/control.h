/******************************************************************************
 * control.h
 * Copyright 2011 University of Waterloo Rocketry Team
 *
 * Main control state machine.
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

#ifndef UWRT_CONTROL_H_
#define UWRT_CONTROL_H_

/** Initialize the control loop. */
void init_control(); 

/** Control state machien poll function */
void control(void * data);

/** Print control state info */
void print_control_state();

#endif // UWRT_CONTROL_H_


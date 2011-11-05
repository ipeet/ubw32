/******************************************************************************
 * test_mock.h
 * Copyright 2011 Iain Peet
 *
 * Functions required to operate the sensor mock.
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

#ifndef TEST_MOCK_H_
#define TEST_MOCK_H_

/** Sets up the mock */
void init_mock();

/** Signal that the engine has been 'ignited' */
void mock_ignite();

#endif //TEST_MOCK_H_

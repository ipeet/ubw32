/******************************************************************************
 * fake_iodev.h
 * Copyright 2010 Iain Peet
 *
 * Provides a fake i/o device interface, to test the IO system.
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

#ifndef fake_iodev_h_
#define fake_iodev_h_

/** Creates a fake io device and mounts it in /dev */
int init_fake_iodev(const char* name);

/** Unomunts a fake io device */
int finalize_fake_iodev();

#endif // fake_iodev_h_


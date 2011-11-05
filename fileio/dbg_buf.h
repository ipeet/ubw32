/******************************************************************************
 * dbg_buf.h
 * Copyright 2010 Iain Peet
 *
 * Provides a file which stores debug messages in a memory ringbuffer.
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

#ifndef dbg_buf_h_
#define dbg_buf_h_

#include "file_io_drv.h"

//! Specify the size (in characters) of the error ringbuffer
#define ERR_BUF_SZ  512

/** Initialize the error ringbuffer and register it in the filesystem 
 *  @return  0 on success, nonzero otherwise */
int init_dbg();

/** Dump the error buffer to the given file descriptor */
int dump_dbg(int fd);

#endif // dbg_buf_h_


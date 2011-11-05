/******************************************************************************
 * file_io.h
 * Copyright 2010 Iain Peet
 *
 * Provides POSIX standard file control functions.
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

#ifndef file_io_h_
#define file_io_h_

// Provide system call definitions, file open flags
#include <fcntl.h>
// Proivde system call definitions
#include <unistd.h>

// Non-POSIX options.  (Highst POSIX option is 0x8000)
#define O_ECHO  0x10000  // Write back every char read

/** Initializes file io system */
void init_file_io();

/** Get a null-terminated line of input.  Provided because the C32 stdio
 *  fgets doesn't appear to actually work.  Also, because it's useful to be
 *  able to gets a fd rather than a full-blown FILE*.
 *  Doesn't include the newline character. */
ssize_t fdgets(int fd, char* str, size_t max_size);

/** Write a null-terminated string of output.  
 *  Provided for convenience */
ssize_t fdputs(int fd, const char* str);

#endif // file_io_h_


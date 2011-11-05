/******************************************************************************
 * dbg_buf.c
 * Copyright 2010 Iain Peet
 *
 * Implements a file handle which directs debug data to an in-memory ringbuffer.
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

#include "dbg_buf.h"

#include "core/error.h"
#include "file_io.h"

//! The debug ringbuffer
static char _dbg_buf[ERR_BUF_SZ];
// The start index of the ringbuffer:
static int  _dbg_start = 0;
// The current length of the ringbuffer:
static int  _dbg_len = 0;

/** File handler callbacks */
static int _dbg_open(int fd, const char* path, int options);
static int _dbg_close(int fd);
static ssize_t _dbg_write(int fd, const void* buf, size_t size);

// Error buffer file handle
static drv_handle_t _dbg_buf_fh = {
    .name = "dbg",
    .ref_count = 0,
    .data = 0,
    .find_handler = std_find_handler,
    .open_handler = _dbg_open,
    .close_handler = _dbg_close,
    .read_handler = 0,
    .write_handler = _dbg_write,
    .lseek_handler = 0,
    .children = 0,
    .next = 0
};

//! Initialize the debug buffer
int init_dbg() {
    // All we need to do is mount the device
    if( mount_dev( &_dbg_buf_fh ) ) {
        err_add(WARNING, UWRT_NOMOUNT, _dbg_buf_fh.name);
        return -1;
    }

    _dbg_start = 0;
    _dbg_len = 0;
    return 0;
}

//! Open the debug buffer 
static int _dbg_open(int fd, const char* path, int options) {
    file_desc_t *fd_data = get_fd_data(fd);
    fd_data->flags = options;
    fd_data->data = 0;
    fd_data->pos = 0;
    return fd;
}

//! Close the debug buffer
static int _dbg_close(int fd) {
    return fd;
}
 
//! Pushes a single character into the debug buffer
static void _dbg_putc(char ch) {
    _dbg_buf[ (_dbg_start+_dbg_len) % ERR_BUF_SZ ] = ch;
    if( _dbg_len < ERR_BUF_SZ ) {
        _dbg_len++;
    } else {
        _dbg_start = (_dbg_start+1) % ERR_BUF_SZ;
    }
}

//! Write characters to the debug buffer
static ssize_t _dbg_write(int fd, const void* buf, size_t size) {
    if(!ERR_BUF_SZ) return 0;
    size_t i;
    for(i=0; i<size; ++i) {
        _dbg_putc( ((const char*)(buf))[i] );
    }
    return size;
}

//! Dump the contents of the debug buffer into the specified file handle
int dump_dbg(int fd) {
    /* Since this is a ringbuffer, we dump data in two stages.  The first
     * stage dumps everything from the start index to the end of the buffer
     * (This may be all that is required).  The second dumps from the
     * beginning of the buffer to the end of the ring */
    int first_chars = _dbg_len;
    if( (_dbg_start+_dbg_len) > ERR_BUF_SZ ) {
        /* If the length of the buffer from the current start point runs off
         * the end of the array, we need to truncate to the end of the array */
        first_chars = ERR_BUF_SZ - _dbg_start;
    }
    if( ! write(fd, _dbg_buf+_dbg_start ,first_chars) ) {
        // Failed to write!
        return -1;
    }

    if( first_chars == _dbg_len ) {
        // Got all the data in the first stage
        return 0;
    }

    /* Second stage: dump data that got wrapped around to the beginning */
    int second_chars = _dbg_len - first_chars;
    if( ! write(fd, _dbg_buf, second_chars) ) {
        // Failed to write!
        return -1;
    }

    return 0;
}


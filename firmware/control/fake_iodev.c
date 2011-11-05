/******************************************************************************
 * fake_iodev.c
 * Copyright 2010 Iain Peet
 *
 * Implements a fake i/o device, for testing the IO system.
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

#include "file_io.h"
#include "file_io_drv.h"
#include "errno.h"

static int fake_open(void* handle, int options);
static int fake_close(void* handle);
static ssize_t fake_read(void* handle, void* buf, size_t size);
static ssize_t fake_write(void*handle, const void* buf, size_t size);
static off_t fake_lseek(void*handle, off_t offset, int whence);

static file_handle_t _fake_dev = {
    .name = 0,
    .flags = 0,
    .cur_fd = -1,
    .ctl_data = 0,
    .find_handler = std_find_handler,
    .open_handler = fake_open,
    .close_handler = fake_close,
    .read_handler = fake_read,
    .write_handler = fake_write,
    .lseek_handler = fake_lseek,
    .children = 0,
    .next = 0
};

int init_fake_iodev(const char* name) {
    _fake_dev.name = name;
    return mount_dev(&_fake_dev);
}

int finalize_fake_iodev() {
    if( unmount_dev(_fake_dev.name) == &_fake_dev ) return 0;
    return 1;
}

static int fake_open(void* handle, int options) {
    file_handle_t* fh = handle;
    if( fh->cur_fd >= 0 ) {
	// Already open
	return fh->cur_fd;
    }
    fh->flags = options;
    int fd = get_fd( handle, -1 );
    if( fd < 0 ) {
	errno = ENFILE;
	return -1;
    }
    return fd;
}

static int fake_close(void* handle) {
    free_fd(((file_handle_t*)handle)->cur_fd);
    return 0;
}

static ssize_t fake_read(void* handle, void* buf, size_t size) {
    int i;
    for(i=0; i<size; ++i) 
	((char*)buf)[i] = 'x';
    return size;
}

static ssize_t fake_write(void* handle, const void* buf, size_t size) {
    return size;
}

static off_t fake_lseek(void *handle, off_t offset, int whence) {
    return offset; 
}


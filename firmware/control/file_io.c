/******************************************************************************
 * file_io.c
 * Copyright 2010 Iain Peet
 *
 * Implements POSIX standard file control functions.
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
#include <GenericTypeDefs.h>

#include "file_io.h"
#include "file_io_drv.h"
#include "errno.h"
#include "HardwareProfile.h"

#define MAX_OPEN_FILES   8

const char PATH_SEP =    '/';

/* Placed in open file table when an open file becomes invalid (unrecoverable
 * error.)  These are cleared by close() */
#define INVALID_HANDLE   ((drv_handle_t*)-1)

/* File descriptor table (indexed by file descriptor number)
 * If the driver pointer is 0, file descriptor is free.
 * If the driver pointer is INVALID_HANDLE, an open file has been invalidated.
 *     (invalidated files are cleared by close()) */
static file_desc_t _open_files[MAX_OPEN_FILES];

/* Root file handle */
static drv_handle_t _root = {
    .name = "",
    .ref_count = 0,
    .data = 0,
    .find_handler = std_find_handler,
    .open_handler = 0,
    .close_handler = 0,
    .read_handler = 0,
    .write_handler = 0,
    .lseek_handler = 0,
    .children = 0,
    .next = 0
};

/* Devfs file handle */
static drv_handle_t _dev = {
    .name = "dev",
    .ref_count = 0,
    .data = 0,
    .find_handler = std_find_handler,
    .open_handler = 0,
    .close_handler = 0,
    .read_handler = 0,
    .write_handler = 0,
    .lseek_handler = 0,
    .children = 0,
    .next = 0
};

/** Null file.  Allows all operations, but does nothing. */
static int _null_open(int fd, const char* path, int options);
static int _null_close(int fd);
static ssize_t _null_read(int fd, void* buf, size_t size)
    { return 0; }
static ssize_t _null_write(int fd, const void* buf, size_t size)
    { return size; }
static off_t _null_lseek(int fd, off_t off, int whence)
    { return 0; }
static drv_handle_t _null_file = {
    .name = "null",
    .find_handler = std_find_handler,
    .open_handler = _null_open,
    .close_handler = _null_close,
    .read_handler = _null_read,
    .write_handler = _null_write,
    .lseek_handler = _null_lseek,
    .children = 0,
    .next = 0
};

//! Check if a fd is open and useable
static int _check_fd(int fd);

//! File IO system initialization
void init_file_io() {
    // Add devfs to root directory
    _root.children = &_dev;
    // Add null file to dev
    _dev.children = &_null_file;

    /* Set initial _open_files state.
     * Standard streams start pointing at the null file */
    _open_files[0].driver = &_null_file;
    _open_files[1].driver = &_null_file;
    _open_files[2].driver = &_null_file;
    int i;
    for(i=3; i<MAX_OPEN_FILES; ++i) 
        _open_files[i].driver = 0;
}

/* Redirect simple mode stdio to this io system */
int _mon_getc(int canblock) {
    char ch = -1;
    if( read(0,&ch,1) != 1 ) return -1;
    return ch;
}
void _mon_putc(char ch) {
    write(1,&ch,1);
}

//! Check file existence / capabilities
int access(const char * path, int mode) {
    drv_handle_t* drvr = _root.find_handler(&_root, path);
    if( !drvr ) {
	// Does not exist
	errno = ENOENT;
	return -1;
    }
    if( mode & R_OK ) {
        // Can we read?
	if( !(drvr->open_handler) || !(drvr->read_handler) ) {
            // No.
            errno = EACCES;
	    return -1;
	}
    }
    if( mode & W_OK ) {
	// Can we write?
	if( !(drvr->open_handler) || !(drvr->write_handler) ) {
	    // No.
	    errno = EACCES;
	    return -1;
	}
    }
    if( mode & X_OK ) {
	// We can't ever execute
	errno = EACCES;
	return -1;
    }

    return 0;
}

//! Open a file
int open(const char * path, int mode, ...) {
    /* NB: This implementation doesn't use the vargs, they're included as part
     *     of the POSIX standard */ 
    drv_handle_t* drv= _root.find_handler(&_root, path);
    if( !drv ) {
        // File does not exist
	errno = ENOENT;
	return -1;
    }
    if( ! (drv->open_handler) ) {
        // File cannot be opened
	errno = EACCES;
	return -1;
    }

    /* Allocate a file descriptor and open the file */
    int fd = get_fd(drv, -1);
    if( fd != drv->open_handler(fd, path, mode) ) {
        // Failed to open, free fd
        free_fd(fd);
        return -1;
    }
    return fd;
}

//! Close a file
int close(int fd) {
    if( (fd<0) || (fd>=MAX_OPEN_FILES) ) {
	// Out of range
	errno = EBADF;	    
	return -1;
    }

    if( ! _open_files[fd].driver ) {
        // File descriptor is not open
        errno = EBADF;
        return -1;
    }

    if( _open_files[fd].driver == INVALID_HANDLE ) {
	// Handle is invalid, close() clears
	_open_files[fd].driver = 0;
	return 0;
    }

    if( ! (_open_files[fd].driver->close_handler) ) {
        // File doesn't support close()
	errno = EINVAL;
	return -1;
    }

    int close_fd = _open_files[fd].driver->close_handler(fd);
    if( close_fd < 0 ) {
        // Failed to close
        return -1;
    }
    free_fd(fd);
    return 0;
}

//! Read data from a file
ssize_t read(int fd, void* buf, size_t count) {
    if( _check_fd(fd) ) return -1;

    if( ! (_open_files[fd].driver->read_handler) ) {
	// File cannot be read
	errno = EINVAL;
	return -1;
    }
    return _open_files[fd].driver->read_handler(fd, buf, count);
}

//! Read a line from a file
ssize_t fdgets(int fd, char* str, size_t max_size) {
    if( _check_fd(fd) ) return -1;

    /* Read data one char at a time */
    size_t i;
    int ret;
    for(i=0; i < (max_size-1); ++i) {
        ret = read(fd,str+i,1);
	if(ret<=0) break; // read error, or can't read now 
	// check if we read a terminating char
	if( (str[i] == CR) || (str[i] == LF) || (str[i] == '\0') ) 
	    break;
    }

    // 'read' indexes the termination, or the failed char - remove this
    str[i] = '\0';
    // Return -1 if we broke on error status.  Otherwise, return chars read:
    return ret<0 ? -1 : i;
}

//! Write a line to file
ssize_t fdputs(int fd, const char* str) {
    if( _check_fd(fd) ) return -1;

    /* Write data one char at a time */
    size_t i;
    int ret;
    for(i=0; str[i] != '\0'; ++i) {
	ret = write(fd,str+i,1);
	if(ret<=0) break;  // writer error, or can't write now
    }

    // Return -1 if we broke on error status.  Otherwise, return chars written:
    return ret<0 ? -1 : i;
}

//! Write data to a file
ssize_t write(int fd, const void* buf, size_t count) {
    if( _check_fd(fd) ) return -1;

    if( ! (_open_files[fd].driver->write_handler) ) {
	// File cannot be written
	errno = EINVAL;
	return -1;
    }
    
    return _open_files[fd].driver->write_handler(fd, buf, count);
}

//! Seek within a file
off_t lseek(int fd, off_t offset, int whence) {
    if( _check_fd(fd) ) return -1;

    if( ! (_open_files[fd].driver->lseek_handler) ) {
	// File cannot seek
	errno = EINVAL;
	return -1;
    }

    return _open_files[fd].driver->lseek_handler(fd, offset, whence);
}

//! Create a duplicate file descriptor
int dup(int oldfd) {
    return dup2(oldfd,-1);
}

//! Create a duplicate file descriptor
int dup2(int oldfd, int newfd) {
    if( _check_fd(oldfd) ) return -1;

    if( (newfd>=0) && (newfd<MAX_OPEN_FILES) && _open_files[newfd].driver ) {
	// Close whatever's at newfd, if it's open
	if( close(newfd) ) {
	    // If close fails (?)
	    return -1;
	}
    }

    // Allocate the new fd
    int fd = get_fd(_open_files[oldfd].driver, newfd);
    // Record the new reference for the file handle
    ++(_open_files[oldfd].driver->ref_count);
    return fd;
}

//! Mount a device file handle in /dev
int mount_dev(drv_handle_t *dev) {
    drv_handle_t *cur_dev = _dev.children;
    
    /* Special case if there are no device mounted yet */
    if(!cur_dev) {
	_dev.children = dev;
	return 0;
    }

    /* Make sure name isn't used (and find end of chain of children) */
    while( cur_dev->next ) {
	if( ! path_cmp(dev->name, cur_dev->name) ) 
	    return -1;  // Name is in use
	cur_dev = cur_dev->next;
    }
    // At last device in chain, need to check its name too
    if( ! path_cmp(dev->name, cur_dev->name) ) return -1;

    //Linkify!
    cur_dev->next = dev;
    return 0;
}

//! Remove a device mounted in /dev
drv_handle_t* unmount_dev(const char* name) {
    // No devices mounted:
    if( ! _dev.children ) return 0;

    /* Find the fd with a matching name */
    drv_handle_t *prev=0, *cur;
    cur = _dev.children;
    while( (cur) && (path_cmp(name,cur->name)) ) {
	prev = cur;
        cur = cur->next;
    }

    // cur becomes 0 if we reach tail without finding name
    if( !cur ) return 0;

    /* Found device, unlink it */
    if( !prev ) {
	// Special case when unlinking head of chain
	_dev.children = cur->next;
    } else {
	// Unlink within chain
	prev->next = cur->next;
    }

    /* If we're unmounting an open file, need to invalidate it */
    inval_dev(cur);

    return cur;
}

//! Allocate an open file descriptor
int get_fd(drv_handle_t * dev, int fd) {
    if( fd >= 0 ) {
        /* Get a specific fd */
	if( _open_files[fd].driver ) {
	    // desired fd is in use
	    return -1;
	}
	_open_files[fd].driver = dev;
	return fd;
    }
    
    /* Use any free open fd */
    for(fd = 0; fd<MAX_OPEN_FILES; ++fd ) {
        if( !_open_files[fd].driver ) {
            // Found a free fd!
	    _open_files[fd].driver = dev;
	    return fd;
	}
    }

    // Found no free fd
    return -1;
}

//! Free an open file descriptor
void free_fd(int fd) {
    _open_files[fd].driver = 0;
}

//! Get a file descriptor struct
file_desc_t *get_fd_data(int fd) {
    if( _check_fd(fd) ) return 0;
    return _open_files + fd;
}

//! Invalidate all file descriptors using a device
void inval_dev(drv_handle_t *dev) {
    int i;
    for(i=0; i<MAX_OPEN_FILES; ++i) {
	if( dev == _open_files[i].driver ) {
	    _open_files[i].driver = INVALID_HANDLE;
	}
    }
}

//! Checks if a fd is open and valid.  Sets errno and returns -1 on error
static int _check_fd(int fd) {
    if( (fd<0) || (fd>=MAX_OPEN_FILES) ) {
        // Invalid fd
	errno = EBADF;
	return -1;
    }
    if( ! _open_files[fd].driver ) {
	// File isn't open
	errno = EBADF;
	return -1;
    }
    if( _open_files[fd].driver == INVALID_HANDLE ) {
	// File is invalidated
	errno = EINVAL;
	return -1;
    }
    return 0;
}

//! Standard file handle find handler (:P)
void* std_find_handler(void* drv, const char* path) {
    /* Path starts with own name, advance to child's name */
    if( ! (path=path_next(path)) ) {
	// No more path entries, therefore this is the desired handle
	return drv;
    }

    /* Search for matching child */
    drv_handle_t *cur_child = ((drv_handle_t*)drv)->children;
    while( cur_child ) {
        if( ! path_cmp(path, cur_child->name) ) {
	    // Recursively open child
	    return cur_child->find_handler(cur_child, path);
	}
	cur_child = cur_child->next;
    }

    // No matching children
    errno = ENOENT;
    return 0;
}

//! Null file open
static int _null_open(int fd, const char* path, int options) {
    _open_files[fd].flags = options;
    _open_files[fd].pos = 0;
    _open_files[fd].data = 0;
    return fd;
}

//! Null close
static int _null_close(int fd) {
    return fd;
}

//! Compare path elements
int path_cmp(const char* s1, const char* s2) {
    while( 1 ) {
        if(    ((*s1 == '\0') || (*s1 == PATH_SEP))
	    && ((*s2 == '\0') || (*s2 == PATH_SEP)) )
	{ // paths terminate together
            return 0;
	}
        if( *s1 != *s2 ) return 1;
        ++s1;
	++s2;
    }
}

//! Advance to the next path element
const char* path_next(const char* path) {
    // Seek to the next sep
    while( ( *path != '\0') && ( *path != PATH_SEP ) ) ++path;
    // If there is no separator, there is no next
    if( *path == '\0' ) return 0;
    ++path;
    // If there is nothing following the separator, there is no next
    if( *path == '\0' ) return 0;
    // We're at the next element
    return path;
}


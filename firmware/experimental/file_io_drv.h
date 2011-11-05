/******************************************************************************
 * file_io_drv.h
 * Copyright 2010 Iain Peet
 *
 * Provides functions and data structures useful for file and driver
 * implementation, but not generally useful to io clients.
 * These functions are implemented in file_io.c
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

#ifndef file_io_drv_h_
#define file_io_drv_h_

#include <Compiler.h>

// Special characters we may be interested in
#define CR	     0x0D
#define LF           0x0A
#define BS           0x08
#define ESC          0x1b

/*********** System call handler types ***********/

/* Finds the driver handling a particular path, intended to be used recursively.
 * @param drvr  The current driver handle in the recursive search.
 * @param path  The current relative path.  The top name should be the name
 *              of the current driver.  Does not usually start with a '/' 
 *              (Excepting when called on the root directory driver)
 * @return  The driver handling the file that the path indicates. 
 *          0 if no such driver is found */
typedef void* (*find_handler_f)(void* drvr, const char* path);

/* Open a file descriptor. 
 * @param fd      The file descriptor allocated to this file.
 * @param path    The absolute path of the file
 * @param options File open mode.
 * @return  The file descriptor on success, -1 on failure, sets errno. 
 * @note fd will be freed automatically if this function fails.
 * @note this should increment the driver refcount on success*/
typedef int (*open_handler_f)(int fd, const char* path, int options); 

/* Close a file descriptor.  
 * @param fd  The file descritpor to close.
 * @return The file descriptor on success.  -1 on failure, sets errno.
 * @note fd will be freed automatically if this function succeeds. 
 * @note this should decrement the driver refcount on success */
typedef int (*close_handler_f)(int fd);

/* Read data from an open file descriptor 
 * @param fd   The file descriptor to read from
 * @param buf  Destination buffer for read data 
 * @param size Number of bytes to read from the file 
 * @return  The number of bytes read.  If less than size and file is blocking,
 *          end of file has been reached.  -1 on error, sets errno. */
typedef ssize_t (*read_handler_f)(int fd, void* buf, size_t size);

/* Write data to an open file descriptor
 * @param fd   The file descriptor to read to
 * @param buf  Buffer containing data to write
 * @param size Size of buf
 * @return  The number of bytes actually written.  May be less than size if
 *          file is non-blocking.  -1 on error, sets errno. */
typedef ssize_t (*write_handler_f)(int fd, const void* buf, size_t size);

/* Set file position of an open file descriptor.
 * @param fd  The file descriptor to modify
 * @param offset Number of bytes by which to seek
 * @param whence Where to seek from.  One of SEEK_SET, SEEK_END, SEEK_CUR.
 * @param The resulting offset from beginning-of-file.  
 *        -1 on error, sets errno */
typedef off_t (*lseek_handler_f)(int fd, off_t offset, int whence);

/* Driver handle for a particular open file. 
 * The driver handle provides device-level interfacing.  Multiple open
 * files may refer to the same driver for operation. */
typedef struct drv_handle_s {
    // The name of this driver.  Becomes the mountpoint in /dev :
    const char*     name;
    // Number of open file descriptors referencing this driver: 
    int             ref_count;
    // Driver-specific data pointer:
    void*           data;
    /* System call handlers.  
     * find_handler is required. std_find_handler is usually sufficient. */
    find_handler_f  find_handler;       
    /* All of these system call handlers are optional. Null indicates an 
     * invalid operation. */
    open_handler_f  open_handler;
    close_handler_f close_handler;
    read_handler_f  read_handler;
    write_handler_f write_handler;
    lseek_handler_f lseek_handler;
    /* Directory management.  
     * If a file is a directory, children point to a chain of contents.
     * Nondireectories will set children to 0.
     * next points too the next file in a directory */
    struct drv_handle_s *children;
    struct drv_handle_s *next;
} drv_handle_t;

/* File descriptor for a particular open file. */
typedef struct file_desc_s {
    drv_handle_t  *driver;  // Driver handling calls for this file
    void          *data;    // Driver-specific data pointer
    int            flags;   // File mode flags
    size_t         pos;     // Current seek position.  (bytes)
} file_desc_t;

/** Mounts a device handle under /dev
 *  @param dev  The device handle to mount.
 *  @return  0 on success
 *           -1 on failure (name already in use) */
int mount_dev(drv_handle_t *dev);

/** Unmounts a device
 *  @param name  The name of the device to unmount
 *  @return  Pointer to the unmounted device handle.
 *           0 if named device not found */
drv_handle_t* unmount_dev(const char* name);

/** Allocate a file descriptor for the give file handle.
 *  The handle will be written into the open file table.
 *  @param dev  The device handle to register.
 *  @param fd   Specify a particular fd to use.  If this fd is in use,
 *              this call fails.  If -1, the first open fd will be used 
 *  @return  Which file descriptor was assigned.
 *           -1 if the file descriptor cannot be allocated */
int get_fd(drv_handle_t * dev, int fd);

/** Free a file descriptor.
 *  The associated file handle will be removed from the open file table */
void free_fd(int fd);

/** Get the file_desc_t struct referred by a file descriptor.  
 *  @return A pointer to the file_desc_t.  0 if fd is invalid. */
file_desc_t * get_fd_data(int fd);

/** Invalidates all file descriptors using a given device handle.
 *  @param dev  The device handle to invalidate */
void inval_dev(drv_handle_t* dev);

/* Compares to the next PATH_SEP or NUL (terminations are equal for comparison),
 * without lexical order.
 * @return 0 if path elements match, 1 otherwise */
int path_cmp(const char*, const char*);

/* Advances to the next element in a path
 * The next element begins immediately after the next separator.
 * @return  Pointer to the next element (within the same string)
 *          0 if there is no next element */
const char* path_next(const char*);

/* Generic find handler.  This matches one of children to the
 * next element in the path, and calls that child's find handler. 
 * If the end of the path is reached, returns the dev.  If children 
 * is null, fails and sets errno.
 * @param path  The path, starting with the driver's own name*/
void* std_find_handler(void* drv, const char* path);

#endif // file_io_drv_h_


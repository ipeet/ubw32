/******************************************************************************
 * uart_api.c
 * Copyright 2010 Iain Peet
 *
 * Implements an interface to the PIC32 uart modules.
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
#include <errno.h>
#include "HardwareProfile.h"

#include "core/error.h"
#include "core/timing.h"
#include "uart_file.h"
#include "file_io_drv.h"

//! Peripheral bus clock frequency
#define PBCLK            80000000L

//! RX ringbuffers for each uart
#define UART_TX_BUF_SIZE 128
static char _uart_tx_buf[UART_MOD_CNT][UART_TX_BUF_SIZE];
static int  _uart_tx_start[UART_MOD_CNT];
static int  _uart_tx_length[UART_MOD_CNT];

//! TX ringbuffers for each uart
#define UART_RX_BUF_SIZE 128
static char _uart_rx_buf[UART_MOD_CNT][UART_RX_BUF_SIZE];
static int  _uart_rx_start[UART_MOD_CNT];
static int  _uart_rx_length[UART_MOD_CNT];

//! Time when uart hardware was initialized
static unsigned int _uart_init_time[UART_MOD_CNT];
/** Time (ms) after initialization where we expect to see signalling errors
 *  (HW comes online in the middle of transmission), and so ignore them */
#define UART_SETTLE_TOL  500

/** Uart configuration */
static void _init_uart1(int baud, parity_t parity, stopbits_t stopbits);
static void _init_uart2(int baud, parity_t parity, stopbits_t stopbits);

/** Register bit defines **/
// Whether interrupts have been recieved:
#define UART1_TXIRQ_STAT  (0x1<<28)
#define UART1_RXIRQ_STAT  (0x1<<27)
#define UART1_ERRIRQ_STAT (0x1<<26)
#define UART1_ALLIRQ_STAT (0x7<<26)
#define UART2_TXIRQ_STAT  (0x1<<10)
#define UART2_RXIRQ_STAT  (0x1<<9)
#define UART2_ERRIRQ_STAT (0x1<<8)
#define UART2_ALLIRQ_STAT (0x7<<8)
// Interrupt enable / disable:
#define UART1_TXIRQ_EN    (0x1<<28)
#define UART1_RXIRQ_EN    (0x1<<27)
#define UART1_ERRIRQ_EN   (0x1<<26)
#define UART1_ALLIRQ_EN   (0x7<<26)
#define UART2_TXIRQ_EN    (0x1<<10)
#define UART2_RXIRQ_EN    (0x1<<9)
#define UART2_ERRIRQ_EN   (0x1<<8)
#define UART2_ALLIRQ_EN   (0x7<<8)
// Whether there is data in (hardware) RX buffer:
#define UART_RX_AVAIL     (0x1)
// Whether there is space in the (hardware) TX buffer:
#define UART_TX_FULL      (0x1<<9)
// Whether TX is in the process of sending data
#define UART_TX_ACTIVE    (0x1<<8)

//! Allowable uart modes.  Append and binary are ignored.
#define UART_OPEN_MODES   ( O_ACCMODE  \
                          | O_NONBLOCK \
                          | O_SYNC     \
                          | O_ECHO     \
                          | O_APPEND   \
                          | O_BINARY   )

/** Additional control data needed by uart driver */
struct _uart_data {
    uart_t module;
    int configured;
    int baud;
    parity_t parity;
    stopbits_t stopbits;
};

/** UART system call handlers */
static int _uart_open(int fd, const char* path,  int options);
static int _uart_close(int fd);
static ssize_t _uart_read(int fd, void* buf, size_t size);
static ssize_t _uart_write(int fd, const void* buf, size_t size);

/** UART file handles */
static struct _uart_data _uart1_data = {
    .module = UART_MOD_1,
    .configured = 0,
    .baud = 0,
    .parity = 0,
    .stopbits = 0
};
static drv_handle_t _uart1_drv = {
    .name = 0,
    .ref_count = 0,
    .data  = &_uart1_data,
    .find_handler = std_find_handler,
    .open_handler = _uart_open,
    .close_handler = _uart_close,
    .read_handler = _uart_read,
    .write_handler = _uart_write,
    .lseek_handler = 0,
    .children = 0,
    .next = 0
};
static struct _uart_data _uart2_data = { 
    .module = UART_MOD_2,
    .configured = 0,  
    .baud = 0,
    .parity = 0,
    .stopbits = 0
};
static drv_handle_t _uart2_drv = {
    .name = 0,
    .ref_count = 0,
    .data = &_uart2_data,
    .find_handler = std_find_handler,
    .open_handler = _uart_open,
    .close_handler = _uart_close,
    .read_handler = _uart_read,
    .write_handler = _uart_write,
    .lseek_handler = 0,
    .children = 0,
    .next = 0
};

/** Convenience array that allows us to reference a uart file handle by its
 *  uart_t module number. */
static drv_handle_t *_uart_drv[] = {&_uart1_drv,&_uart2_drv};
static struct _uart_data *_uart_data[] = {&_uart1_data,&_uart2_data};

//! Configure one of the uart modules
int init_uart(const char* name, uart_t module, int baud, parity_t parity, stopbits_t stopbits) {
    // Check if the module has already been initialized
    if(_uart_data[module]->configured) return -1;

    /* Mount dev */
    _uart_drv[module]->name = name;
    if( mount_dev( _uart_drv[module] ) ) {
        // Failed to mount, device will be inaccessible, so stop here
        return -1;
    }

    /* File handle init */
    _uart_data[module]->configured = 1;
    _uart_data[module]->baud = baud;
    _uart_data[module]->parity = parity;
    _uart_data[module]->stopbits = stopbits;

    /* Record start time */
    _uart_init_time[module] = sys_time();

    /* Register init - depends on module */
    switch(module) {
        case UART_MOD_1: 
            _init_uart1(baud,parity,stopbits);
            return 0;
        case UART_MOD_2:
            _init_uart2(baud,parity,stopbits);
            return 0;
        default: return -1; // Should not happen!
    }
}
static void _init_uart1(int baud, parity_t parity, stopbits_t stopbits) {
    /* Set up interrupts */
    IEC0CLR = UART1_ALLIRQ_EN; // first, ensure uart1 irqs are disabled
    IPC6CLR = 0x7 << 2; // clear priority
    IPC6SET = 0x4 << 2; // set priority 4
    IPC6CLR = 0x3;  // sub-priority 0

    /* Set up UART */
    U1MODE = 0x0800 | parity | stopbits; // no flow control
    U1STA = 0x1400; // irq on tx space,  RX/TX enable, irq on rx char
    U1BRG = PBCLK / (16*baud) - 1;        

    /* Enable interrupts */
    // make sure any old interrupt status is cleared :
    IFS0CLR = UART1_ALLIRQ_STAT;
    // enable rx, tx and error irqs (since all is configured) :
    IEC0SET = UART1_ALLIRQ_EN;  

    // Turn on the uart
    U1MODESET = 0x8000;
}
static void _init_uart2(int baud, parity_t parity, stopbits_t stopbits) {
    /* Set up interrupts */
    IEC1CLR = UART2_ALLIRQ_EN; // first, ensure uart2 irqs are disabled
    IPC8CLR = 0x7 << 2; // clear priority
    IPC8SET = 0x4 << 2; // set priority 4
    IPC8CLR = 0x3;      // clear sub-priority
    IPC8SET = 0x1;      // set sub-priority 1

    /* Set up the UART */
    U2MODE = 0x0800 | parity | stopbits; // no flow control
    U2STA = 0x1400; // irq on tx space, RX/TX enable, irq on rx char
    U2BRG = PBCLK / (16*baud) - 1;

    /* Enable interrupts */
    // clear any old irq status
    IFS1CLR = UART2_ALLIRQ_STAT;
    // enable rx, tx, and err irqs
    IEC1SET = UART2_ALLIRQ_EN;

    // Turn on the uart
    U2MODESET = 0x8000;
}

//! Shut down one of the uarts
int finalize_uart(uart_t module) {
    // Check if module is actually initialized
    if( !_uart_data[module]->configured ) return 0; // it isn't

    // Check if module is open
    if( _uart_drv[module]->ref_count ) return -1; // it's open, so fail

    // Unmount device
    unmount_dev( _uart_drv[module]->name );

    /* Shut down hardware module - turn it off and disable it's interrupts */
    switch(module) {
        case UART_MOD_1:
            IEC0CLR = UART1_ALLIRQ_EN; // disable uart1 interrupts
            U1MODECLR = 0x8000;  // turn off uart1
            break;
        case UART_MOD_2:
            IEC1CLR = UART2_ALLIRQ_EN; // disable uart2 interrupts
            U1MODECLR = 0x8000; // turn off uart2
            break;
        default: break;
    };

    return 0;
}

static int _uart_putc(uart_t module, char ch);  //(single-char helper)

//! Open a uart
static int _uart_open(int fd, const char* path __attribute__((unused)), int options) {
    file_desc_t *fd_data = get_fd_data(fd);
    drv_handle_t *drv = fd_data->driver;
    struct _uart_data *udata = (struct _uart_data*)(drv->data);

    // Check flags are allowed
    if( options & ~UART_OPEN_MODES) {
        // Invalid access mode
        errno = EINVAL;
        return -1;
    }

    // Check that the uart is initialized
    if( ! udata->configured ) {
        // Module does not actually exist
        // This really shouldn't happen
        errno = ENOENT;
        return -1;
    }

    // Init the file descriptor
    fd_data->flags = options;
    fd_data->data = 0;
    fd_data->pos = 0;

    ++(drv->ref_count);

    return fd;
}

//! Close a uart file
static int _uart_close(int fd) {
    drv_handle_t *drv = get_fd_data(fd)->driver;
    --(drv->ref_count);
    return fd;
}

//! Transmit data on a uart
static ssize_t _uart_write(int fd, const void* buf, size_t size) {
    /* Convenience vars */
    file_desc_t *fd_data = get_fd_data(fd);
    struct _uart_data *udata = (struct _uart_data*)(fd_data->driver->data);
    
    uart_t module = udata->module;
    int options = fd_data->flags;

    unsigned int written;
    for(written=0; written<size; ++written) {
        if( ! _uart_putc(module, ((const char*)buf)[written]) ) 
            continue; /* (success, go on to next char) */ 
        
        /* The tx buffer is full, what do we do now? */
        if( options & O_NONBLOCK ) {
            // Don't wait for space, get out now
             err_add(WARNING,UWRT_UART_TX_OF,fd_data->driver->name);
            break;
        }

        delay();
        // char hasn't been written yet, retry:
        --written;
    }

    /* Wait for all data to be transmitted, if requested */
    while ( options & O_SYNC ) {
        switch(module) {
            case UART_MOD_1:
                if( !_uart_tx_length[module] && !(U1STA & UART_TX_ACTIVE) ) 
                    goto flush_done;
            case UART_MOD_2:
                if( !_uart_tx_length[module] && !(U2STA & UART_TX_ACTIVE) )
                    goto flush_done;
            default:
                goto flush_done;
        }
    } 
    flush_done:
    
    return written;
}

/* Place a single character in the RX ringbuffer 
 * Returns 0 on success, 1 if buffer is full */
static int _uart_putc(uart_t module, char ch) {
    // We can check for fullness and drop out without disabling interrupts
    if( _uart_tx_length[module] == UART_TX_BUF_SIZE ) return 1;
    int retstatus = 0;

    /* Need to disable tx interrupt for this module, to sync on ringbuffer */
    switch(module) {
        case UART_MOD_1:
            IEC0CLR = UART1_TXIRQ_EN;
            break;
        case UART_MOD_2:
            IEC1CLR = UART2_TXIRQ_EN;
            break;
        default: break;
    }

    /* Push the character */
    int used_hw_buf = 0;
    switch(module) {
        case UART_MOD_1:
            if( ! (U1STA & UART_TX_FULL) ) { //is hw buffer full?
                // IF hardware buf is full, skip the ringbuffer
                U1TXREG = (unsigned int)(ch);
                used_hw_buf = 1;
            }
            break;
        case UART_MOD_2:
            if( ! (U2STA & UART_TX_FULL) ) {
                U2TXREG = (unsigned int)(ch);
                used_hw_buf = 1;
            }
            break;
        default: break;
    }
    /* If hw buf was full, add char to ringbuffer */
    if( ! used_hw_buf ) {
        /* If hw buf is full, add char to ringbuffer */
        int tx_inx = (_uart_tx_start[module] + _uart_tx_length[module]) 
                     % UART_TX_BUF_SIZE;
        _uart_tx_buf[module][tx_inx] = ch;
        ++_uart_tx_length[module];
    }

    /* Re-enable tx interrupt */
    switch(module) {
        case UART_MOD_1:
            IEC0SET = UART1_TXIRQ_EN;
            break;
        case UART_MOD_2:
            IEC1SET = UART2_TXIRQ_EN;
            break;
        default: break;
    }
    
    return retstatus;
}

static char _uart_getc(uart_t module);  // (single-char helper)

//! Read data from a uart
static ssize_t _uart_read(int fd, void* buf, size_t size) {
    /* Convenience vars */
    file_desc_t *fd_data = get_fd_data(fd);
    struct _uart_data *udata = (struct _uart_data*)(fd_data->driver->data);

    uart_t module = udata->module;
    int options = fd_data->flags;

    unsigned int read;
    for(read=0; read<size; ++read) {
        if( _uart_rx_length[module] ) {
            ((char*)buf)[read] = _uart_getc(module);
            continue;
        }

        /* The Rx buffer is empty, what do do? */
        if( options & O_NONBLOCK ) {
            // Can't wait for new data, get out now
            break;
        }
        delay();
        // Haven't actually read a char this iteration:
        --read;
    }        
    return read;
}

/* Obtain a single character from the RX ringbuffer.
 * Returns 0 if the buffer is empty */
static char _uart_getc(uart_t module) {
    // We can check for emptiness and drop out without disabling interrupts
    if(!_uart_rx_length[module]) return 0;
    
    /* Need to disable rx interrupt for this module, to synchronize ringbuf */
    switch(module) {
        case UART_MOD_1: 
            IEC0CLR = UART1_RXIRQ_EN;
            break;
        case UART_MOD_2:
            IEC1CLR = UART2_RXIRQ_EN;
        default: break;  // off to the races...
    }

    /* Pop the character */
    char retval = _uart_rx_buf[module][_uart_rx_start[module]];
    --_uart_rx_length[module];
    _uart_rx_start[module] = (_uart_rx_start[module]+1) % UART_RX_BUF_SIZE;
    
    /* Aaand re-enable the interrupt */
    switch(module) {
        case UART_MOD_1:
            IEC0SET = UART1_RXIRQ_EN;
            break;
        case UART_MOD_2:
            IEC1SET = UART2_RXIRQ_EN;
            break;
        default: break;
    }
    
    return retval;
}

/** Uart interrupt service logic */

//! Pop a char to transmit from the ringbuffer.  Return NUL if empty.
static char _irq_tx_char(uart_t module) {
    if( ! _uart_tx_length[module] ) return 0;
    char retval = _uart_tx_buf[module][_uart_tx_start[module]];
    --_uart_tx_length[module];
    _uart_tx_start[module] = (_uart_tx_start[module]+1) % UART_TX_BUF_SIZE;
    return retval;
}
//! Push a received character into the ringbuffer.
static void _irq_rx_char(uart_t module, char ch) {
    // calculate index where char will be written
    int add_inx = (_uart_rx_start[module]+_uart_rx_length[module]) 
                  % UART_RX_BUF_SIZE;
    _uart_rx_buf[module][add_inx] = ch;
    
    if( _uart_rx_length[module] < UART_RX_BUF_SIZE ) {
        // there's space, we just added another character
        ++_uart_rx_length[module];
    } else {
        // buffer is full, we overwrote first char, need to rotate buffer
        err_add(WARNING,UWRT_UART_RX_OF,_uart_drv[module]->name);
        _uart_rx_start[module] = (_uart_rx_start[module]+1)%UART_RX_BUF_SIZE;
    }
}

//! uart 1 interrupt service routine
void __ISR(_UART_1_VECTOR, ipl4)  uart1_handler() {
    // Error status must be dealt with first, since it is cleared by RX reads
    if( IFS0 & UART1_ERRIRQ_STAT ) { // Error
        if( (sys_time() - _uart_init_time[UART_MOD_1]) > UART_SETTLE_TOL ) {
            /* Ignore signalling errors during settling period */
            err_add(ERROR,UWRT_UART_COMM,_uart1_drv.name);
        }
        IFS0CLR = UART1_ERRIRQ_STAT; // ack IRQ
    }
    
    if( IFS0 & UART1_TXIRQ_STAT ) { // TX
        // TX while there's hw buf space, and chars in the sw buf
        while( !(U1STA & UART_TX_FULL) && (_uart_tx_length[UART_MOD_1]) ) {
            U1TXREG = _irq_tx_char(UART_MOD_1);
        }
        IFS0CLR = UART1_TXIRQ_STAT; // ack IRQ
    }
    
    if( IFS0 & UART1_RXIRQ_STAT ) { // RX
        // RX while there are chars in hw buf
        while( U1STA & UART_RX_AVAIL ) {
            _irq_rx_char(UART_MOD_1, U1RXREG & 0xff);
        }
        IFS0CLR = UART1_RXIRQ_STAT; // ack IRQ
    } 
}

//! uart 2 interrupt service routine
void __ISR(_UART_2_VECTOR,ipl4) uart2_handler() {
    // Error status must be handled first, since it is cleared by RX buf reads
    if( IFS1 & UART2_ERRIRQ_STAT ) { // Error flag
        if( (sys_time() - _uart_init_time[UART_MOD_2]) > UART_SETTLE_TOL ) {
            /* Ignore signalling errors during settling period */
            err_add(ERROR,UWRT_UART_COMM,_uart2_drv.name);
        }
        IFS1CLR = UART2_ERRIRQ_STAT;
    }

    if( IFS1 & UART2_TXIRQ_STAT ) { // TX flag
        // TX while there's hw buf space, and chars to tx
        while( !(U2STA & UART_TX_FULL) && (_uart_tx_length[UART_MOD_2]) ) {
            U2TXREG = _irq_tx_char(UART_MOD_2);
        }
        IFS1CLR = UART2_TXIRQ_STAT; // ack IRQ
    }

    if( IFS1 & UART2_RXIRQ_STAT ) { // RX flag
        // RX while there are chars in the hw buf
        while( U2STA & UART_RX_AVAIL ) {
            _irq_rx_char(UART_MOD_2, U2RXREG & 0xff);
        }
        IFS1CLR = UART2_RXIRQ_STAT; // ack IRQ 
    }
}

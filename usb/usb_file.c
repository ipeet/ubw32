/******************************************************************************
 * uart_api.c
 * Copyright Complicated: Based on work by Microchip Inc, Brian Schmalz, and
 * Iain Peet
 *
 * Implements functions for reading from / writing to a virtual usb serial 
 * device. 
 ******************************************************************************/

#include <GenericTypeDefs.h>
#include <Compiler.h>
#include <errno.h>

#include "file_io.h"
#include "file_io_drv.h"
#include "error.h"
#include "timing.h"

#include "usb_config.h"
#include "usb_device.h"
#include "usb.h"
#include "usb_function_cdc.h"
#include "HardwareProfile.h"
#include "usb_file.h"

#define TX_BUF_SIZE  256
#define RX_BUF_SIZE  128 
#define USB_SIZE     64   // (packet size)

static char _usb_rx_buf[USB_SIZE];
static char _usb_tx_buf[USB_SIZE];

// USB TX stream ringbuffer
static unsigned char _tx_buf[TX_BUF_SIZE];
static unsigned int  _tx_start  = 0;  // index of the first buffered char
static unsigned int  _tx_length = 0;  // number of buffered chars
static void _tx_buf_push(char ch);  // append a char to the tx buffer

// USB RX stream ringbuffer
static unsigned char _rx_buf[RX_BUF_SIZE];
static unsigned int  _rx_start  = 0;
static unsigned int  _rx_length = 0;

static void _pump_tx();   // See if there is any data to send to PC, and if so, do it
static void _pump_rx();   // Fetch any new data from USB
void _mon_putc (char c); // Our USB based stream character printer

/* Implemented IO Handlers */
static int _usb_open(int fd, const char* path, int options);
static int _usb_close(int fd);
static ssize_t _usb_read(int fd, void* buf, size_t size);
static ssize_t _usb_write(int fd, const void* buf, size_t size);

struct _usb_data {
    int flags;
};
static struct _usb_data _usb_drv_data = {.flags=0};

/* USB File Handle */
static drv_handle_t _usb_drv = {
    .name = 0,
    .ref_count = 0,
    .data = &_usb_drv_data,
    .find_handler = std_find_handler,
    .open_handler = _usb_open,
    .close_handler = _usb_close,
    .read_handler = _usb_read,
    .write_handler = _usb_write,
    .lseek_handler = 0,
    .children = 0,
    .next = 0
};
// Whether usb file handle is currently mounted
static int _usb_mounted = 0;

// Supported open modes ( append and binary mode are allowed, but ignored )
#define USB_OPEN_MODES  ( O_ACCMODE  \
                        | O_NONBLOCK \
                        | O_SYNC     \
                        | O_APPEND   \
                        | O_BINARY   )

//! Open the USB
static int _usb_open(int fd, const char* path, int options) {
    if( options & ~USB_OPEN_MODES ) {
        // Requested invalid mode
        errno = EINVAL;
        return -1;
    }

    file_desc_t *fd_data = get_fd_data(fd);
    fd_data->flags = options;
    ++(_usb_drv.ref_count);

    return fd;
}

//! Close the USB
static int _usb_close(int fd) {
    --(_usb_drv.ref_count);
    return 0;
}

//! Copy data from receive buffer
static ssize_t _usb_read(int fd, void* buf, size_t size) {
    /* Copy data into the buffer */
    unsigned int read;
    file_desc_t *fd_data = get_fd_data(fd);
    for( read = 0; read < size; ++read ) {
        if( _rx_length ) {
            ((char*)buf)[read] = _rx_buf[_rx_start];
            _rx_start = (_rx_start+1)%RX_BUF_SIZE;
            --_rx_length;
        } else if( (USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1) ) {
            // USB isn't useable, so we have to quit
            break;
        } else if( fd_data->flags & O_NONBLOCK ) {
            // If we're not blocking, end read
            break;
        } else {
            // No data available, so wait for more
            --read;
            while( !_rx_length ) {
            	delay();
            }
        }
    }
    return read;
}

//! Copy data to send buffer.
static ssize_t _usb_write(int fd, const void* buf, size_t size) {
    /* Copy the data into the buffer */
    unsigned int written;
    file_desc_t *fd_data = get_fd_data(fd);
    for( written = 0; written < size; ++written ) {
        if( _tx_length < TX_BUF_SIZE ) {
            // If there's space in the buffer, add the character
            _tx_buf_push( ((const char*)buf)[written] );
        } else if ( (USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1) ) {
            // If buffer is full and USB isn't serviceable, bail out
            break;
        } else if ( fd_data->flags & O_NONBLOCK ) {
            // We've run out of space and can't wait, bail
            err_add(WARNING,UWRT_USB_TX_OF,0);
            break;
        } else {
            // Otherwise, wait for buffer space to free up
            while( _tx_length >= TX_BUF_SIZE ) {
                delay();
            }
            --written; // (didn't actually write)
        }
    }

    /* If synchronous, wait for data to be flushed */
    if( fd_data->flags & O_SYNC ) {
        while( _tx_length ) {
            delay();
        }
    }

    return written;
}

//! Poll the USB stack
static void _pump_usb(void* data)
{   
    USBDeviceTasks();

    /* Further action depends on status.  We need to move on to service
     * the CDC driver and our own file upkeep if USB is configured, but sit on
     * our hands otherwise.  We also need to maintain the USB status blinky,
     * which is: off when detached, blinking when not configured, and
     * on when configured */

    // Check if USB is ready for use
    if((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl==1)) {
        /* If USB is not ready, make sure the device handle isn't mounted */
        if(_usb_mounted) {
            unmount_dev(_usb_drv.name);
            _usb_mounted = 0;
        }
    }

    /* Service the blinky.  This is the last thing to be done if usb is not
     * in configured state */
    static unsigned int last_toggle = 0;
    switch(USBDeviceState) {
        case CONFIGURED_STATE: //led on
            mLED_1_On();
            break;  // need to finish poll...

        case DETACHED_STATE: //led off
            mLED_1_Off();
            return;  // nothing more to do

        default:  // somewhere between the above, blink
            if( (sys_time()-last_toggle) > 250 ) {
                mLED_1_Toggle();
                last_toggle = sys_time();
            }
            return;  // nothing more to do

    }

    /* Service code which REQUIRES usb in configured state */
    CDCTxService();

    if( !_usb_mounted ) {
        /* If USB is ready, make sure the device handle is mounted */
        if( ! mount_dev(&_usb_drv) ) {
            _usb_mounted = 1;
        }
    }

    _pump_rx();
    _pump_tx();
}

//! Transmits data from the buffer whenever possible
static void _pump_tx()
{
    // Only send if there's something there to send
    if( !_tx_length ) return;

    // Check that USB is clear to sned
    if( !mUSBUSARTIsTxTrfReady() ) return;

    // Copy up to a USB packet worth of data to the buffer
    unsigned char i;
    unsigned int txBytes = _tx_length < USB_SIZE ? _tx_length : USB_SIZE;
    for( i=0; i<txBytes; ++i ) {
        _usb_tx_buf[i] = _tx_buf[ (_tx_start+i) % TX_BUF_SIZE ];
    }
    _tx_start = (_tx_start+txBytes) % TX_BUF_SIZE;
    _tx_length = _tx_length - txBytes;
    putUSBUSART( _usb_tx_buf, txBytes );
}

//! Receive any data available from USB
static void _pump_rx() {
    // Pull in some new data if there is new data to pull in
    int numBytesRead = getsUSBUSART(_usb_rx_buf,64);
        
    if( !numBytesRead ) return;

    /* Copy data into the buffer */
    int cur_byte;
    for( cur_byte = 0; cur_byte < numBytesRead; ++cur_byte ) {
        // copy the current byte
        _rx_buf[ (_rx_start+_rx_length) % RX_BUF_SIZE ] = _usb_rx_buf[cur_byte];
        // echo if desired
        if(_usb_drv_data.flags & O_ECHO) {
            _tx_buf_push( _usb_rx_buf[cur_byte] );
        }

        /* Advance the buffer */
        if( _rx_length < RX_BUF_SIZE ) {
            ++_rx_length;
        } else {
            // Buffer is overrun, roll around to keep newest data
            err_add(WARNING,UWRT_USB_RX_OF,0);
            _rx_start = (_rx_start+1) % RX_BUF_SIZE;
        }
    }

}

//! Append a char to the tx buffer
static void _tx_buf_push(char ch) {
    _tx_buf[ (_tx_start+_tx_length) % TX_BUF_SIZE ] = ch;
    if( _tx_length < TX_BUF_SIZE ) 
        ++_tx_length;
    else
        _tx_start = (_tx_start+1) % TX_BUF_SIZE;
}

void init_usb(const char* name, int flags) {
        _usb_drv.name = name;
        _usb_drv_data.flags = flags;

        //	The USB specifications require that USB peripheral devices must never source
        //	current onto the Vbus pin.  Additionally, USB peripherals should not source
        //	current on D+ or D- when the host/hub is not actively powering the Vbus line.
        //	When designing a self powered (as opposed to bus powered) USB peripheral
        //	device, the firmware should make sure not to turn on the USB module and D+
        //	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
        //	firmware needs some means to detect when Vbus is being powered by the host.
        //	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
        // 	can be used to detect when Vbus is high (host actively powering), or low
        //	(host is shut down or otherwise not supplying power).  The USB firmware
        // 	can then periodically poll this I/O pin to know when it is okay to turn on
        //	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
        //	peripheral device, it is not possible to source current on D+ or D- when the
        //	host is not actively providing power on Vbus. Therefore, implementing this
        //	bus sense feature is optional.  This firmware can be made to use this bus
        //	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
        //	HardwareProfile.h file.    
        #if defined(USE_USB_BUS_SENSE_IO)
         	tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
        #endif
            
            //	If the host PC sends a GetStatus (device) request, the firmware must respond
        //	and let the host know if the USB peripheral device is currently bus powered
        //	or self powered.  See chapter 9 in the official USB specifications for details
        //	regarding this request.  If the peripheral device is capable of being both
        //	self and bus powered, it should not return a hard coded value for this request.
        //	Instead, firmware should check if it is currently self or bus powered, and
        //	respond accordingly.  If the hardware has been configured like demonstrated
        //	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
        //	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
        //	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
        //	has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
        //	to it in HardwareProfile.h.
        #if defined(USE_SELF_POWER_SENSE_IO)
        	tris_self_power = INPUT_PIN;	// See HardwareProfile.h
        #endif
        
        USBDeviceInit();

        add_poll(_pump_usb,0);
}

// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
        //Example power saving code.  Insert appropriate code here for the desired
        //application behavior.  If the microcontroller will be put to sleep, a
        //process similar to that shown below may be used:
        
        //ConfigureIOPinsForLowPower();
        //SaveStateOfAllInterruptEnableBits();
        //DisableAllInterruptEnableBits();
        //EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
        //Sleep();
        //RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
        //RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

        //IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is 
        //cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause 
        //things to not work as intended.	
        

}


/******************************************************************************
 * Function:        void _USB1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the USB interrupt bit is set
 *        				In this example the interrupt is only used when the device
 *        				goes to sleep when it receives a USB suspend command
 *
 * Note:            None
 *****************************************************************************/
#if 0
void __attribute__ ((interrupt)) _USB1Interrupt(void)
{
    #if !defined(self_powered)
        if(U1OTGIRbits.ACTVIF)
        {
            IEC5bits.USB1IE = 0;
            U1OTGIEbits.ACTVIE = 0;
            IFS5bits.USB1IF = 0;
        
            //USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
            USBClearInterruptFlag(USBIdleIFReg,USBIdleIFBitNum);
            //USBSuspendControl = 0;
        }
    #endif
}
#endif

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *        				suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *        				mode, the host may wake the device back up by sending non-
 *        				idle state signalling.
 *        				
 *        				This call back is invoked when a wakeup from USB suspend 
 *        				is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
        // If clock switching or other power savings measures were taken when
        // executing the USBCBSuspend() function, now would be a good time to
        // switch back to normal full power run mode conditions.  The host allows
        // a few milliseconds of wakeup time, after which the device must be 
        // fully back to normal, and capable of receiving and processing USB
        // packets.  In order to do this, the USB module must receive proper
        // clocking (IE: 48MHz clock must be available to SIE for full speed USB
        // operation).
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

        // Typically, user firmware does not need to do anything special
        // if a USB error occurs.  For example, if the host sends an OUT
        // packet to your device, but the packet gets corrupted (ex:
        // because of a bad connection, or the user unplugs the
        // USB cable during the transmission) this will typically set
        // one or more USB error interrupt flags.  Nothing specific
        // needs to be done however, since the SIE will automatically
        // send a "NAK" packet to the host.  In response to this, the
        // host will normally retry to send the packet again, and no
        // data loss occurs.  The system will typically recover
        // automatically, without the need for application firmware
        // intervention.
        
        // Nevertheless, this callback function is provided, such as
        // for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 *         				firmware must process the request and respond
 *        				appropriately to fulfill the request.  Some of
 *        				the SETUP packets will be for standard
 *        				USB "chapter 9" (as in, fulfilling chapter 9 of
 *        				the official USB specifications) requests, while
 *        				others may be specific to the USB device class
 *        				that is being implemented.  For example, a HID
 *        				class device needs to be able to respond to
 *        				"GET REPORT" type of requests.  This
 *        				is not a standard USB chapter 9 request, and 
 *        				therefore not handled by usb_device.c.  Instead
 *        				this request should be handled by class specific 
 *        				firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
    USBCheckCDCRequest();
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *        				called when a SETUP, bRequest: SET_DESCRIPTOR request
 *        				arrives.  Typically SET_DESCRIPTOR requests are
 *        				not used in most applications, and it is
 *        				optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 *         				SET_CONFIGURATION (wValue not = 0) request.  This 
 *        				callback function should initialize the endpoints 
 *        				for the device's usage according to the current 
 *        				configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
    CDCInitEP();
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 *         				peripheral devices to wake up a host PC (such
 *        				as if it is in a low power suspend to RAM state).
 *        				This can be a very useful feature in some
 *        				USB applications, such as an Infrared remote
 *        				control	receiver.  If a user presses the "power"
 *        				button on a remote control, it is nice that the
 *        				IR receiver can detect this signalling, and then
 *        				send a USB "command" to the PC to wake up.
 *        				
 *        				The USBCBSendResume() "callback" function is used
 *        				to send this special USB signalling which wakes 
 *        				up the PC.  This function may be called by
 *        				application firmware to wake up the PC.  This
 *        				function should only be called when:
 *        				
 *        				1.  The USB driver used on the host PC supports
 *        					the remote wakeup capability.
 *        				2.  The USB configuration descriptor indicates
 *        					the device is remote wakeup capable in the
 *        					bmAttributes field.
 *        				3.  The USB host PC is currently sleeping,
 *        					and has previously sent your device a SET 
 *        					FEATURE setup packet which "armed" the
 *        					remote wakeup capability.   
 *
 *        				This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            Interrupt vs. Polling
 *                  -Primary clock
 *                  -Secondary clock ***** MAKE NOTES ABOUT THIS *******
 *                   > Can switch to primary first by calling USBCBWakeFromSuspend()
 
 *                  The modifiable section in this routine should be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of 1-13 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at lest 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;
    
    USBResumeControl = 1;                // Start RESUME signaling
    
    delay_count = 1800U;                // Set RESUME line for 1-13 ms
    do
    {
        delay_count--;
    }while(delay_count);
    USBResumeControl = 0;
}


/*******************************************************************
 * Function:        void USBCBEP0DataReceived(void)
 *
 * PreCondition:    ENABLE_EP0_DATA_RECEIVED_CALLBACK must be
 *                  defined already (in usb_config.h)
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called whenever a EP0 data
 *                  packet is received.  This gives the user (and
 *                  thus the various class examples a way to get
 *                  data that is received via the control endpoint.
 *                  This function needs to be used in conjunction
 *                  with the USBCBCheckOtherReq() function since 
 *                  the USBCBCheckOtherReq() function is the apps
 *                  method for getting the initial control transfer
 *                  before the data arrives.
 *
 * Note:            None
 *******************************************************************/
#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)
void USBCBEP0DataReceived(void)
{
}
#endif

/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER:
            Nop();
            break;
        default:
            break;
    }      
    return TRUE; 
}


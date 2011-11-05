/********************************************************************
 FileName:     main.c
 Dependencies: See INCLUDES section
 Processor:                PIC18 or PIC24 USB Microcontrollers
 Hardware:                The code is natively intended to be used on the following
                                 hardware platforms: PICDEM� FS USB Demo Board, 
                                 PIC18F87J50 FS USB Plug-In Module, or
                                 Explorer 16 + PIC24 USB PIM.  The firmware may be
                                 modified for use on other USB platforms by editing the
                                 HardwareProfile.h file.
 Complier:          Microchip C18 (for PIC18) or C30 (for PIC24)
 Company:                Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the �Company�) for its PIC� Microcontroller is intended and
 supplied to you, the Company�s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
********************************************************************/

#include <GenericTypeDefs.h> 
#include <Compiler.h>
#include <string.h>
#include <ctype.h>

#include "HardwareProfile.h"
#include "adc.h"
#include "control.h"
#include "dbg_buf.h"
#include "error.h"
#include "file_io.h"
#include "fake_iodev.h"
#include "gps.h"
#include "igniters.h"
#include "spi_press.h"
#include "timing.h"
#include "uart_file.h"
#include "usb_config.h"
#include "usb_device.h"
#include "usb.h"
#include "usb_function_cdc.h"
#include "usb_file.h"

unsigned int __attribute__((section(".boot_software_key_sec,\"aw\",@nobits#"))) SoftwareKey;

static void init_system();

static void handle_cmd(char* cmd);

// Print help info
static void help();
// Print errors
static void print_errors();
// Reset into bootloader
static void enter_bootloader();
// 'gps' command
static void gps_cmd();
// Get pressure
static void pressure();
// Print sensor data
static void sense();
// Print control state
static void control_mon();
// 'adc' command
static void adc_cmd(char *args);
// pwm test command
static void pwm_cmd();

char temp_err[32];

/* Blink an LED, indicating that polling is continuing */
void heartbeat(void* data) 
{
    // blink much more rapidly immediately after reset, to indicate reset.
    int period = sys_time() < 2000 ? 75 : 250;

    if( (sys_time()/period) & 0x1 ) {
        mLED_2_On();
    } else {
        mLED_2_Off();
    }
}

int main()
{   
    init_timing();
    add_poll(heartbeat, 0);
    init_system();
    init_adc();
    init_file_io();
    init_dbg();
    init_igniters();
    init_pressure();

    /* Set up stderr (directed to debug ringbuffer) */ 
    int dbg_fd = open("/dev/dbg",O_RDWR);
    if( dbg_fd != -1 ) {
        // Opened successfully, set as stderr
        dup2(dbg_fd,2);
        if(dbg_fd != 2) close(dbg_fd);
        dbg_fd = 2;
    } else {
        err_add(ERROR, UWRT_ERROR, "Failed to open debug ringbuffer.");
    }
    /* This little gem forces the C32 'full mode' stdio to be linked,
     * (it gets linked only if either fopen or freopen are required)
     * full mode will use write() rather than _mon_putc() for output, which
     * in turn causes fprintf to stderr to write() to fd 2, which means that
     * output goes into the debug ringbuffer, where we want it.  (Ick.) */
    FILE* kludge = fopen("/dev/null","r");
    if(kludge) fclose(kludge);
    fprintf(stderr,"System alive!\n");


    // If we just powered up, we will get here before the GPS module has powered
    // up properly, and the initialization will not work properly (note that we
    // also wait to init the uart so we don't buffer crap)
    sleep(1);
    init_uart("uart1",UART_MOD_1,9600,NO_PARITY,ONE_STOPBIT);
    int gps_fd = open("/dev/uart1",O_RDWR|O_NONBLOCK);
    if(gps_fd>=0) init_gps(gps_fd);

    // NB: this depends on inited GPS!
    init_control();

    // Init USB IO
    init_usb("usb", O_ECHO);
    setvbuf(stdout,0,_IONBF,0);

    /* Main loop I/O state */
    char iobuf[2][128];
    char* cur_iob = iobuf[0];
    char* prev_iob = iobuf[1];
    char* tmp;
    cur_iob[0] = '\0';
    prev_iob[0] = '\0';
    int i;
    int usb_fd = -1;

    int happy_usb = 0; // Whether we have an active usb connection
    while (1) {
        delay();  // poll

        /* USB I/O logic: only active when connected to computer */
        if( USBDeviceState < CONFIGURED_STATE ) {
            happy_usb = 0;
            continue;
        }

        // USB just got configured
        if (!happy_usb) {
            /* Check if USB file is mounted */
            if( access("/dev/usb",F_OK) ) {
                // USB doesn't exist!  
                continue;
            }
    
            /* Open USB stream */
            if( (usb_fd = open("/dev/usb",O_RDWR)) == -1 ) {
                continue; // Fail to open
            }
    
            /* Make stdin, stdout, go through usb */
            dup2(usb_fd,0);
            dup2(usb_fd,1);
            close(usb_fd);
            usb_fd = 0;

            happy_usb = 1;
        }

        write(1,"> ",2);
        i = fdgets(usb_fd,cur_iob,128);
        if(i) {
            handle_cmd(cur_iob);
            tmp = prev_iob;
            prev_iob = cur_iob;
            cur_iob = tmp;
        } else if(strlen(prev_iob))  {
            handle_cmd(prev_iob);
        }
    }

    // Unrecoverable error, reset.
    Reset();
}

// Handles a command received from stdin 
static void handle_cmd(char * cmd) {
    // burn whitespace
    while( *cmd && isspace(*cmd) ) ++cmd;

    /* Find command handler */
    if( !strncmp(cmd,"adc",3) ) {
        adc_cmd(cmd+3);
    } else if( !strncmp(cmd,"cmon",4) ) {
        control_mon();
    } else if( !strncmp(cmd,"dbg",3) ) {
        dump_dbg(1 /*stdout*/);  
    } else if( !strncmp(cmd,"err",3) ) {
        print_errors();
    } else if( !strncmp(cmd,"gps",3) ) {
        gps_cmd();
    } else if( !strncmp(cmd,"help",4) ) {
        help();
    } else if( !strncmp(cmd,"load",4) ) {
        print_load();
    } else if( !strncmp(cmd,"press",5) ) {
        pressure();
    } else if( !strncmp(cmd,"pwm",3) ) {
        pwm_cmd();
    } else if( !strncmp(cmd,"reset",5) ) {
        enter_bootloader();
    } else if( !strncmp(cmd,"sense",5) ) {
        sense();
    } else if ( !strncmp(cmd,"tm",2) ) {
        printf("System time : +%dms\n",sys_time());
        //printf("Microseconds: +%uus\n",(int)(sys_time_ns()/1000));
    } else {
        // unknown command!
        printf("Unknown command: %s\n",cmd);
    }
}

static void adc_cmd(char* args)
{

    unsigned int inputs;
    int frequency = 0;
    if( (sscanf(args," 0x%x %d", &inputs, &frequency) < 1) 
        && (sscanf(args," %d %d", &inputs, &frequency) < 1) ) 
    {
        printf("Must specify bitmask of inputs to read!\n");
        return;
    }

    adc_print(inputs, frequency);
}

static void enter_bootloader() {
    // Shut down USB
    USBModuleDisable();
    // Wait a while, so host gets the idea
    int i;
    for(i=0; i<0x1ffff; ++i) Nop();
    SoftwareKey = 0x12345678;
    // Unlock system
    Reset();
}

static void help() {
    printf("Command listing:\n"
           "adc <channel mask> [freq, Hz]\n"
           "cmon\n"
           "dbg\n"
           "err\n"
           "gps\n"
           "help\n"
           "load\n"
           "press\n"
           "reset\n"
           "sense\n"
           "tm\n");
}

static void print_errors() {
    if( !err_peek() ) {
        fdputs(1,"No errors have occurred.\n");
        return;
    }
    while( err_peek() ) {
        err_print(1,err_peek());
        err_remove();
        if(err_peek()) fdputs(1,"\n");
    }
}

static void pressure() {
    const press_msmt_t *msmt = press_data();
    if(!msmt) {
        printf("No pressure data has been acquired.\n");
        return;
    }
    printf("Timestamp   : %dms\n",msmt->tstamp);
    printf("Pressure    : %d\n",msmt->press);
    printf("Temperature : %d\n",msmt->temp);
    printf("Altitude    : %lf\n", msmt->alt_calc);
}

static void pwm_cmd() {
    // Set D1 to an output
    TRISDCLR = 0x2;

    int cycle = 0;
    int j;
    for(; cycle < 10000; ++cycle) {
        for(j=0; j<5000; ++j) {
            PORTDSET = 0x2;
        }
        for(j=0; j<10000; ++j) {
            PORTDCLR = 0x2;
        }
    }

}

static void gps_cmd() {
    const gps_data_t* gpsp = gps_data();
    if(!gpsp) {
        printf("No GPS data has been received.\n");
        return;
    }
    //printf("%s\n",gps_last_msg());
    printf( gpsp->gps_mode ? "Have Fix\n" : "No Fix\n" );
    printf("Timestamp : %dms\n",gpsp->tstamp );
    printf("Satellites: %d\n",gpsp->satellites);
    printf("UTC       : %02d:%02d:%02d.%03d\n",
           gpsp->utc_h,gpsp->utc_m,gpsp->utc_s,gpsp->utc_ms);
    printf("LAT       : %f deg\n",gpsp->latitude);
    printf("LON       : %f deg\n",gpsp->longitude);
    printf("ALT       : %f m\n",gpsp->altitude);
    printf("HDOP      : %f\n",gpsp->hdop);
}

static void sense() {
    while( swUser ) {
        if( gps_has_new() && press_has_new() ) {
            const gps_data_t * gps = gps_data();
            const press_msmt_t * press = press_data();
            printf("%06d,%d,%.06f,%.06f,%.06f,%06d,%06d,%04d\n",
                   gps->tstamp,
                   gps->gps_mode,
                   gps->latitude,
                   gps->longitude,
                   gps->altitude,
                   press->tstamp,
                   press->press,
                   press->temp
                  );
        }
        delay();
    }
}

static void control_mon() {
    while( swUser ) {
        print_control_state();
        delay();
    }
}

static void init_system()
{
    // Configure the proper PB frequency and the number of wait states
    SYSTEMConfigWaitStatesAndPB(80000000L);

    // Enable the cache for the best performance
    CheKseg0CacheOn();        
    mJTAGPortEnable(0);
    PMCONbits.ON = 0;
    
    //Initialize all of the LED pins
    mInitAllLEDs();
    mInitAllSwitches();

    // enable multi-vector interrupts
    INTEnableSystemMultiVectoredInt();
}


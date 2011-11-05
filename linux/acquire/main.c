/******************************************************************************
 * main.c
 * Copyright 2011 Iain Peet
 *
 * Simple program which instructs the ubw32 to begin making adc readings, then
 * read them and dump them to a file while also echoing them to stdout.
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

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

FILE* output = 0;
FILE* ubw = 0;

void usage(char* argv[]);  // prints usage info

const char* stop_cmd = 0;

void sigint_handler(int signal)
{
    signal = 0; // go away warning

    if(output) fclose(output);

    if(stop_cmd && strlen(stop_cmd) ) {
        fprintf(ubw, "%s\n", stop_cmd);
    }
    if(ubw) fclose(ubw);

    printf("\nExiting!\n");
    exit(0);
}

int main(int argc, char *argv[])
{
    if( argc != 5) {
        // Need exactly 4 parameters
        usage(argv);
        exit(1);
    }

    stop_cmd = argv[3];

    /* Attempt to open the output file */
    output = fopen(argv[4], "w");
    if(!output) {
        fprintf(stderr, "Could not open output file: %s\n", argv[4]);
        perror("fopen");
        exit(1);
    }

    /* Attempt to open the ubw */
    ubw = fopen(argv[1], "r+");
    if(!ubw) {
        fprintf(stderr, "Could not open ubw: %s\n", argv[1]);
        perror("fopen");
        exit(1);
    }

    // So we can close files and reset ubw properly on sigint.
    signal(SIGINT, sigint_handler);

    /* Issue the command */
    if( fprintf(ubw, "%s\n", argv[2]) < 0 ) {
        fprintf(stderr, "UBW Write Error!\n");
        perror("fprintf");
        exit(1);
    }

    char buf[256]; 

    /* Look for the echo of our command */
    while(1) {
        if(! fgets(buf, 256, ubw) ) {
            fprintf(stderr, "UBW Read Error!\n");
            perror("fgets");
            fclose(output);
            exit(1);
        }

        if( strstr(buf, argv[2]) ) break;
    }

    /* Start reading real output */
    while(1) {
        if(! fgets(buf, 256, ubw) ) {
            fprintf(stderr, "UBW Read Error!\n");
            perror("fgets");
            fclose(output);
            exit(1);
        }

        if( fprintf(output, "%s", buf) < 0 ) {
            fprintf(stderr, "Output file write error!\n");
            perror("fprintf");
            fclose(output);
            exit(1);
        }

        printf("%s", buf);
    }

    return 0;
}

void usage(char *argv[])
{
    printf("%s - Capture the output of a command sent to the UBW\n", argv[0]);
    printf("Usage: %s [ubw dev] [start cmd] [stop cmd] [output file]\n", argv[0]);
    printf("\n"
           "Options:\n"
           "[ubw dev]     Ubw32 device file name\n"
           "[start cmd]   Command whose output is to be captured.\n"
           "[stop cmd]    Command to issue when this program exits, used\n"
           "              to stop a continuously-printing command\n"
           "[output file] File to which results should be written\n");

}



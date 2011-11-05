/******************************************************************************
 * gnuplot.c
 * Copyright 2011 Iain Peet
 *
 * Implements simple helpers which push data to gnuplot
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "gnuplot.h"

// The format string used to construct temporary files for gnuplot
static const char *SERIES_FILENAME_FMT = "/tmp/ser%d.%d";

// File to write gnuplot commands to
static char *CMD_FILE_FMT = "/tmp/gnuplotcmd.%d";

static int _series_wr(gnuplot_series_t *series, int inx, int plot_inx);
static int _cmd_wr(int num_series, int plot_inx);

// Plot some series
void gnuplot_show(gnuplot_series_t series[], int num_series) {
    // unique plot index so plots don't thwomp each other
    static int plot_inx = 0;
    plot_inx++;

    /* Write the series data files */
    int i;
    for (i = 0; i < num_series; ++i) {
        _series_wr(series + i, i, plot_inx);
    }

    // Write the command file
    if (_cmd_wr(num_series, plot_inx)) return;

    // Fork a process for gnuplot
    int fork_ret = fork();
    if (fork_ret < 0) {
        perror("fork");
        fprintf(stderr, "Failed to fork\n");
        return;
    } else if (!fork_ret) {
        /* Child process, exec gnuplot */
        char *const cmd_filename = malloc(64);
        sprintf(cmd_filename, CMD_FILE_FMT, plot_inx);
        char *const gnuplot_args[] = {"gnuplot", "-p", cmd_filename, 0};
        execv("/usr/bin/gnuplot", gnuplot_args);
        // exec never returns if it succeeds
        perror("execvp");
        fprintf(stderr, "Failed to exec gnuplot");
        exit(1);
    }
}

// Write a single series to a file in /tmp
static int _series_wr(gnuplot_series_t *series, int inx, int plot_inx) {
    /* Open the file for writing */
    char filename[64];
    sprintf(filename, SERIES_FILENAME_FMT, inx, plot_inx);
    FILE* ser_file = fopen(filename, "w");
    if (!ser_file) {
        perror("fopen");
        fprintf(stderr, "Failed to open series file: %s\n", filename);
        return -1;
    }

    /* Write series data */
    unsigned i;
    for (i = 0; i < series->len; ++i) {
        /* Write a single line, containing one point */
        if (series->xvals) {
            fprintf(ser_file, "%f, %f\n", series->xvals[i], series->yvals[i]);
        } else {
            fprintf(ser_file, "%f\n", series->yvals[i]);
        }

        if (ferror(ser_file)) {
            perror("fprintf");
            fprintf(stderr, "Failed to write to series file: %s\n", filename);
            return -1;
        }
    }
    fclose(ser_file);

    return 0;
}

// Write a file containing plot commands for gnuplot
static int _cmd_wr(int num_series, int plot_inx) {
    char cmd_filename[64];
    sprintf(cmd_filename, CMD_FILE_FMT, plot_inx);
    FILE* cmd_file = fopen(cmd_filename, "w");
    if (!cmd_file) {
        perror("fopen");
        fprintf(stderr, "Failed to open command file: %s\n", cmd_filename);
        return -1;
    }

    /* Write the command */
    fprintf(cmd_file, "plot ");
    char ser_filename[32];
    int i;
    for (i = 0; i < num_series; ++i) {
        sprintf(ser_filename, SERIES_FILENAME_FMT, i, plot_inx);
        fprintf(cmd_file, "'%s' smooth bezier", ser_filename);
        if (i != (num_series - 1)) {
            fprintf(cmd_file, ", ");
        }
    }
    fprintf(cmd_file, "\n");

    if(ferror(cmd_file)) {
        perror("fprintf");
        fprintf(stderr, "Failed to write command file: %s\n", cmd_filename);
        return -1;
    }

    fclose(cmd_file);

    return 0;
}

// For testing this on its own
#ifdef TEST_GNUPLOT
int main() {
    double xvals[2][2] = { {1, 2}, {2, 3} };
    double yvals[2][2] = { {0, 1}, {4, 9} };
    gnuplot_series_t series[2] = {
        {yvals[0], xvals[0], 2},
        {yvals[1], xvals[1], 2} };
    gnuplot_show(series, 2);

    double xvals2[2][2] = { {1, 4}, {2, 3} };
    double yvals2[2][2] = { {2, 3}, {4, 9} };
    gnuplot_series_t series2[2] = {
        {yvals2[0], xvals2[0], 2},
        {yvals2[1], xvals2[1], 2} };
    gnuplot_show(series2, 2);
}
#endif


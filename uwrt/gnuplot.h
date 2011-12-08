/******************************************************************************
 * gnuplot.h`
 * Copyright 2011 Iain Peet
 *
 * Provides simple helpers which push data to gnuplot.
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

#ifndef UWRT_GNUPLOT_H_
#define UWRT_GNUPLOT_H_

#include <stddef.h>

typedef struct {
    double *yvals; //series y values
    double *xvals; //series x values.  Optional.
    size_t len;    //number of points in series.
} gnuplot_series_t;

/**
 * Plot data with gnuplot.
 * @param series The plot series to show.
 * @param num_series The number of series
 */
void gnuplot_show(gnuplot_series_t series[], int num_series);

#endif // UWRT_GNUPLOT_H_


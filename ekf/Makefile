#  Makefile
#  Copyright 2011 Iain Peet
# 
#  This is intended to be included by the root makefile.  Its purpose
#  is to provide a manifest of the source files available in this
#  directory.
# 
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>

# This gets the dir path that the including makefile used for this
# makefile.  This same path should be used for this folders sources.
# NB: immediate substitution is important, since MAKEFILE_LIST is volatile.
EKF_DIR:=$(dir $(lastword $(MAKEFILE_LIST)))

# Local Source file list:
EKF_LINUX_CC_SRCS:= \
	kalman.c \
	kalman_test.c

# Prepend the current directory name
EKF_LINUX_CC_SRCS:= $(patsubst %,$(EKF_DIR)%,$(EKF_LINUX_CC_SRCS))

# Append this dirs srcs to the appropriate src lists:
LINUX_CC_SRCS:=$(LINUX_CC_SRCS) $(EKF_LINUX_CC_SRCS)

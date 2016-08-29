#=================================================
# Makefile for optionalFunctions API
#-------------------------------------------------
# Copyright (C) 2008
#
# $Id: makefile 5089 2010-02-01 06:08:56Z ki4ma20 $
#=================================================

# macro

# compile rule
ifneq ($(and $(WITH_OPENCV),$(WITH_OPENGL)),)
BUILD_WITH_OPENCV_OPENGL=1
else
 ifdef WITH_OPENCV
BUILD_WITH_OPENCV=1
 else
    ifdef WITH_OPENGL
BUILD_WITH_OPENGL=1
    else
    endif
 endif
endif

# API source files
USING_OPENCV_API =
USING_OPENGL_API =

COMMON_API	 = environmentCtrl.cpp byte_arrayCtrl.cpp timeCtrl.cpp\
		   histogramCtrl.cpp socket_clientCtrl.cpp lidar_dataCtrl.cpp vlp16Ctrl.cpp

ifdef BUILD_WITH_OPENCV_OPENGL
API_SRC = $(USING_OPENCV_API) $(USING_OPENGL_API) $(COMMON_API)
else
ifdef BUILD_WITH_OPENCV
API_SRC = $(USING_OPENCV_API) $(COMMON_API)
else
ifdef BUILD_WITH_OPENGL
API_SRC = $(USING_OPENGL_API) $(COMMON_API)
else
API_SRC = $(COMMON_API)
endif
endif
endif

# header files
HEAD	= ${API_SRC:.cpp=.h}

# API object files
API_OBJ = ${API_SRC:.cpp=.o}

# compiler setting
ifdef ARM_EABIHF
CC	= arm-linux-gnueabihf-g++-4.6
else
CC	= g++
endif

# compile option
CFLAGS	= -g -Wall -Werror
ifdef WITH_OPENCV
CFLAGS = -DUSE_OPENCV -g -Wall -Werror
else
endif


# gneration rule
all:$(API_OBJ)

$(API_OBJ):$(HEAD)
.cpp.o:
	$(CC) $(CFLAGS) -c $<

# clean
clean:
	rm -f *.o core* *~

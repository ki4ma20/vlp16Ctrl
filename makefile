#===============================================================
# Makefile for program using optionalFunctions API
#---------------------------------------------------------------
# author : Kiyoshi MATSUO
# since	 : 2008/06/18
# $Id: 2016-08-18 18:38 +0900 $
#
#===============================================================

# macro

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

# source files

USING_OPENCV_SRC =

USING_OPENGL_SRC =

USING_OPENCV_OPENGL_SRC =

COMMON_SRC 	 = vlp16_control_test.cpp

ifdef BUILD_WITH_OPENCV_OPENGL
SRC 	= $(USING_OPENCV_SRC) $(USING_OPENGL_SRC) $(USING_OPENCV_OPENGL_SRC) $(COMMON_SRC)
else
ifdef BUILD_WITH_OPENCV
SRC 	= $(USING_OPENCV_SRC) $(COMMON_SRC)
else
ifdef BUILD_WITH_OPENGL
SRC	= $(USING_OPENGL_SRC) $(COMMON_SRC)
else
SRC 	= $(COMMON_SRC)
endif
endif
endif

# object tag
OBJ	= ${SRC:.cpp=.o}
TARGET  = ${SRC:.cpp=}

# directory path of library sources
LIB_DIR = lib/

# output directory of build image
TARGET_PUT = bin/

# API names
USING_OPENCV_API =
USING_OPENGL_API =

COMMON_API	 = $(LIB_DIR)environmentCtrl.cpp $(LIB_DIR)byte_arrayCtrl.cpp\
		   $(LIB_DIR)histogramCtrl.cpp\
		   $(LIB_DIR)timeCtrl.cpp $(LIB_DIR)socket_clientCtrl.cpp\
		   $(LIB_DIR)lidar_dataCtrl.cpp $(LIB_DIR)vlp16Ctrl.cpp

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

# API files
API_OBJ = ${API_SRC:.cpp=.o}

# compiler
ifdef ARM_EABIHF
CC	= arm-linux-gnueabihf-g++-4.6
else
CC	= g++
endif

# compile options
CFLAGS	= -g -O4 -Wall -Werror -I$(LIB_DIR) -fopenmp

USING_OPENGL_LIBS = -lglut -lGLU
USING_OPENCV_LIBS = -lopencv_photo -lopencv_highgui -lopencv_core -lopencv_imgproc -lopencv_features2d -lopencv_nonfree
COMMON_LIBS = -lm -lpthread -lrt

ifdef BUILD_WITH_OPENCV_OPENGL
LIBS =  $(COMMON_LIBS)  $(USING_OPENGL_LIBS) $(USING_OPENCV_LIBS)
CFLAGS = -DUSE_OPENCV -g -O4 -Wall -Werror -I$(LIB_DIR) -fopenmp
else
ifdef BUILD_WITH_OPENCV
LIBS = $(COMMON_LIBS) $(USING_OPENCV_LIBS)
CFLAGS = -DUSE_OPENCV -g -O4 -Wall -Werror -I$(LIB_DIR) -fopenmp
else
ifdef BUILD_WITH_OPENGL
LIBS = $(COMMON_LIBS) $(USING_OPENGL_LIBS)
CFLAGS = -g -O4 -Wall -Werror -I$(LIB_DIR) -fopenmp
else
LIBS = $(COMMON_LIBS)
CFLAGS = -g -O4 -Wall -Werror -I$(LIB_DIR) -fopenmp
endif
endif
endif

#
# build rule
#

# subsystem generation
all: $(TARGET) subsystem
	rm -f *.o
subsystem:
	cd $(LIB_DIR) && $(MAKE)

$(OBJ):subsystem $(HEAD)
.cpp.o:
	$(CC) $(CFLAGS) -c $<

all: $(TARGET)

# main build rule
$(TARGET): subsystem $(OBJ)
	$(CC) $(API_OBJ) $@.o -o $@ $(LIBS) $(CFLAGS) && mv $@ $(TARGET_PUT)

# make clean
clean:
	rm -f $(TARGET) *.o *~ core* && cd $(LIB_DIR) && make clean && cd ../$(TARGET_PUT) && rm -f $(TARGET)

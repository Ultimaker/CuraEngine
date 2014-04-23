#
# Makefile for CuraEngine
#

BUILD_DIR = build
SRC_DIR = src
LIBS_DIR = libs

BUILD_TYPE = RELEASE

VERSION ?= DEV
CXX ?= g++
CFLAGS += -c -flto -Wall -Wextra -Wold-style-cast -Woverloaded-virtual -std=c++11 -DVERSION=\"$(VERSION)\" -isystem libs

ifeq ($(BUILD_TYPE),DEBUG)
	CFLAGS+=-ggdb -Og -g
endif
ifeq ($(BUILD_TYPE),PROFILE)
	CFLAGS+= -pg
endif
ifeq ($(BUILD_TYPE),RELEASE)
	CFLAGS+= -O3 -fomit-frame-pointer
endif

LDFLAGS += -flto -Lbuild/ -lclipper

SOURCES_RAW = bridge.cpp comb.cpp gcodeExport.cpp infill.cpp inset.cpp layerPart.cpp main.cpp optimizedModel.cpp pathOrderOptimizer.cpp polygonOptimizer.cpp raft.cpp settings.cpp skin.cpp skirt.cpp slicer.cpp support.cpp timeEstimate.cpp
SOURCES_RAW += modelFile/modelFile.cpp utils/gettime.cpp utils/logoutput.cpp utils/socket.cpp
SOURCES = $(addprefix $(SRC_DIR)/,$(SOURCES_RAW))

OBJECTS_RAW = $(SOURCES_RAW:.cpp=.o)
OBJECTS = $(addprefix $(BUILD_DIR)/,$(OBJECTS_RAW))

DIRS = $(sort $(dir $(OBJECTS)))

EXECUTABLE = $(BUILD_DIR)/CuraEngine

ifeq ($(OS),Windows_NT)
	#For windows make it large address aware, which allows the process to use more then 2GB of memory.
	EXECUTABLE := $(EXECUTABLE).exe
	CFLAGS += -march=pentium4
	LDFLAGS += -Wl,--large-address-aware -lm -lwsock32
	MKDIR_PREFIX = mkdir 
	MKDIR_POSTFIX = 2> NUL
else
	MKDIR_PREFIX = mkdir -p
	MKDIR_POSTFIX = 
	UNAME := $(shell uname)
	ifeq ($(UNAME), Linux)
		OPEN_HTML=firefox
		LDFLAGS += --static
	endif
	ifeq ($(UNAME), Darwin)
		OPEN_HTML=open
		#For MacOS force to build
		CFLAGS += -force_cpusubtype_ALL -mmacosx-version-min=10.6 -arch x86_64 -arch i386
		LDFLAGS += -force_cpusubtype_ALL -mmacosx-version-min=10.6 -arch x86_64 -arch i386
	endif
endif

all: $(DIRS) $(SOURCES) $(EXECUTABLE)

$(BUILD_DIR)/libclipper.a: $(LIBS_DIR)/clipper/clipper.cpp
	$(CXX) $(CFLAGS) -o $(BUILD_DIR)/libclipper.a $(LIBS_DIR)/clipper/clipper.cpp

$(EXECUTABLE): $(OBJECTS) $(BUILD_DIR)/libclipper.a
	$(CXX) $(OBJECTS) -o $@ $(LDFLAGS)

$(DIRS):
	-$(MKDIR_PREFIX) "$@" $(MKDIR_POSTFIX)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CFLAGS) $< -o $@

test: $(EXECUTABLE)
	python tests/runtest.py $(abspath $(EXECUTABLE))

## clean stuff
clean:
	rm -f $(EXECUTABLE) $(OBJECTS)

help:
	@cat Makefile |grep \#\#| grep \: |cut -d\# -f3

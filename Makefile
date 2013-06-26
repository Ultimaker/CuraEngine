#
# Makefile for CuraEngine
#

# simplest working invocation to compile it
#g++ main.cpp modelFile/modelFile.cpp clipper/clipper.cpp -I. -o CuraEngine

CC = g++
CFLAGS += -I. -c -Wall -Wextra -O3 -fomit-frame-pointer
# also include debug symbols
#CFLAGS+=-ggdb
LDFLAGS +=
SOURCES = main.cpp settings.cpp modelFile/modelFile.cpp clipper/clipper.cpp 
OBJECTS = $(SOURCES:.cpp=.o)
EXECUTABLE = CuraEngine
UNAME := $(shell uname)

ifeq ($(UNAME), Linux)
	OPEN_HTML=firefox
endif
ifeq ($(UNAME), Darwin)
	OPEN_HTML=open
	#For MacOS force to build 
	CFLAGS += -force_cpusubtype_ALL -mmacosx-version-min=10.6 -arch x86_64 -arch i386
	LDFLAGS += -force_cpusubtype_ALL -mmacosx-version-min=10.6 -arch x86_64 -arch i386
endif
ifeq ($(UNAME), MINGW32_NT-6.1)
	#For windows make it large address aware, which allows the process to use more then 2GB of memory.
	CFLAGS += -march=pentium4
	LDFLAGS += -Wl,--large-address-aware
endif

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

tests: $(EXECUTABLE)
	python run_tests.py

## clean stuff
clean:
	rm -f $(EXECUTABLE) $(OBJECTS)

help:
	@cat Makefile |grep \#\#| grep \: |cut -d\# -f3

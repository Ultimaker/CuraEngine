#
# Makefile for Cura_SteamEngine
#

# simplest working invocation to compile it
#g++ main.cpp modelFile/modelFile.cpp clipper/clipper.cpp -I. -o Cura_SteamEngine

CC=g++
CFLAGS=-I. -c -Wall -O3 -fomit-frame-pointer
# also include debug symbols
#CFLAGS+=-ggdb
LDFLAGS=
SOURCES=main.cpp modelFile/modelFile.cpp clipper/clipper.cpp 
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=CuraEngine
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
ifeq ($(UNAME), MINGW32_NT6.1)
	#For windows make it large address aware, which allows the process to use more then 2GB of memory.
	CFLAGS += -march=pentium4
	LDFLAGS += -Wl,--large-address-aware
endif

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@


## gcode: will run several tests and output the gCode and loads the first 100,000 lines into an HTML5 preview tool
gcode: $(EXECUTABLE) httpServer
	./$(EXECUTABLE) --html
	rm -f html/gCodeViewer/output.gcode 2>/dev/null
	ln -f output.gcode html/gCodeViewer/
	$(OPEN_HTML) "http://localhost:8000/gCodeViewer/index.html?analyzeFile=output.gcode"

## layers: make 'layers' will output layer SVGs and loads it up in your browser.
layers: $(EXECUTABLE)
	./$(EXECUTABLE) --html
	$(OPEN_HTML) output.html

## tests: will run several tests and output the gCode
tests: $(EXECUTABLE) wolt testmodels klein
	./Cura_SteamEngine --verbose -i  ./testcase_models/woltBaseline.stl
	./Cura_SteamEngine --verbose -i ./testcase_models/woltNotFlat.stl
	./Cura_SteamEngine --verbose -i ./testcase_models/wolt_scaled200Perc.stl
	./Cura_SteamEngine --verbose -i ./testcase_models/wolt_smoothingOn.stl
	#find ./testcase_models/*.stl -exec ./$(EXECUTABLE) {} \|\| echo ====== FAILED EXIT CODE $? ====== \;
	rm html/gCodeViewer/output.gcode 2>/dev/null || echo
	head -n 100000 output.gcode > html/gCodeViewer/output.gcode

## robot: will slice the robot
robot: testmodels
	./Cura_SteamEngine --verbose -i ./testcase_models/ultirobot_Martijn.stl

## robot: will slice the klein bottle
klein: testmodels
	./Cura_SteamEngine --verbose -i ./testcase_models/kleinbottle_Dizingof.stl

wolt:
	./Cura_SteamEngine --verbose -i ./testcase_models/wolt.stl

testmodels:
	@ls testcase_models/ultirobot_Martijn.stl 2>/dev/null || \
	( echo Fetching ultirobot_Martijn.stl... && \
	wget -O testcase_models/ultirobot_Martijn.stl "http://www.thingiverse.com/download:36866" )
	@ls testcase_models/kleinbottle_Dizingof.stl 2>/dev/null || \
	( echo Fetching klein bottle stl file... && \
	wget -O testcase_models/kleinbottle_Dizingof.stl http://www.thingiverse.com/download:69728 )

## clean stuff
clean:
	rm -f $(EXECUTABLE) $(OBJECTS) output.html

clean_all: clean
	rm -f ./testcase_models/ultirobot_Martijn.stl kleinbottle_Dizingof.stl

httpServer:
	echo Starting simple webserver for online gCodeViewer...
	ps aux |grep SimpleHTTP 1>/dev/null || python -m SimpleHTTPServer 2>&1 1>/dev/null

help:
	@cat Makefile |grep \#\#| grep \: |cut -d\# -f3

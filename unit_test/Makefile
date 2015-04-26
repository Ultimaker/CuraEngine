
GPP ?= g++
OPTS ?= -O2  -std=c++11
		
CURAROOT=..
LIBS_LOC= $(CURAROOT)/libs
SRC_LOC= $(CURAROOT)/src
CLIPPER_LOC = $(LIBS_LOC)/clipper


test_polygon: test_polygon.o clipper.o
	$(GPP) -o $@ test_polygon.o clipper.o


test_polygon.o: test_polygon.cpp $(CLIPPER_LOC)/clipper.hpp $(SRC_LOC)/utils/polygon.h
	$(GPP) -o $@ $(OPTS) -c -I $(LIBS_LOC) -I$(SRC_LOC) $<


clipper.o: $(CLIPPER_LOC)/clipper.cpp $(CLIPPER_LOC)/clipper.hpp
	$(GPP) -o $@ $(OPTS) -c -I $(CLIPPER_LOC) $<


clipper_convex_hull.o: $(SRC_LOC)/utils/clipper_convex_hull.cpp $(SRC_LOC)/utils/clipper_convex_hull.h
	$(GPP) -o $@ $(OPTS) -c -I $(CLIPPER_LOC) $<


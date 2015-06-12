
DEFS+=-std=c++11

ACLOCAL_AMFLAGS=-I m4

AM_LDFLAGS=
AM_CPPFLAGS=

noinst_HEADERS = \
    $(top_srcdir)/autoclean.sh \
    $(top_srcdir)/autogen.sh \
    $(top_srcdir)/src/bridge.h \
    $(top_srcdir)/src/comb.h \
    $(top_srcdir)/src/commandSocket.h \
    $(top_srcdir)/src/debug.h \
    $(top_srcdir)/src/FffGcodeWriter.h \
    $(top_srcdir)/src/FffPolygonGenerator.h \
    $(top_srcdir)/src/fffProcessor.h \
    $(top_srcdir)/src/gcodeExport.h \
    $(top_srcdir)/src/gcodePlanner.h \
    $(top_srcdir)/src/infill.h \
    $(top_srcdir)/src/inset.h \
    $(top_srcdir)/src/layerPart.h \
    $(top_srcdir)/src/mesh.h \
    $(top_srcdir)/src/multiVolumes.h \
    $(top_srcdir)/src/pathOrderOptimizer.h \
    $(top_srcdir)/src/polygonOptimizer.h \
    $(top_srcdir)/src/raft.h \
    $(top_srcdir)/src/settingRegistry.h \
    $(top_srcdir)/src/settings.h \
    $(top_srcdir)/src/skin.h \
    $(top_srcdir)/src/skirt.h \
    $(top_srcdir)/src/sliceDataStorage.h \
    $(top_srcdir)/src/slicer.h \
    $(top_srcdir)/src/support.h \
    $(top_srcdir)/src/timeEstimate.h \
    $(top_srcdir)/src/weaveDataStorage.h \
    $(top_srcdir)/src/Weaver.h \
    $(top_srcdir)/src/Wireframe2gcode.h \
    $(top_srcdir)/src/modelFile/modelFile.h \
    $(top_srcdir)/src/utils/BucketGrid2D.h \
    $(top_srcdir)/src/utils/floatpoint.h \
    $(top_srcdir)/src/utils/gettime.h \
    $(top_srcdir)/src/utils/intpoint.h \
    $(top_srcdir)/src/utils/linearAlg2D.h \
    $(top_srcdir)/src/utils/logoutput.h \
    $(top_srcdir)/src/utils/polygon.h \
    $(top_srcdir)/src/utils/polygondebug.h \
    $(top_srcdir)/src/utils/polygonUtils.h \
    $(top_srcdir)/src/utils/socket.h \
    $(top_srcdir)/src/utils/string.h \
    $(top_srcdir)/Cura.proto \
    $(NULL)

bin_PROGRAMS=CuraEngine

%.pb.h %.pb.cc : %.proto
	protoc --proto_path=. --cpp_out=. $?

$(top_srcdir)/src/main.cpp: $(top_srcdir)/Cura.pb.h

CuraEngine_DATAS= \
    $(top_srcdir)/fdmprinter.json \
    $(NULL)

CuraEngine_SOURCES= \
    $(top_srcdir)/src/bridge.cpp \
    $(top_srcdir)/src/comb.cpp \
    $(top_srcdir)/src/gcodeExport.cpp \
    $(top_srcdir)/src/infill.cpp \
    $(top_srcdir)/src/inset.cpp \
    $(top_srcdir)/src/layerPart.cpp \
    $(top_srcdir)/src/main.cpp \
    $(top_srcdir)/src/pathOrderOptimizer.cpp \
    $(top_srcdir)/src/polygonOptimizer.cpp \
    $(top_srcdir)/src/raft.cpp \
    $(top_srcdir)/src/settings.cpp \
    $(top_srcdir)/src/skin.cpp \
    $(top_srcdir)/src/skirt.cpp \
    $(top_srcdir)/src/slicer.cpp \
    $(top_srcdir)/src/support.cpp \
    $(top_srcdir)/src/timeEstimate.cpp \
    $(top_srcdir)/src/modelFile/modelFile.cpp \
    $(top_srcdir)/src/utils/gettime.cpp \
    $(top_srcdir)/src/utils/logoutput.cpp \
    $(top_srcdir)/src/commandSocket.cpp \
    $(top_srcdir)/src/FffGcodeWriter.cpp \
    $(top_srcdir)/src/FffPolygonGenerator.cpp \
    $(top_srcdir)/src/gcodePlanner.cpp \
    $(top_srcdir)/src/mesh.cpp \
    $(top_srcdir)/src/multiVolumes.cpp \
    $(top_srcdir)/src/settingRegistry.cpp \
    $(top_srcdir)/src/Weaver.cpp \
    $(top_srcdir)/src/Wireframe2gcode.cpp \
    $(top_srcdir)/src/utils/polygonUtils.cpp \
    $(top_srcdir)/src/utils/polygon.cpp \
    $(top_srcdir)/Cura.pb.cc \
    $(NULL)

#sdsvr_CPPFLAGS = $(LIBCRC_CFLAGS) $(HUNSPELL_CFLAGS)
#sdsvr_LDFLAGS = $(LIBCRC_LDFLAGS) $(HUNSPELL_LDFLAGS)
#AM_CPPFLAGS += $(LIBCRC_CFLAGS)
#AM_LDFLAGS+= $(LIBCRC_LDFLAGS)

# because the mongoose.c will block at the socketpair function
# at the OpenShift host, so disable the socketpair here:
# -DNS_DISABLE_SOCKETPAIR=1 -DMONGOOSE_NO_SOCKETPAIR=1
DEFS+= \
    `getconf LFS_CFLAGS` \
    `getconf LFS64_CFLAGS` \
    -D_GNU_SOURCE \
    -D_FILE_OFFSET_BITS=64 \
    -DHAVE_MMAP64=1 \
    $(NULL)

AM_CPPFLAGS+= \
    -I$(top_srcdir)/src \
    -I$(top_builddir)/include/ \
    -I$(top_builddir)/src/modelFile \
    $(NULL)

#    -L$(top_builddir)/libs/clipper/
#-I$(top_builddir)/libs/

AM_LDFLAGS+= \
    -L$(top_builddir)/src/ \
    `getconf LFS_LDFLAGS` \
    `getconf LFS64_LDFLAGS` \
    `pkg-config --libs polyclipping` \
    `pkg-config --libs protobuf` -lArcus \
    $(NULL)

if DEBUG
# use "valgrind --tool=memcheck --leak-check=yes" to check memory leak, MemWatch will drag the program.
#DEFS+=-DMEMWATCH
DEFS+= -DDEBUG=1
AM_LDFLAGS+= -lbfd
AM_CPPFLAGS+=-g -Wall

else
AM_CPPFLAGS+=-O3 -Wall
endif

#AM_CPPFLAGS+=-fpermissive

EXT_FLAGS=
@MK@GITNUMTMP=$(shell cd "$(top_srcdir)"; A=$$(git show | head -n 1 | awk '{print $$2}'); echo $${A:0:7}; cd - > /dev/null )
#@MK@SVNNUMTMP=$(shell cd "$(top_srcdir)"; LC_ALL=C svn info | grep -i Revision | awk '{print $$2}'; cd - > /dev/null )
#@MK@ifeq ($(SVNNUMTMP),)
#EXT_FLAGS+= -DSVN_VERSION='"${GITNUMTMP}"'
#@MK@else
#EXT_FLAGS+= -DSVN_VERSION='"${SVNNUMTMP}"'
#@MK@endif
@MK@ifeq ($(GITNUMTMP),)
@MK@else
EXT_FLAGS+= -DSVN_VERSION='"${GITNUMTMP}"'
@MK@endif
DEFS+=$(EXT_FLAGS)

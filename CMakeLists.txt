project(CuraEngine)

cmake_minimum_required(VERSION 3.0.0)

find_package(Arcus REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR} libs)

add_library(clipper STATIC libs/clipper/clipper.cpp)

set(engine_SRCS
    src/bridge.cpp
    src/comb.cpp
    src/commandSocket.cpp
    src/gcodeExport.cpp
    src/gcodePlanner.cpp
    src/infill.cpp
    src/inset.cpp
    src/layerPart.cpp
    src/main.cpp
    src/mesh.cpp
    src/multiVolumes.cpp
    src/optimizedModel.cpp
    src/pathOrderOptimizer.cpp
    src/polygonOptimizer.cpp
    src/raft.cpp
    src/settings.cpp
    src/skin.cpp
    src/skirt.cpp
    src/slicer.cpp
    src/support.cpp
    src/timeEstimate.cpp

    src/modelFile/modelFile.cpp

    src/utils/gettime.cpp
    src/utils/logoutput.cpp
)

protobuf_generate_cpp(engine_PB_SRCS engine_PB_HEADERS Cura.proto)

add_executable(CuraEngine ${engine_SRCS} ${engine_PB_SRCS})
target_link_libraries(CuraEngine clipper Arcus)

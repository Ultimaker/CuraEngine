# Copyright (c) 2026 UltiMaker
# CuraEngine is released under the terms of the AGPLv3 or higher.

cmake_policy(SET CMP0091 NEW)  # For MSVC flags, will be ignored on non-Windows OS's
cmake_minimum_required(VERSION 3.20)
project(CuraEngine)
find_package(standardprojectsettings REQUIRED)  # imports the cmake module https://github.com/Ultimaker/conan-ultimaker-index/recipes/standardprojectsettings
AssureOutOfSourceBuilds()

option(ENABLE_ARCUS "Enable support for ARCUS" ON)
option(ENABLE_TESTING "Build with unit tests" OFF)
option(ENABLE_BENCHMARKS "Build with Benchmarks" OFF)
option(EXTENSIVE_WARNINGS "Build with all warnings" ON)
option(ENABLE_PLUGINS "Build with plugins" ON)
option(ENABLE_REMOTE_PLUGINS "Build with all warnings" OFF)
option(ENABLE_SENTRY "Send crash data via Sentry" OFF)
option(ENABLE_MORE_COMPILER_OPTIMIZATION_FLAGS "Enable more optimization flags" ON)
option(USE_SYSTEM_LIBS "Use the system libraries if available" OFF)
option(OLDER_APPLE_CLANG "Apple Clang <= 13 used" OFF)
option(ENABLE_THREADING "Enable threading support" ON)
option(ENABLE_CURAVIZ "Build with CuraViz toolbox" OFF)

if (${ENABLE_ARCUS} OR ${ENABLE_PLUGINS} OR ${ENABLE_CURAVIZ})
    find_package(protobuf REQUIRED)
endif ()

MESSAGE(STATUS "Building with plugins support: ${ENABLE_PLUGINS}")
if (${ENABLE_PLUGINS})
    find_package(asio-grpc REQUIRED)
    find_package(gRPC REQUIRED)
    find_package(curaengine_grpc_definitions REQUIRED)
    find_package(semver REQUIRED)
    MESSAGE(STATUS "Plugin secure remotes allowed: ${ENABLE_REMOTE_PLUGINS}")
endif ()

message(STATUS "Building with Arcus: ${ENABLE_ARCUS}")
if (${ENABLE_ARCUS})
    find_package(arcus REQUIRED)
    protobuf_generate_cpp(engine_PB_SRCS engine_PB_HEADERS Cura.proto)
endif ()

message(STATUS "Building with CuraViz: ${ENABLE_CURAVIZ}")
if (${ENABLE_CURAVIZ})
    find_package(curaviz_grpc_definitions REQUIRED)
endif ()

### Compiling CuraEngine ###
# First compile all of CuraEngine as library.

set(engine_SRCS # Except main.cpp.
        src/Application.cpp
        src/utils/conical_overhang.cpp
        src/ExtruderPlan.cpp
        src/ExtruderTrain.cpp
        src/FffGcodeWriter.cpp
        src/FffPolygonGenerator.cpp
        src/FffProcessor.cpp
        src/GCodePathConfig.cpp
        src/infill/Infill.cpp
        src/InfillOrderOptimizer.cpp
        src/InsetOrderOptimizer.cpp
        src/interlocking/InterlockingGenerator.cpp
        src/layerPart.cpp
        src/LayerPlan.cpp
        src/LayerPlanBuffer.cpp
        src/Mesh.cpp
        src/MeshGroup.cpp
        src/mesh_material_splitter/MeshMaterialSplitter.cpp
        src/utils/mold.cpp
        src/utils/multi_volumes.cpp
        src/PathOrdering.cpp
        src/PathAdapter.cpp
        src/PathOrderMonotonic.cpp
        src/Preheat.cpp
        src/PrintInformation.cpp
        src/prime_tower/PrimeTower.cpp
        src/prime_tower/PrimeTowerNormal.cpp
        src/prime_tower/PrimeTowerInterleaved.cpp
        src/utils/raft.cpp
        src/Scene.cpp
        src/SkinInfillAreaComputation.cpp
        src/SkirtBrim.cpp
        src/SupportInfillPart.cpp
        src/SupportLayer.cpp
        src/SupportStorage.cpp
        src/Slice.cpp
        src/SliceLayer.cpp
        src/SliceLayerPart.cpp
        src/SliceMeshStorage.cpp
        src/SliceDataStorage.cpp
        src/Slicer.cpp
        src/SlicerLayer.cpp
        src/utils/area_support.cpp
        src/TimeEstimateCalculator.cpp
        src/SlicedUVCoordinates.cpp
        src/TextureDataProvider.cpp
        src/TopSurface.cpp
        src/TreeSupportTipGenerator.cpp
        src/TreeModelVolumes.cpp
        src/TreeSupport.cpp
        src/TreeSupportElement.cpp
        src/WallsComputation.cpp
        src/WallToolPaths.cpp

        src/arachne/SkeletalTrapezoidation.cpp
        src/arachne/SkeletalTrapezoidationGraph.cpp
        src/arachne/STHalfEdge.cpp
        src/arachne/STHalfEdgeNode.cpp

        src/beading_strategy/BeadingStrategy.cpp
        src/utils/beading_strategy_factory.cpp
        src/beading_strategy/DistributedBeadingStrategy.cpp
        src/beading_strategy/LimitedBeadingStrategy.cpp
        src/beading_strategy/RedistributeBeadingStrategy.cpp
        src/beading_strategy/WideningBeadingStrategy.cpp
        src/beading_strategy/OuterWallInsetBeadingStrategy.cpp
        src/beading_strategy/InnerWallInsetBeadingStrategy.cpp

        src/bridge/bridge.cpp
        src/types/geometry/ExpansionRange.cpp
        src/types/geometry/SegmentOverlappingData.cpp
        src/types/geometry/TransformedSegment.cpp
        src/types/geometry/TransformedShape.cpp
        src/bridge/LineAnchoringScoringCriterion.cpp
        src/bridge/BridgeAngleScoringCriterion.cpp
        src/bridge/AlignShapeScoringCriterion.cpp

        src/communication/ArcusCommunication.cpp
        src/communication/ArcusCommunicationPrivate.cpp
        src/communication/CommandLine.cpp
        src/communication/EmscriptenCommunication.cpp
        src/communication/Listener.cpp

        src/gcode/GCodeExport.cpp
        src/gcode/FixedGCodePart.cpp
        src/gcode/ResolvedGCodePart.cpp
        src/gcode/GcodeTemplateResolver.cpp

        src/infill/AbstractLinesInfill.cpp
        src/infill/ImageBasedDensityProvider.cpp
        src/infill/NoZigZagConnectorProcessor.cpp
        src/infill/ZigzagConnectorProcessor.cpp
        src/infill/lightning/LightningDistanceField.cpp
        src/infill/lightning/LightningGenerator.cpp
        src/infill/lightning/LightningLayer.cpp
        src/infill/lightning/LightningTreeNode.cpp
        src/infill/RegularNGonalInfill.cpp
        src/infill/SierpinskiFill.cpp
        src/infill/SierpinskiFillProvider.cpp
        src/infill/SubDivCube.cpp
        src/infill/GyroidInfill.cpp
        src/infill/lightning/GroundingLocation.cpp

        src/routing/Comb.cpp
        src/routing/GCodePath.cpp
        src/routing/LinePolygonsCrossings.cpp
        src/routing/NozzleTempInsert.cpp
        src/routing/SpeedDerivatives.cpp

        src/plugins/broadcast_settings_request.cpp
        src/plugins/empty.cpp
        src/plugins/gcode_paths_modify_request.cpp
        src/plugins/gcode_paths_modify_response.cpp
        src/plugins/handshake_request.cpp
        src/plugins/handshake_response.cpp
        src/plugins/infill_generate_request.cpp
        src/plugins/infill_generate_response.cpp
        src/plugins/postprocess_request.cpp
        src/plugins/postprocess_response.cpp
        src/plugins/simplify_request.cpp
        src/plugins/simplify_response.cpp

        src/progress/Progress.cpp
        src/progress/ProgressStageEstimator.cpp

        src/types/AdaptiveLayer.cpp
        src/types/AdaptiveLayerHeights.cpp
        src/types/MeshPathConfigs.cpp
        src/types/PathConfigStorage.cpp
        src/types/SettingContainersEnvironmentAdapter.cpp
        src/Settings.cpp
        src/types/ZSeamConfig.cpp

        src/types/geometry/AABB.cpp
        src/types/geometry/AABB3D.cpp
        src/utils/channel.cpp
        src/types/geometry/Segment3LL.cpp
        src/types/CuraViz.cpp
        src/types/Date.cpp
        src/interlocking/DilationKernel.cpp
        src/types/geometry/ExtrusionJunction.cpp
        src/types/geometry/ExtrusionLine.cpp
        src/types/geometry/ExtrusionSegment.cpp
        src/types/TimeKeeper.cpp
        src/utils/linear_alg_2d.cpp
        src/types/geometry/ListPolyIt.cpp
        src/types/geometry/Matrix4x3D.cpp
        src/utils/MeshUtils.cpp
        src/types/geometry/MinimumSpanningTree.cpp
        src/types/OBJ.cpp
        src/types/geometry/PlanarPolygon3LL.cpp
        src/types/geometry/PolygonConnector.cpp
        src/types/geometry/PathsPointIndex.cpp
        src/types/geometry/PolygonsPointIndexSegmentLocator.cpp
        src/types/geometry/PolygonsSegmentIndex.cpp
        src/utils/polygon_utils.cpp
        src/types/geometry/PolylineStitcher.cpp
        src/types/geometry/Simplify.cpp
        src/types/SVG.cpp
        src/types/geometry/SpatialLookup.cpp
        src/types/geometry/SquareGrid.cpp
        src/types/ThreadPool.cpp
        src/types/ToolpathVisualizer.cpp
        src/utils/voronoi_utils.cpp
        src/types/geometry/VoxelGrid.cpp
        src/types/geometry/VoxelUtils.cpp
        src/types/geometry/MixedPolylineStitcher.cpp

        src/types/scoring/BestElementFinder.cpp
        src/types/scoring/CornerScoringCriterion.cpp
        src/types/scoring/DistanceScoringCriterion.cpp
        src/types/scoring/ExclusionAreaScoringCriterion.cpp
        src/types/scoring/PositionBasedScoringCriterion.cpp
        src/types/scoring/RandomScoringCriterion.cpp
        src/types/scoring/TextureScoringCriterion.cpp

        src/types/geometry/ClosedLinesSet.cpp
        src/types/geometry/OpenLinesSet.cpp
        src/types/geometry/Point2LL.cpp
        src/types/geometry/Point3LL.cpp
        src/types/geometry/Polygon.cpp
        src/types/geometry/Shape.cpp
        src/types/geometry/PointMatrix.cpp
        src/types/geometry/PointsSet.cpp
        src/types/geometry/SingleShape.cpp
        src/types/geometry/PartsView.cpp
        src/types/geometry/LinesSet.cpp
        src/types/geometry/Polyline.cpp
        src/types/geometry/ClosedPolyline.cpp
        src/types/geometry/MixedLinesSet.cpp
        src/types/geometry/MendedShape.cpp

        src/types/geometry/conversions/Point2D_Point2LL.cpp
)

add_library(_CuraEngine STATIC ${engine_SRCS} ${engine_PB_SRCS})
if (ENABLE_THREADING)
    use_threads(_CuraEngine)
endif ()

target_include_directories(_CuraEngine
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> # Include Cura.pb.h
)

target_compile_definitions(_CuraEngine
        PUBLIC
        $<$<BOOL:${ENABLE_ARCUS}>:ARCUS>
        $<$<BOOL:${ENABLE_PLUGINS}>:ENABLE_PLUGINS>
        $<$<BOOL:${ENABLE_CURAVIZ}>:ENABLE_CURAVIZ>
        $<$<AND:$<BOOL:${ENABLE_PLUGINS}>,$<BOOL:${ENABLE_REMOTE_PLUGINS}>>:ENABLE_REMOTE_PLUGINS>
        $<$<BOOL:${OLDER_APPLE_CLANG}>:OLDER_APPLE_CLANG>
        CURA_ENGINE_VERSION=\"${CURA_ENGINE_VERSION}\"
        CURA_ENGINE_HASH=\"${CURA_ENGINE_HASH}\"
        $<$<BOOL:${ENABLE_TESTING}>:BUILD_TESTS>
        PRIVATE
        $<$<BOOL:${WIN32}>:NOMINMAX>
        $<$<CONFIG:Debug>:ASSERT_INSANE_OUTPUT>
        $<$<CONFIG:Debug>:USE_CPU_TIME>
        $<$<CONFIG:Debug>:DEBUG>
)

enable_sanitizers(_CuraEngine)

if (${EXTENSIVE_WARNINGS})
    set_project_warnings(_CuraEngine)
endif ()

if (ENABLE_ARCUS)
    target_link_libraries(_CuraEngine PUBLIC arcus::arcus)
endif ()

find_package(mapbox-wagyu REQUIRED)
find_package(clipper REQUIRED)
find_package(RapidJSON REQUIRED)
find_package(stb REQUIRED)
find_package(Boost REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(range-v3 REQUIRED)
find_package(scripta REQUIRED)
find_package(PNG REQUIRED)
find_package(cura-formulae-engine REQUIRED)

if (NOT EMSCRIPTEN)
    find_package(TBB REQUIRED)
endif ()

if (ENABLE_SENTRY)
    find_package(sentry REQUIRED)
endif ()

if (ENABLE_TESTING)
    find_package(GTest REQUIRED)
endif ()

target_link_libraries(_CuraEngine
        PUBLIC
        spdlog::spdlog
        range-v3::range-v3
        fmt::fmt
        clipper::clipper
        mapbox-wagyu::mapbox-wagyu
        rapidjson
        stb::stb
        boost::boost
        scripta::scripta
        PNG::PNG
        cura-formulae-engine::cura-formulae-engine
        $<$<NOT:$<BOOL:${EMSCRIPTEN}>>:onetbb::onetbb>
        $<$<TARGET_EXISTS:semver::semver>:semver::semver>
        $<$<TARGET_EXISTS:curaengine_grpc_definitions::curaengine_grpc_definitions>:curaengine_grpc_definitions::curaengine_grpc_definitions>
        $<$<TARGET_EXISTS:curaviz_grpc_definitions::curaviz_grpc_definitions>:curaviz_grpc_definitions::curaviz_grpc_definitions>
        $<$<TARGET_EXISTS:asio-grpc::asio-grpc>:asio-grpc::asio-grpc>
        $<$<TARGET_EXISTS:grpc::grpc>:grpc::grpc>
        $<$<TARGET_EXISTS:protobuf::libprotobuf>:protobuf::libprotobuf>
        $<$<TARGET_EXISTS:sentry::sentry>:sentry::sentry>
        $<$<TARGET_EXISTS:GTest::gtest>:GTest::gtest>)

target_compile_definitions(_CuraEngine PRIVATE
        $<$<BOOL:${ENABLE_SENTRY}>:SENTRY_URL=\"${SENTRY_URL}\">
        SENTRY_ENVIRONMENT=\"${SENTRY_ENVIRONMENT}\"
)

if (NOT WIN32)
    add_executable(CuraEngine src/main.cpp) # Then compile main.cpp as separate executable, and link the library to it.
else ()
    message(STATUS "Using windres")
    set(RES_FILES "CuraEngine.rc")
    ENABLE_LANGUAGE(RC)
    if (NOT MSVC)
        SET(CMAKE_RC_COMPILER_INIT windres)
        SET(CMAKE_RC_COMPILE_OBJECT
                "<CMAKE_RC_COMPILER> <FLAGS> -O coff <DEFINES> -i <SOURCE> -o <OBJECT>"
        )
    endif ()
    add_executable(CuraEngine src/main.cpp ${RES_FILES}) # ..., but don't forget the glitter!
    if (ENABLE_SENTRY)
        set_target_properties(CuraEngine PROPERTIES LINK_FLAGS "/DEBUG:FULL")
    endif ()
endif (NOT WIN32)

if (ENABLE_THREADING)
    use_threads(CuraEngine)
endif ()

if (EMSCRIPTEN)
    message(STATUS "Building for Emscripten")
    target_link_options(_CuraEngine
            PUBLIC
            "SHELL:-sINVOKE_RUN=0"
            "SHELL:-sEXPORT_NAME=CuraEngine"
            "SHELL:-sEXPORTED_RUNTIME_METHODS=[callMain,FS]"
            "SHELL:-sFORCE_FILESYSTEM=1"
            "SHELL:-sALLOW_MEMORY_GROWTH=1"
            "SHELL:-sEXPORT_ES6=1"
            "SHELL:-sMODULARIZE=1"
            "SHELL:-sSINGLE_FILE=1"
            "SHELL:-sENVIRONMENT=web,worker,node"
            "SHELL:-sERROR_ON_UNDEFINED_SYMBOLS=0"
            "SHELL:-sWASM_BIGINT=1"
            "SHELL:-sSTACK_SIZE=196608"
            $<$<CONFIG:Debug>:SHELL:-sASSERTIONS=2>
            $<$<CONFIG:Debug>:SHELL:-sSAFE_HEAP=1>
            $<$<CONFIG:Debug>:SHELL:-sSTACK_OVERFLOW_CHECK=2>
            $<$<CONFIG:Debug>:SHELL:-g3>
            $<$<CONFIG:Debug>:SHELL:-gsource-map>
            "SHELL:-lembind"
            "SHELL:--emit-tsd CuraEngine.d.ts"
    )
endif ()

target_link_libraries(CuraEngine PRIVATE
        _CuraEngine
        $<$<TARGET_EXISTS:sentry::sentry>:sentry::sentry>
)
target_compile_definitions(CuraEngine PRIVATE
        $<$<BOOL:${ENABLE_SENTRY}>:SENTRY_URL=\"${SENTRY_URL}\">
        SENTRY_ENVIRONMENT=\"${SENTRY_ENVIRONMENT}\"
        VERSION=\"${CURA_ENGINE_VERSION}\"
)

# Compiling the test environment.
if (ENABLE_TESTING OR ENABLE_BENCHMARKS)
    set(TESTS_HELPERS_SRC tests/ReadTestPolygons.cpp)

    set(TESTS_SRC_ARCUS)
    if (ENABLE_ARCUS)
        list(APPEND TESTS_SRC_ARCUS
                ArcusCommunicationTest
                ArcusCommunicationPrivateTest)
        list(APPEND TESTS_HELPERS_SRC tests/arcus/MockSocket.cpp)
    endif ()

    add_library(test_helpers ${TESTS_HELPERS_SRC})
    target_compile_definitions(test_helpers PUBLIC $<$<BOOL:${BUILD_TESTING}>:BUILD_TESTS> $<$<BOOL:${ENABLE_ARCUS}>:ARCUS>)
    target_include_directories(test_helpers PUBLIC "include" ${CMAKE_BINARY_DIR}/generated)
    target_link_libraries(test_helpers PRIVATE
            _CuraEngine
            GTest::gtest
            GTest::gmock
            clipper::clipper
            mapbox-wagyu::mapbox-wagyu
            $<$<TARGET_EXISTS:curaengine_grpc_definitions::curaengine_grpc_definitions>:curaengine_grpc_definitions::curaengine_grpc_definitions>
            $<$<TARGET_EXISTS:asio-grpc::asio-grpc>:asio-grpc::asio-grpc>
            $<$<TARGET_EXISTS:grpc::grpc>:grpc::grpc>
            $<$<TARGET_EXISTS:protobuf::libprotobuf>:protobuf::libprotobuf>)
    if (ENABLE_ARCUS)
        target_link_libraries(test_helpers PUBLIC arcus::arcus)
    endif ()
endif ()

if (ENABLE_BENCHMARKS)
    add_subdirectory(benchmark)
    if (NOT WIN32)
        add_subdirectory(stress_benchmark)
    endif ()
endif ()

if (ENABLE_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif ()

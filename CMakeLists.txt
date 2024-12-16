# Copyright (c) 2022 Ultimaker B.V.
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

if (${ENABLE_ARCUS} OR ${ENABLE_PLUGINS})
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

### Compiling CuraEngine ###
# First compile all of CuraEngine as library.

set(engine_SRCS # Except main.cpp.
        src/Application.cpp
        src/bridge.cpp
        src/ConicalOverhang.cpp
        src/ExtruderTrain.cpp
        src/FffGcodeWriter.cpp
        src/FffPolygonGenerator.cpp
        src/FffProcessor.cpp
        src/GCodePathConfig.cpp
        src/infill.cpp
        src/InterlockingGenerator.cpp
        src/InsetOrderOptimizer.cpp
        src/layerPart.cpp
        src/mesh.cpp
        src/MeshGroup.cpp
        src/Mold.cpp
        src/multiVolumes.cpp
        src/path_ordering.cpp
        src/PathAdapter.cpp
        src/Preheat.cpp
        src/PrimeTower/PrimeTower.cpp
        src/PrimeTower/PrimeTowerNormal.cpp
        src/PrimeTower/PrimeTowerInterleaved.cpp
        src/raft.cpp
        src/Scene.cpp
        src/SkeletalTrapezoidation.cpp
        src/SkeletalTrapezoidationGraph.cpp
        src/skin.cpp
        src/SkirtBrim.cpp
        src/SupportInfillPart.cpp
        src/Slice.cpp
        src/sliceDataStorage.cpp
        src/slicer.cpp
        src/support.cpp
        src/timeEstimate.cpp
        src/TopSurface.cpp
        src/TreeSupportTipGenerator.cpp
        src/TreeModelVolumes.cpp
        src/TreeSupport.cpp
        src/WallsComputation.cpp
        src/WallToolPaths.cpp

        src/BeadingStrategy/BeadingStrategy.cpp
        src/BeadingStrategy/BeadingStrategyFactory.cpp
        src/BeadingStrategy/DistributedBeadingStrategy.cpp
        src/BeadingStrategy/LimitedBeadingStrategy.cpp
        src/BeadingStrategy/RedistributeBeadingStrategy.cpp
        src/BeadingStrategy/WideningBeadingStrategy.cpp
        src/BeadingStrategy/OuterWallInsetBeadingStrategy.cpp

        src/communication/ArcusCommunication.cpp
        src/communication/ArcusCommunicationPrivate.cpp
        src/communication/CommandLine.cpp
        src/communication/EmscriptenCommunication.cpp
        src/communication/Listener.cpp

        src/feature_generation/FeatureGenerator.cpp include/feature_generation/FeatureGenerator.h
        src/feature_generation/MeshFeatureGenerator.cpp include/feature_generation/MeshFeatureGenerator.h
        src/feature_generation/MeshInfillGenerator.cpp include/feature_generation/MeshInfillGenerator.h
        src/feature_generation/MeshInsetsGenerator.cpp include/feature_generation/MeshInsetsGenerator.h
        src/feature_generation/SkirtBrimAppender.cpp include/feature_generation/SkirtBrimAppender.h

        src/infill/ImageBasedDensityProvider.cpp
        src/infill/NoZigZagConnectorProcessor.cpp
        src/infill/ZigzagConnectorProcessor.cpp
        src/infill/LightningDistanceField.cpp
        src/infill/LightningGenerator.cpp
        src/infill/LightningLayer.cpp
        src/infill/LightningTreeNode.cpp
        src/infill/SierpinskiFill.cpp
        src/infill/SierpinskiFillProvider.cpp
        src/infill/SubDivCube.cpp
        src/infill/GyroidInfill.cpp

        src/plan_export/CommunicationExporter.cpp include/plan_export/CommunicationExporter.h
        src/plan_export/ConsoleExporter.cpp include/plan_export/ConsoleExporter.h
        src/plan_export/ConsumptionEstimationExporter.cpp include/plan_export/ConsumptionEstimationExporter.h
        src/plan_export/GCodeExporter.cpp include/plan_export/GCodeExporter.h
        src/plan_export/MultiExporter.cpp include/plan_export/MultiExporter.h
        src/plan_export/PlanExporter.cpp include/plan_export/PlanExporter.h

        src/path_planning/Comb.cpp include/path_planning/Comb.h
        src/path_planning/GCodePath.cpp include/path_planning/GCodePath.h
        src/path_planning/LinePolygonsCrossings.cpp
        src/path_planning/NozzleTempInsert.cpp include/path_planning/NozzleTempInsert.h
        src/path_planning/SpeedDerivatives.cpp include/path_planning/SpeedDerivatives.h

        include/print_operation/ContinuousExtruderMoveSequencePtr.h
        include/print_operation/ExtruderPlanPtr.h
        include/print_operation/FeatureExtrusionPtr.h
        include/print_operation/LayerPlanPtr.h
        include/print_operation/PrintOperationPtr.h
        src/print_operation/ContinuousExtruderMoveSequence.cpp include/print_operation/ContinuousExtruderMoveSequence.h
        src/print_operation/ExtruderMove.cpp include/print_operation/ExtruderMove.h
        src/print_operation/ExtruderPlan.cpp include/print_operation/ExtruderPlan.h
        src/print_operation/ExtrusionMove.cpp include/print_operation/ExtrusionMove.h
        src/print_operation/FeatureExtrusion.cpp include/print_operation/FeatureExtrusion.h
        src/print_operation/LayerPlan.cpp include/print_operation/LayerPlan.h
        src/print_operation/PrintPlan.cpp include/print_operation/PrintPlan.h
        src/print_operation/MeshFeatureExtrusion.cpp include/print_operation/MeshFeatureExtrusion.h
        src/print_operation/PrintOperation.cpp include/print_operation/PrintOperation.h
        src/print_operation/PrintOperationSequence.cpp include/print_operation/PrintOperationSequence.h
        src/print_operation/TravelMove.cpp include/print_operation/TravelMove.h
        src/print_operation/TravelRoute.cpp include/print_operation/TravelRoute.h
        src/print_operation/WallFeatureExtrusion.cpp include/print_operation/WallFeatureExtrusion.h

        include/operation_transformation/ChangeSequenceAction.h
        include/operation_transformation/ClosestStartPoint.h
        include/operation_transformation/StartCandidatePoint.h
        src/operation_transformation/LayerPlanTravelMovesInserter.cpp include/operation_transformation/LayerPlanTravelMovesInserter.h
        src/operation_transformation/AddTravelMovesProcessor.cpp include/operation_transformation/AddTravelMovesProcessor.h
        src/operation_transformation/BedAdhesionConstraintsGenerator.cpp include/operation_transformation/BedAdhesionConstraintsGenerator.h
        src/operation_transformation/DirectTravelMoveGenerator.cpp include/operation_transformation/DirectTravelMoveGenerator.h
        src/operation_transformation/ExtruderPlanScheduler.cpp include/operation_transformation/ExtruderPlanScheduler.h
        src/operation_transformation/FeatureExtrusionsConstraintsGenerator.cpp include/operation_transformation/FeatureExtrusionsConstraintsGenerator.h
        src/operation_transformation/FeatureExtrusionScheduler.cpp include/operation_transformation/FeatureExtrusionScheduler.h
        src/operation_transformation/InsertOperationsProcessor.cpp include/operation_transformation/InsertOperationsProcessor.h
        src/operation_transformation/MeshFeaturesConstraintsGenerator.cpp include/operation_transformation/MeshFeaturesConstraintsGenerator.h
        src/operation_transformation/MonotonicConstraintsGenerator.cpp include/operation_transformation/MonotonicConstraintsGenerator.h
        src/operation_transformation/MoveSequencesConstraintsGenerator.cpp include/operation_transformation/MoveSequencesConstraintsGenerator.h
        src/operation_transformation/PrintOperationProcessor.cpp include/operation_transformation/PrintOperationTransformer.h
        src/operation_transformation/TravelMoveGenerator.cpp include/operation_transformation/TravelMoveGenerator.h

        src/plugins/converters.cpp

        src/progress/Progress.cpp
        src/progress/ProgressStageEstimator.cpp

        src/settings/AdaptiveLayerHeights.cpp
        src/settings/FlowTempGraph.cpp
        src/settings/MeshPathConfigs.cpp
        src/settings/PathConfigStorage.cpp
        src/settings/Settings.cpp
        src/settings/ZSeamConfig.cpp

        src/utils/AABB.cpp
        src/utils/AABB3D.cpp
        src/utils/channel.cpp
        src/utils/Date.cpp
        src/utils/ExtrusionJunction.cpp
        src/utils/ExtrusionLine.cpp
        src/utils/ExtrusionSegment.cpp
        src/utils/gettime.cpp
        src/utils/linearAlg2D.cpp
        src/utils/ListPolyIt.cpp
        src/utils/Matrix4x3D.cpp
        src/utils/MinimumSpanningTree.cpp
        src/utils/PolygonConnector.cpp
        src/utils/PolygonsPointIndex.cpp
        src/utils/PolygonsSegmentIndex.cpp
        src/utils/polygonUtils.cpp
        src/utils/PolylineStitcher.cpp
        src/utils/Simplify.cpp
        src/utils/SVG.cpp
        src/utils/SquareGrid.cpp
        src/utils/ThreadPool.cpp
        src/utils/ToolpathVisualizer.cpp
        src/utils/VoronoiUtils.cpp
        src/utils/VoxelUtils.cpp
        src/utils/MixedPolylineStitcher.cpp
        src/utils/scoring/BestElementFinder.cpp
        src/utils/scoring/CornerScoringCriterion.cpp
        src/utils/scoring/DistanceScoringCriterion.cpp
        src/utils/scoring/ExclusionAreaScoringCriterion.cpp
        src/utils/scoring/RandomScoringCriterion.cpp

        src/geometry/Point2LL.cpp
        src/geometry/Point3LL.cpp
        src/geometry/Polygon.cpp
        src/geometry/Shape.cpp
        src/geometry/PointsSet.cpp
        src/geometry/SingleShape.cpp
        src/geometry/PartsView.cpp
        src/geometry/LinesSet.cpp
        src/geometry/Polyline.cpp
        src/geometry/ClosedPolyline.cpp
        src/geometry/MixedLinesSet.cpp
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
        $<$<TARGET_EXISTS:semver::semver>:semver::semver>
        $<$<TARGET_EXISTS:curaengine_grpc_definitions::curaengine_grpc_definitions>:curaengine_grpc_definitions::curaengine_grpc_definitions>
        $<$<TARGET_EXISTS:asio-grpc::asio-grpc>:asio-grpc::asio-grpc>
        $<$<TARGET_EXISTS:grpc::grpc>:grpc::grpc>
        $<$<TARGET_EXISTS:protobuf::libprotobuf>:protobuf::libprotobuf>
        $<$<TARGET_EXISTS:sentry::sentry>:sentry::sentry>
        $<$<TARGET_EXISTS:GTest::gtest>:GTest::gtest>)

target_compile_definitions(_CuraEngine PRIVATE
        $<$<BOOL:${ENABLE_SENTRY}>:SENTRY_URL=\"${SENTRY_URL}\">
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

if (CMAKE_CXX_PLATFORM_ID STREQUAL "emscripten")
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
            "SHELL:-sENVIRONMENT=web"
            "SHELL:-sERROR_ON_UNDEFINED_SYMBOLS=0"
            "SHELL:-sWASM_BIGINT=1"
            "SHELL:-sSTACK_SIZE=196608"
            $<$<CONFIG:Debug>:SHELL:-sASSERTIONS=2>
            $<$<CONFIG:Debug>:SHELL:-sSAFE_HEAP=1>
            $<$<CONFIG:Debug>:SHELL:-sSTACK_OVERFLOW_CHECK=2>
            $<$<CONFIG:Debug>:SHELL:-g3>
            $<$<CONFIG:Debug>:SHELL:-gsource-map>
            "SHELL:-lembind"
            "SHELL:--embind-emit-tsd CuraEngine.d.ts"
    )
endif ()

target_link_libraries(CuraEngine PRIVATE
        _CuraEngine
        $<$<TARGET_EXISTS:sentry::sentry>:sentry::sentry>
)
target_compile_definitions(CuraEngine PRIVATE
        $<$<BOOL:${ENABLE_SENTRY}>:SENTRY_URL=\"${SENTRY_URL}\">
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

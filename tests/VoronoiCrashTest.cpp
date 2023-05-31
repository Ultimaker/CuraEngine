// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "WallsComputation.h" //Unit under test.
#include "ReadTestPolygons.h"
#include "ReadTestSettings.h"
#include "settings/Settings.h" //Settings to generate walls with.
#include "sliceDataStorage.h" //Sl
#include "utils/polygon.h" //To create example polygons.

#include <filesystem>
#include <gtest/gtest.h>
#include <range/v3/view/join.hpp>
#include <unordered_set>

#include <scripta/logger.h>
#include <spdlog/spdlog.h>

namespace cura
{

class VoronoiCrashTest : public testing::TestWithParam<std::tuple<std::string, std::string>>
{
public:
};

/*!
 *
 */
TEST_P(VoronoiCrashTest, SectionsTest)
{
    const auto params = GetParam();
    const std::string polygon_file = std::get<0>(params);
    const std::string setting_file = std::get<1>(params);

    spdlog::info("Testing polygon: {}", polygon_file);
    spdlog::info("Testing with settings: {}", setting_file);

    std::vector<Polygons> shapes;
    auto read_poly = readTestPolygons(polygon_file, shapes);
    ASSERT_TRUE(read_poly);

    Settings settings;
    auto read_settings = readTestSettings(setting_file, settings);
    ASSERT_TRUE(read_settings);

    size_t wall_count = settings.get<size_t>("wall_line_count");
    spdlog::info("wall_count: {}", wall_count);

    SliceLayer layer;
    for (const Polygons& shape : shapes)
    {
        layer.parts.emplace_back();
        SliceLayerPart& part = layer.parts.back();
        part.outline.add(shape);
    }

    LayerIndex layer_idx(100);
    WallsComputation walls_computation(settings, layer_idx);

    walls_computation.generateWalls(&layer, SectionType::WALL);
}

const std::vector<std::string> polygon_filenames = {
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/slice_polygon_001.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/slice_polygon_002.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/slice_polygon_003.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/slice_polygon_004.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/slice_polygon_005.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/slice_polygon_006.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/slice_polygon_007.txt").string(),
};

const std::vector<std::string> setting_filenames = {
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/settings_001.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/settings_002.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/settings_003.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/settings_004.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/settings_005.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/settings_006.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/settings_007.txt").string(),
};

INSTANTIATE_TEST_SUITE_P(TestCrashingPolygons, VoronoiCrashTest, testing::Combine(testing::ValuesIn(polygon_filenames), testing::ValuesIn(setting_filenames)));

} // namespace cura

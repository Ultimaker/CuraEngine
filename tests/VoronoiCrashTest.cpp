// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <boost/geometry/io/wkt/read.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>

#include "WallsComputation.h" //Unit under test.
#include "ReadTestSettings.h"
#include "settings/Settings.h" //Settings to generate walls with.
#include "sliceDataStorage.h" //Sl
#include "utils/polygon.h" //To create example polygons.

#include <fstream>
#include <filesystem>
#include <gtest/gtest.h>
#include <unordered_set>

#include <scripta/logger.h>
#include <spdlog/spdlog.h>

namespace cura
{

class VoronoiCrashTest : public testing::TestWithParam<std::tuple<std::string, std::string>>
{
public:
};

std::vector<Polygons> multiPolygonsFromWkt(const std::string& wkt)
{
    typedef boost::geometry::model::d2::point_xy<double> point_type;
    typedef boost::geometry::model::polygon<point_type> polygon_type;
    typedef boost::geometry::model::multi_polygon<polygon_type> multi_polygon_type;

    multi_polygon_type boost_polygons {};
    boost::geometry::read_wkt(wkt, boost_polygons);

    std::vector<Polygons> polygons;

    for (const auto& boost_polygon: boost_polygons)
    {
        Polygons polygon;

        Polygon outer;
        for (const auto& point: boost_polygon.outer())
        {
            outer.add(Point(point.x(), point.y()));
        }
        polygon.add(outer);

        for (const auto& hole: boost_polygon.inners())
        {
            Polygon inner;
            for (const auto& point: hole)
            {
                inner.add(Point(point.x(), point.y()));
            }
            polygon.add(inner);
        }

        polygons.push_back(polygon);
    }
    return polygons;
}

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

    std::ifstream file(polygon_file);
    std::ostringstream ss;
    ss << file.rdbuf();
    const auto shapes = multiPolygonsFromWkt(ss.str());

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
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/slice_polygon_001.wkt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/slice_polygon_002.wkt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/slice_polygon_003.wkt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/slice_polygon_004.wkt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/slice_polygon_005.wkt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/slice_polygon_006.wkt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/slice_polygon_007.wkt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/slice_polygon_008.wkt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/slice_polygon_009.wkt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/slice_polygon_010.wkt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/slice_polygon_011.wkt").string(),
};

const std::vector<std::string> setting_filenames = {
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/settings_001.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/settings_002.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/settings_003.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/settings_004.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/settings_005.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/settings_006.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/settings_007.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/settings_008.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/settings_009.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/settings_010.txt").string(),
    std::filesystem::path(__FILE__).parent_path().append("voronoi_crash_resources/settings_011.txt").string(),
};

INSTANTIATE_TEST_SUITE_P(TestCrashingPolygons, VoronoiCrashTest, testing::Combine(testing::ValuesIn(polygon_filenames), testing::ValuesIn(setting_filenames)));

} // namespace cura
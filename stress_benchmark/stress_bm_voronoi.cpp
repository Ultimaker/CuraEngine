// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <filesystem>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <source_location>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/read.hpp>
#include <fmt/format.h>
#include <range/v3/view/iota.hpp>
#include <spdlog/spdlog.h>

#include "WallsComputation.h" //Unit under test.
#include "settings/Settings.h" //Settings to generate walls with.
#include "sliceDataStorage.h" //Sl
#include "utils/polygon.h" //To create example polygons.

struct Resource
{
    std::filesystem::path wkt_file;
    std::filesystem::path settings_file;

    std::string stem() const
    {
        return wkt_file.stem().string();
    }

    std::vector<cura::Polygons> polygons() const
    {
        using point_type = boost::geometry::model::d2::point_xy<double>;
        using polygon_type = boost::geometry::model::polygon<point_type>;
        using multi_polygon_type = boost::geometry::model::multi_polygon<polygon_type>;

        multi_polygon_type boost_polygons{};
        std::ifstream file{ wkt_file };
        if (! file)
        {
            spdlog::error("Could not read shapes from: {}", wkt_file.string());
        }

        std::stringstream buffer;
        buffer << file.rdbuf();

        boost::geometry::read_wkt(buffer.str(), boost_polygons);

        std::vector<cura::Polygons> polygons;

        for (const auto& boost_polygon : boost_polygons)
        {
            cura::Polygons polygon;

            cura::Polygon outer;
            for (const auto& point : boost_polygon.outer())
            {
                outer.add(cura::Point(point.x(), point.y()));
            }
            polygon.add(outer);

            for (const auto& hole : boost_polygon.inners())
            {
                cura::Polygon inner;
                for (const auto& point : hole)
                {
                    inner.add(cura::Point(point.x(), point.y()));
                }
                polygon.add(inner);
            }

            polygons.push_back(polygon);
        }
        return polygons;
    }

    cura::Settings settings() const
    {
        cura::Settings settings;
        std::ifstream file{ settings_file };
        if (! file)
        {
            spdlog::error("Could not read settings from: {}", settings_file.string());
        }

        std::string line;
        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            std::string key;
            std::string value;

            if (std::getline(std::getline(iss, key, '='), value))
            {
                settings.add(key, value);
            }
        }
        return settings;
    }
};

std::vector<Resource> getResources()
{
    auto resource_path = std::filesystem::path(std::source_location::current().file_name()).parent_path().append("stress_bm_voronoi_resources");

    std::vector<Resource> resources;
    for (const auto& p : std::filesystem::recursive_directory_iterator(resource_path))
    {
        if (p.path().extension() == ".wkt")
        {
            auto settings = p.path();
            settings.replace_extension(".settings");
            spdlog::info("Adding resources for: {}", p.path().filename().stem().string());
            resources.emplace_back(Resource{ .wkt_file = p, .settings_file = settings });
        }
    }
    return resources;
};

int main()
{
    const auto resources = getResources();
    size_t crashCount = 0;

    for (const auto& resource : resources)
    {
        const auto& shapes = resource.polygons();
        const auto& settings = resource.settings();

        pid_t pid = fork();

        if (pid == -1)
        {
            spdlog::critical("Unable to fork");
            return 1;
        }

        if (pid == 0)
        {
            cura::SliceLayer layer;
            for (const cura::Polygons& shape : shapes)
            {
                layer.parts.emplace_back();
                cura::SliceLayerPart& part = layer.parts.back();
                part.outline.add(shape);
            }

            cura::LayerIndex layer_idx(100);
            cura::WallsComputation walls_computation(settings, layer_idx);

            walls_computation.generateWalls(&layer, cura::SectionType::WALL);
            exit(EXIT_SUCCESS);
        }
        else
        {
            int status;
            waitpid(pid, &status, 0);

            if (WIFSIGNALED(status))
            {
                ++crashCount;
                spdlog::error("Crash detected for: {}", resource.stem());
            }
        }
    }
    spdlog::info("Total number of crashes: {}", crashCount);
    return 0;
}
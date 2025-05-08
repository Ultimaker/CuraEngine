// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <csignal>
#include <docopt/docopt.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string_view>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/read.hpp>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <spdlog/spdlog.h>

#include "WallsComputation.h"
#include "geometry/OpenPolyline.h"
#include "geometry/Polygon.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "settings/Settings.h"
#include "sliceDataStorage.h"


constexpr std::string_view USAGE = R"(Stress Benchmark.

Executes a Stress Benchmark on CuraEngine.

Usage:
  stress_benchmark -o FILE
  stress_benchmark (-h | --help)
  stress_benchmark --version

Options:
  -h --help                      Show this screen.
  --version                      Show version.
  -o FILE                        Specify the output Json file.
)";

struct Resource
{
    std::filesystem::path wkt_file;
    std::filesystem::path settings_file;

    std::string stem() const
    {
        return wkt_file.stem().string();
    }

    std::vector<cura::Shape> polygons() const
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

        std::vector<cura::Shape> polygons;

        for (const auto& boost_polygon : boost_polygons)
        {
            cura::Shape polygon;

            cura::Polygon outer;
            for (const auto& point : boost_polygon.outer())
            {
                outer.push_back(cura::Point2LL(point.x(), point.y()));
            }
            polygon.push_back(outer);

            for (const auto& hole : boost_polygon.inners())
            {
                cura::Polygon inner;
                for (const auto& point : hole)
                {
                    inner.push_back(cura::Point2LL(point.x(), point.y()));
                }
                polygon.push_back(inner);
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
    auto resource_path = std::filesystem::path(std::source_location::current().file_name()).parent_path().append("resources");

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
}

void handleChildProcess(const auto& shapes, const auto& settings)
{
    cura::SliceLayer layer;
    for (const cura::Shape& shape : shapes)
    {
        layer.parts.emplace_back();
        cura::SliceLayerPart& part = layer.parts.back();
        part.outline.push_back(shape);
    }
    cura::LayerIndex layer_idx(100);
    cura::WallsComputation walls_computation(settings, layer_idx);
    walls_computation.generateWalls(&layer, cura::SectionType::WALL);
    exit(EXIT_SUCCESS);
}

size_t checkCrashCount(size_t crashCount, int status, const auto& resource)
{
    if (WIFSIGNALED(status))
    {
        ++crashCount;
        spdlog::error("# Crash detected for: {}", resource.stem());
    }
    else
    {
        spdlog::info("+ Test case {} processed normally", resource.stem());
    }
    return crashCount;
}

rapidjson::Value
    createRapidJSONObject(rapidjson::Document::AllocatorType& allocator, const std::string& test_name, const auto value, const std::string& unit, const std::string& extra_info)
{
    rapidjson::Value obj(rapidjson::kObjectType);
    rapidjson::Value key("name", allocator);
    rapidjson::Value val1(test_name.c_str(), test_name.length(), allocator);
    obj.AddMember(key, val1, allocator);
    key.SetString("unit", allocator);
    rapidjson::Value val2(unit.c_str(), unit.length(), allocator);
    obj.AddMember(key, val2, allocator);
    key.SetString("value", allocator);
    rapidjson::Value val3(value);
    obj.AddMember(key, val3, allocator);
    key.SetString("extra", allocator);
    rapidjson::Value val4(extra_info.c_str(), extra_info.length(), allocator);
    obj.AddMember(key, val4, allocator);
    return obj;
}

void createAndWriteJson(const std::filesystem::path& out_file, double stress_level, const std::string& extra_info, const size_t no_test_cases)
{
    rapidjson::Document doc;
    doc.SetArray();
    rapidjson::Document::AllocatorType& allocator = doc.GetAllocator();
    auto no_test_cases_obj = createRapidJSONObject(allocator, "Number of test cases", no_test_cases, "-", "");
    doc.PushBack(no_test_cases_obj, allocator);

    auto stress_obj = createRapidJSONObject(allocator, "General Stress Level", stress_level, "%", extra_info);
    doc.PushBack(stress_obj, allocator);

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer);

    spdlog::info("Writing Json results: {}", std::filesystem::absolute(out_file).string());
    std::ofstream file{ out_file };
    if (! file)
    {
        spdlog::critical("Failed to open the file: {}", out_file.string());
        exit(EXIT_FAILURE);
    }
    file.write(buffer.GetString(), buffer.GetSize());
    file.close();
}

int main(int argc, const char** argv)
{
    constexpr bool show_help = true;
    constexpr std::string_view version = "0.1.0";
    const std::map<std::string, docopt::value> args = docopt::docopt(fmt::format("{}", USAGE), { argv + 1, argv + argc }, show_help, fmt::format("{}", version));

    const auto resources = getResources();
    size_t crash_count = 0;
    std::vector<std::string> extra_infos;

    for (const auto& resource : resources)
    {
        const auto& shapes = resource.polygons();
        const auto& settings = resource.settings();

        spdlog::critical("Starting test case {}", resource.stem());
        pid_t engine_pid = fork();
        if (engine_pid == -1)
        {
            spdlog::critical("Unable to fork - engine");
            return EXIT_FAILURE;
        }
        else if (engine_pid == 0)
        {
            handleChildProcess(shapes, settings);
            return EXIT_SUCCESS;
        }
        else
        {
            pid_t waiter_pid = fork();
            if (waiter_pid == -1)
            {
                spdlog::critical("Unable to fork - waiter");
                return EXIT_FAILURE;
            }
            else if (waiter_pid == 0)
            {
                sleep(30);
                kill(engine_pid, SIGKILL);
                return EXIT_SUCCESS;
            }
            else
            {
                int status;
                waitpid(engine_pid, &status, 0);
                const auto old_crash_count = crash_count;
                crash_count = checkCrashCount(crash_count, status, resource);
                if (old_crash_count != crash_count)
                {
                    extra_infos.emplace_back(resource.stem());
                }
            }
        }
    }
    const double stress_level = static_cast<double>(crash_count) / static_cast<double>(resources.size()) * 100.0;
    spdlog::info("Stress level: {:.2f} [%]", stress_level);

    createAndWriteJson(std::filesystem::path{ args.at("-o").asString() }, stress_level, fmt::format("Crashes in: {}", fmt::join(extra_infos, ", ")), resources.size());
    return EXIT_SUCCESS;
}

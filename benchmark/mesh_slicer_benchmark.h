// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef CURAENGINE_BENCHMARK_MESH_SLICER_BENCHMARK_H
#define CURAENGINE_BENCHMARK_MESH_SLICER_BENCHMARK_H

#include <array>
#include <benchmark/benchmark.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numbers>
#include <vector>

#include <spdlog/spdlog.h>

#include "Application.h"
#include "MeshGroup.h"
#include "Slice.h"
#include "mesh.h"
#include "slicer.h"
#include "utils/Coord_t.h"
#include "utils/Matrix4x3D.h"

namespace cura
{

class MeshSlicerBenchmarkFixture : public benchmark::Fixture
{
public:
    struct StlVertex
    {
        float x;
        float y;
        float z;
    };

    std::filesystem::path cylinder_stl_path;
    std::filesystem::path torus_stl_path;

    static void writeProceduralTorusBinaryStl(const std::filesystem::path& path, const size_t major_steps, const size_t minor_steps)
    {
        constexpr float major_radius = 25.0F;
        constexpr float minor_radius = 8.0F;
        constexpr float z_offset = 10.0F;

        auto vertex_at = [&](const size_t i, const size_t j) -> StlVertex
        {
            const float u = static_cast<float>(2.0 * std::numbers::pi * static_cast<double>(i % major_steps) / static_cast<double>(major_steps));
            const float v = static_cast<float>(2.0 * std::numbers::pi * static_cast<double>(j % minor_steps) / static_cast<double>(minor_steps));
            const float r = major_radius + minor_radius * std::cos(v);
            return StlVertex{ r * std::cos(u), r * std::sin(u), minor_radius * std::sin(v) + z_offset };
        };

        const uint32_t face_count = static_cast<uint32_t>(major_steps * minor_steps * 2);
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        const std::array<char, 80> header{ "CuraEngine Binary STL Benchmark Torus" };
        out.write(header.data(), header.size());
        out.write(reinterpret_cast<const char*>(&face_count), sizeof(face_count));

        auto write_triangle = [&](const StlVertex& a, const StlVertex& b, const StlVertex& c)
        {
            std::array<char, 50> record{};
            const std::array<float, 12> coords{ 0.0F, 0.0F, 1.0F, a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z };
            std::memcpy(record.data(), coords.data(), sizeof(coords));
            out.write(record.data(), record.size());
        };

        for (size_t i = 0; i < major_steps; ++i)
        {
            for (size_t j = 0; j < minor_steps; ++j)
            {
                const StlVertex p00 = vertex_at(i, j);
                const StlVertex p10 = vertex_at(i + 1, j);
                const StlVertex p11 = vertex_at(i + 1, j + 1);
                const StlVertex p01 = vertex_at(i, j + 1);
                write_triangle(p00, p10, p11);
                write_triangle(p00, p11, p01);
            }
        }
    }

    void configureSceneSettings(Scene& scene)
    {
        scene.settings.add("layer_height_0", "0.2");
        scene.settings.add("layer_height", "0.1");
        scene.settings.add("layer_0_z_overlap", "0.0");
        scene.settings.add("raft_airgap", "0.0");
        scene.settings.add("raft_base_thickness", "0.2");
        scene.settings.add("raft_interface_thickness", "0.2");
        scene.settings.add("raft_interface_layers", "1");
        scene.settings.add("raft_surface_thickness", "0.2");
        scene.settings.add("raft_surface_layers", "1");
        scene.settings.add("raft_surface_extruder_nr", "0");
        scene.settings.add("magic_mesh_surface_mode", "normal");
        scene.settings.add("meshfix_extensive_stitching", "false");
        scene.settings.add("meshfix_keep_open_polygons", "false");
        scene.settings.add("minimum_polygon_circumference", "1");
        scene.settings.add("meshfix_maximum_resolution", "0.04");
        scene.settings.add("meshfix_maximum_deviation", "0.02");
        scene.settings.add("meshfix_maximum_extrusion_area_deviation", "2000");
        scene.settings.add("wall_transition_angle", "10");
        scene.settings.add("xy_offset", "0");
        scene.settings.add("xy_offset_layer_0", "0");
        scene.settings.add("hole_xy_offset", "0");
        scene.settings.add("hole_xy_offset_max_diameter", "0");
        scene.settings.add("support_mesh", "false");
        scene.settings.add("anti_overhang_mesh", "false");
        scene.settings.add("cutting_mesh", "false");
        scene.settings.add("infill_mesh", "false");
        scene.settings.add("adhesion_type", "none");
        scene.extruders.emplace_back(0, &scene.settings);
    }

    void SetUp(const ::benchmark::State& /*state*/) override
    {
        spdlog::set_level(spdlog::level::warn);
        Application::getInstance().startThreadPool();
        Application::getInstance().current_slice_ = std::make_shared<Slice>(1);
        configureSceneSettings(Application::getInstance().current_slice_->scene);

        cylinder_stl_path = std::filesystem::path(__FILE__).parent_path().parent_path() / "tests" / "integration" / "resources" / "cylinder1000.stl";
        torus_stl_path = std::filesystem::temp_directory_path() / "curaengine_benchmark_torus_100k.stl";
        if (! std::filesystem::exists(torus_stl_path))
        {
            writeProceduralTorusBinaryStl(torus_stl_path, 250, 200); // 100,000 manifold triangles, 50,000 vertices
        }
    }

    void TearDown(const ::benchmark::State& /*state*/) override
    {
        spdlog::set_level(spdlog::level::info);
    }
};

BENCHMARK_DEFINE_F(MeshSlicerBenchmarkFixture, load_mesh_cylinder1000)(benchmark::State& st)
{
    Scene& scene = Application::getInstance().current_slice_->scene;
    MeshGroup& mesh_group = scene.mesh_groups.back();
    const Matrix4x3D transformation;

    for (auto _ : st)
    {
        mesh_group.meshes.clear();
        bool ok = loadMeshIntoMeshGroup(&mesh_group, cylinder_stl_path, transformation, scene.settings);
        benchmark::DoNotOptimize(ok);
        benchmark::DoNotOptimize(mesh_group.meshes);
    }
}

BENCHMARK_REGISTER_F(MeshSlicerBenchmarkFixture, load_mesh_cylinder1000)->Unit(benchmark::kMillisecond);

BENCHMARK_DEFINE_F(MeshSlicerBenchmarkFixture, load_mesh_torus_100k)(benchmark::State& st)
{
    Scene& scene = Application::getInstance().current_slice_->scene;
    MeshGroup& mesh_group = scene.mesh_groups.back();
    const Matrix4x3D transformation;

    for (auto _ : st)
    {
        mesh_group.meshes.clear();
        bool ok = loadMeshIntoMeshGroup(&mesh_group, torus_stl_path, transformation, scene.settings);
        benchmark::DoNotOptimize(ok);
        benchmark::DoNotOptimize(mesh_group.meshes);
    }
}

BENCHMARK_REGISTER_F(MeshSlicerBenchmarkFixture, load_mesh_torus_100k)->Unit(benchmark::kMillisecond);

BENCHMARK_DEFINE_F(MeshSlicerBenchmarkFixture, slice_mesh_cylinder1000)(benchmark::State& st)
{
    Scene& scene = Application::getInstance().current_slice_->scene;
    MeshGroup& mesh_group = scene.mesh_groups.back();
    const Matrix4x3D transformation;
    loadMeshIntoMeshGroup(&mesh_group, cylinder_stl_path, transformation, scene.settings);
    Mesh& mesh = mesh_group.meshes[0];

    const auto layer_thickness = scene.settings.get<coord_t>("layer_height");
    const auto initial_layer_thickness = scene.settings.get<coord_t>("layer_height_0");
    const size_t num_layers = (mesh.getAABB().max_.z_ - initial_layer_thickness) / layer_thickness + 1;

    for (auto _ : st)
    {
        Slicer slicer(&mesh, layer_thickness, num_layers, false, nullptr, SlicingTolerance::MIDDLE, initial_layer_thickness);
        benchmark::DoNotOptimize(slicer.layers);
    }
}

BENCHMARK_REGISTER_F(MeshSlicerBenchmarkFixture, slice_mesh_cylinder1000)->Unit(benchmark::kMillisecond);

BENCHMARK_DEFINE_F(MeshSlicerBenchmarkFixture, slice_mesh_torus_100k)(benchmark::State& st)
{
    Scene& scene = Application::getInstance().current_slice_->scene;
    MeshGroup& mesh_group = scene.mesh_groups.back();
    const Matrix4x3D transformation;
    loadMeshIntoMeshGroup(&mesh_group, torus_stl_path, transformation, scene.settings);
    Mesh& mesh = mesh_group.meshes[0];

    const auto layer_thickness = scene.settings.get<coord_t>("layer_height");
    const auto initial_layer_thickness = scene.settings.get<coord_t>("layer_height_0");
    const size_t num_layers = (mesh.getAABB().max_.z_ - initial_layer_thickness) / layer_thickness + 1;

    for (auto _ : st)
    {
        Slicer slicer(&mesh, layer_thickness, num_layers, false, nullptr, SlicingTolerance::MIDDLE, initial_layer_thickness);
        benchmark::DoNotOptimize(slicer.layers);
    }
}

BENCHMARK_REGISTER_F(MeshSlicerBenchmarkFixture, slice_mesh_torus_100k)->Unit(benchmark::kMillisecond);

} // namespace cura

#endif // CURAENGINE_BENCHMARK_MESH_SLICER_BENCHMARK_H

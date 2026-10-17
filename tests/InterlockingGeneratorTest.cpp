// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "Application.h"
#include "ExtruderTrain.h"
#include "InterlockingGenerator.h"
#include "Slice.h"
#include "geometry/Shape.h"
#include "mesh.h"
#include "slicer.h"
#include "utils/Coord_t.h"

namespace cura
{

namespace
{

void addBox(Mesh& mesh, const coord_t x0, const coord_t y0, const coord_t x1, const coord_t y1, const coord_t z0, const coord_t z1)
{
    const Point3LL p000(x0, y0, z0);
    const Point3LL p100(x1, y0, z0);
    const Point3LL p110(x1, y1, z0);
    const Point3LL p010(x0, y1, z0);
    const Point3LL p001(x0, y0, z1);
    const Point3LL p101(x1, y0, z1);
    const Point3LL p111(x1, y1, z1);
    const Point3LL p011(x0, y1, z1);

    mesh.addFace(p000, p010, p110);
    mesh.addFace(p000, p110, p100);
    mesh.addFace(p001, p101, p111);
    mesh.addFace(p001, p111, p011);
    mesh.addFace(p000, p100, p101);
    mesh.addFace(p000, p101, p001);
    mesh.addFace(p010, p011, p111);
    mesh.addFace(p010, p111, p110);
    mesh.addFace(p000, p001, p011);
    mesh.addFace(p000, p011, p010);
    mesh.addFace(p100, p110, p111);
    mesh.addFace(p100, p111, p101);
    mesh.finish();
}

} // namespace

/*
 * Two overlapping one-layer meshes on different extruders. Interface dilation of
 * layer 0 emits grid cells below z = 0. Those cells cover no printable layer, but
 * indexing them used to wrap a size_t and crash.
 */
class InterlockingGeneratorTest : public testing::Test
{
    void SetUp() override
    {
        Application::getInstance().startThreadPool();
        Application::getInstance().current_slice_ = std::make_shared<Slice>(1);

        Scene& scene = Application::getInstance().current_slice_->scene;
        scene.settings.add("layer_height_0", "0.2");
        scene.settings.add("layer_height", "0.2");
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
        scene.extruders.emplace_back(1, &scene.settings);
    }

protected:
    void expectInterlocking(const char* depth, const char* boundary_avoidance, const bool expect_outline_change)
    {
        Scene& scene = Application::getInstance().current_slice_->scene;
        MeshGroup& mesh_group = scene.mesh_groups.back();
        mesh_group.settings.add("interlocking_orientation", "0");
        mesh_group.settings.add("interlocking_beam_layer_count", "2");
        mesh_group.settings.add("interlocking_depth", depth);
        mesh_group.settings.add("interlocking_boundary_avoidance", boundary_avoidance);

        mesh_group.meshes.reserve(2);
        Mesh& mesh_a = mesh_group.meshes.emplace_back(mesh_group.settings);
        Mesh& mesh_b = mesh_group.meshes.emplace_back(mesh_group.settings);
        mesh_a.settings_.add("wall_0_extruder_nr", "0");
        mesh_b.settings_.add("wall_0_extruder_nr", "1");
        for (Mesh* mesh : { &mesh_a, &mesh_b })
        {
            mesh->settings_.add("interlocking_beam_width", "0.4");
            mesh->settings_.add("min_wall_line_width", "0.1");
            mesh->settings_.add("line_width", "0.4");
        }

        constexpr coord_t mm = 1000;
        addBox(mesh_a, 0, 0, 10 * mm, 10 * mm, 0, 400);
        addBox(mesh_b, 5 * mm, 0, 15 * mm, 10 * mm, 0, 400);

        const coord_t layer_thickness = scene.settings.get<coord_t>("layer_height");
        const coord_t initial_layer_thickness = scene.settings.get<coord_t>("layer_height_0");
        Slicer slicer_a(&mesh_a, layer_thickness, 1, false, nullptr, SlicingTolerance::MIDDLE, initial_layer_thickness);
        Slicer slicer_b(&mesh_b, layer_thickness, 1, false, nullptr, SlicingTolerance::MIDDLE, initial_layer_thickness);

        ASSERT_EQ(slicer_a.layers.size(), 1u);
        ASSERT_EQ(slicer_b.layers.size(), 1u);
        const Shape before_a = slicer_a.layers[0].polygons_;
        const Shape before_b = slicer_b.layers[0].polygons_;
        ASSERT_GT(std::abs(before_a.area()), 0.0);
        ASSERT_GT(std::abs(before_b.area()), 0.0);

        std::vector<Slicer*> volumes{ &slicer_a, &slicer_b };
        InterlockingGenerator::generateInterlockingStructure(volumes);

        const Shape& after_a = slicer_a.layers[0].polygons_;
        const Shape& after_b = slicer_b.layers[0].polygons_;
        EXPECT_GT(std::abs(after_a.area()), 0.0);
        EXPECT_GT(std::abs(after_b.area()), 0.0);
        if (expect_outline_change)
        {
            EXPECT_GT(std::abs(before_a.xorPolygons(after_a).area()), 0.0);
            EXPECT_GT(std::abs(before_b.xorPolygons(after_b).area()), 0.0);
        }
        else
        {
            EXPECT_DOUBLE_EQ(std::abs(before_a.area()), std::abs(after_a.area()));
            EXPECT_DOUBLE_EQ(std::abs(before_b.area()), std::abs(after_b.area()));
        }
    }
};

TEST_F(InterlockingGeneratorTest, AvoidanceZeroDoesNotCrashAndStillInterlocks)
{
    expectInterlocking("2", "0", true);
}

TEST_F(InterlockingGeneratorTest, ShallowAvoidanceDoesNotCrashAndStillInterlocks)
{
    // Interface depth 2 reaches grid z = -1. Avoidance 1 does not, so those cells used to survive into handleThinAreas.
    expectInterlocking("2", "1", true);
}

TEST_F(InterlockingGeneratorTest, DeeperInterfaceThanAvoidanceDoesNotCrash)
{
    // Interface depth 4 reaches grid z = -2. Avoidance 2 only reaches grid z = -1, so those cells used to remain.
    // On this pair the printable interface sits on the outer skin, and avoidance 2 removes it, so the layer area stays put.
    expectInterlocking("4", "2", false);
}

} // namespace cura

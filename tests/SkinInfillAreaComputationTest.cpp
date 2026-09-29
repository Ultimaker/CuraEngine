// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include <gtest/gtest.h>

#include "Application.h"
#include "ExtruderTrain.h"
#include "Slice.h"
#include "geometry/Polygon.h"
#include "mesh.h"
#include "skin.h"
#include "sliceDataStorage.h"
#include "utils/Coord_t.h"

// NOLINTBEGIN(*-magic-numbers)
namespace cura
{

class SkinAreaProbe : public SkinInfillAreaComputation
{
public:
    SkinAreaProbe(const LayerIndex& layer_nr, SliceMeshStorage& mesh)
        : SkinInfillAreaComputation(layer_nr, mesh, true)
    {
    }

    void run()
    {
        generateSkinAndInfillAreas();
    }
};

class SkinInfillAreaComputationTest : public testing::Test
{
public:
    Polygon square_;

    SkinInfillAreaComputationTest()
    {
        square_.emplace_back(0, 0);
        square_.emplace_back(MM2INT(10), 0);
        square_.emplace_back(MM2INT(10), MM2INT(10));
        square_.emplace_back(0, MM2INT(10));

        Application::getInstance().current_slice_ = std::make_shared<Slice>(1);
        Scene& scene = Application::getInstance().current_slice_->scene;
        scene.extruders.emplace_back(0, &scene.settings);
        scene.extruders.back().settings_.add("initial_layer_line_width_factor", "100");
    }

    ~SkinInfillAreaComputationTest() override
    {
        Application::getInstance().current_slice_.reset();
    }

    static void addSettings(Mesh& mesh, const bool adaptive, const char* top_layers, const char* bottom_layers, const char* top_thickness, const char* bottom_thickness)
    {
        mesh.settings_.add("adaptive_layer_height_enabled", adaptive ? "true" : "false");
        mesh.settings_.add("top_layers", top_layers);
        mesh.settings_.add("bottom_layers", bottom_layers);
        mesh.settings_.add("initial_bottom_layers", bottom_layers);
        mesh.settings_.add("top_thickness", top_thickness);
        mesh.settings_.add("bottom_thickness", bottom_thickness);
        mesh.settings_.add("skin_line_width", "0.4");
        mesh.settings_.add("skin_no_small_gaps_heuristic", "true");
        mesh.settings_.add("top_skin_preshrink", "0");
        mesh.settings_.add("bottom_skin_preshrink", "0");
        mesh.settings_.add("top_skin_expand_distance", "0");
        mesh.settings_.add("bottom_skin_expand_distance", "0");
        mesh.settings_.add("min_skin_width_for_expansion", "0");
        mesh.settings_.add("min_infill_area", "0");
        mesh.settings_.add("top_bottom_extruder_nr", "0");
        mesh.settings_.add("cutting_mesh", "false");
        mesh.settings_.add("anti_overhang_mesh", "false");
        mesh.settings_.add("infill_mesh", "false");
    }

    void fillLayer(SliceLayer& layer, const coord_t thickness) const
    {
        layer.thickness = thickness;
        layer.parts.emplace_back();
        SliceLayerPart& part = layer.parts.back();
        part.outline.push_back(square_);
        part.inner_area.push_back(square_);
        part.boundaryBox.calculate(part.outline);
    }

    static void generate(SliceMeshStorage& mesh)
    {
        for (LayerIndex layer_nr = 0; layer_nr < static_cast<LayerIndex>(mesh.layers.size()); ++layer_nr)
        {
            SkinAreaProbe(layer_nr, mesh).run();
        }
    }

    static bool isSolidSkin(const SliceLayerPart& part)
    {
        return ! part.skin_parts.empty() && part.infill_area.empty();
    }

    static bool isInfillOnly(const SliceLayerPart& part)
    {
        return part.skin_parts.empty() && ! part.infill_area.empty();
    }
};

TEST_F(SkinInfillAreaComputationTest, ThinAdaptiveLayersKeepRequestedThickness)
{
    // Nominal counts are 2, which is what Cura sends for a 0.4 mm layer height.
    // Adaptive layers are 0.1 mm, so 0.8 mm top and 0.4 mm bottom need more layers.
    Mesh mesh;
    addSettings(mesh, true, "2", "2", "0.8", "0.4");
    SliceMeshStorage storage(&mesh, 20);
    for (SliceLayer& layer : storage.layers)
    {
        fillLayer(layer, MM2INT(0.1));
    }

    generate(storage);

    for (int layer = 0; layer < 20; ++layer)
    {
        const SliceLayerPart& part = storage.layers[static_cast<size_t>(layer)].parts.front();
        if (layer < 4 || layer >= 12)
        {
            EXPECT_TRUE(isSolidSkin(part)) << "layer " << layer;
        }
        else
        {
            EXPECT_TRUE(isInfillOnly(part)) << "layer " << layer;
        }
    }
}

TEST_F(SkinInfillAreaComputationTest, ThickAdaptiveLayersUseFewerSkinLayers)
{
    // Nominal counts are 8, as if the layer height were 0.1 mm. Actual layers are 0.4 mm.
    Mesh mesh;
    addSettings(mesh, true, "8", "8", "0.8", "0.8");
    SliceMeshStorage storage(&mesh, 10);
    for (SliceLayer& layer : storage.layers)
    {
        fillLayer(layer, MM2INT(0.4));
    }

    generate(storage);

    for (int layer = 0; layer < 10; ++layer)
    {
        const SliceLayerPart& part = storage.layers[static_cast<size_t>(layer)].parts.front();
        if (layer < 2 || layer >= 8)
        {
            EXPECT_TRUE(isSolidSkin(part)) << "layer " << layer;
        }
        else
        {
            EXPECT_TRUE(isInfillOnly(part)) << "layer " << layer;
        }
    }
}

TEST_F(SkinInfillAreaComputationTest, AdaptiveSkinRoundsUpAPartialLayer)
{
    // 0.3 mm layers. 0.8 mm top needs 3 layers (0.9 mm). 0.5 mm bottom needs 2 layers (0.6 mm).
    Mesh mesh;
    addSettings(mesh, true, "1", "1", "0.8", "0.5");
    SliceMeshStorage storage(&mesh, 12);
    for (SliceLayer& layer : storage.layers)
    {
        fillLayer(layer, MM2INT(0.3));
    }

    generate(storage);

    for (int layer = 0; layer < 12; ++layer)
    {
        const SliceLayerPart& part = storage.layers[static_cast<size_t>(layer)].parts.front();
        if (layer < 2 || layer >= 9)
        {
            EXPECT_TRUE(isSolidSkin(part)) << "layer " << layer;
        }
        else
        {
            EXPECT_TRUE(isInfillOnly(part)) << "layer " << layer;
        }
    }
}

TEST_F(SkinInfillAreaComputationTest, ThinFinalLayerKeepsThePenultimateLayerSolid)
{
    // The 0.1 mm final layer cannot cover a 0.4 mm top by itself.
    Mesh mesh;
    addSettings(mesh, true, "2", "0", "0.4", "0");
    SliceMeshStorage storage(&mesh, 10);
    for (SliceLayer& layer : storage.layers)
    {
        fillLayer(layer, MM2INT(0.4));
    }
    storage.layers.back().thickness = MM2INT(0.1);

    generate(storage);

    for (size_t layer = 0; layer < storage.layers.size(); ++layer)
    {
        const SliceLayerPart& part = storage.layers[layer].parts.front();
        if (layer >= 8)
        {
            EXPECT_TRUE(isSolidSkin(part)) << "layer " << layer;
        }
        else
        {
            EXPECT_TRUE(isInfillOnly(part)) << "layer " << layer;
        }
    }
}

TEST_F(SkinInfillAreaComputationTest, ThinLayerBelowThickTopDoesNotAddSkin)
{
    // The two 0.4 mm layers above layer 7 already cover the requested top thickness.
    Mesh mesh;
    addSettings(mesh, true, "2", "0", "0.8", "0");
    SliceMeshStorage storage(&mesh, 10);
    for (SliceLayer& layer : storage.layers)
    {
        fillLayer(layer, MM2INT(0.4));
    }
    storage.layers[7].thickness = MM2INT(0.1);

    generate(storage);

    for (size_t layer = 0; layer < storage.layers.size(); ++layer)
    {
        const SliceLayerPart& part = storage.layers[layer].parts.front();
        if (layer >= 8)
        {
            EXPECT_TRUE(isSolidSkin(part)) << "layer " << layer;
        }
        else
        {
            EXPECT_TRUE(isInfillOnly(part)) << "layer " << layer;
        }
    }
}

TEST_F(SkinInfillAreaComputationTest, ConstantLayerHeightKeepsConfiguredCounts)
{
    // Thickness settings would imply 8 layers, but adaptive layers are off.
    Mesh mesh;
    addSettings(mesh, false, "2", "2", "0.8", "0.8");
    SliceMeshStorage storage(&mesh, 20);
    for (SliceLayer& layer : storage.layers)
    {
        fillLayer(layer, MM2INT(0.1));
    }

    generate(storage);

    for (int layer = 0; layer < 20; ++layer)
    {
        const SliceLayerPart& part = storage.layers[static_cast<size_t>(layer)].parts.front();
        if (layer < 2 || layer >= 18)
        {
            EXPECT_TRUE(isSolidSkin(part)) << "layer " << layer;
        }
        else
        {
            EXPECT_TRUE(isInfillOnly(part)) << "layer " << layer;
        }
    }
}

} // namespace cura
// NOLINTEND(*-magic-numbers)

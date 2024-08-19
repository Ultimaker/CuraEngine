// Copyright (c) 2018 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "Mold.h"

#include <numbers>

#include "Application.h" //To get settings.
#include "ExtruderTrain.h"
#include "Scene.h"
#include "Slice.h"
#include "geometry/OpenPolyline.h"
#include "geometry/Point2LL.h"
#include "settings/types/Ratio.h"
#include "sliceDataStorage.h"
#include "slicer.h"

namespace cura
{

void Mold::process(std::vector<Slicer*>& slicer_list)
{
    Scene& scene = Application::getInstance().current_slice_->scene;
    { // check whether we even need to process molds
        bool has_any_mold = false;
        for (unsigned int mesh_idx = 0; mesh_idx < slicer_list.size(); mesh_idx++)
        {
            Mesh& mesh = scene.current_mesh_group->meshes[mesh_idx];
            if (mesh.settings_.get<bool>("mold_enabled"))
            {
                has_any_mold = true;
                mesh.expandXY(mesh.settings_.get<coord_t>("mold_width"));
            }
        }
        if (! has_any_mold)
        {
            return;
        }
    }

    LayerIndex layer_count = 0;
    { // compute layer_count
        for (unsigned int mesh_idx = 0; mesh_idx < slicer_list.size(); mesh_idx++)
        {
            Slicer& slicer = *slicer_list[mesh_idx];
            LayerIndex layer_count_here = slicer.layers.size();
            layer_count = std::max(layer_count, layer_count_here);
        }
    }

    const coord_t layer_height = scene.current_mesh_group->settings.get<coord_t>("layer_height");
    std::vector<Shape> mold_outline_above_per_mesh; // the outer outlines of the layer above without the original model(s) being cut out
    mold_outline_above_per_mesh.resize(slicer_list.size());
    for (int layer_nr = layer_count - 1; layer_nr >= 0; layer_nr--)
    {
        Shape all_original_mold_outlines; // outlines of all models for which to generate a mold (insides of all molds)

        // first generate outlines
        for (unsigned int mesh_idx = 0; mesh_idx < slicer_list.size(); mesh_idx++)
        {
            const Mesh& mesh = scene.current_mesh_group->meshes[mesh_idx];
            Slicer& slicer = *slicer_list[mesh_idx];
            if (! mesh.settings_.get<bool>("mold_enabled") || layer_nr >= static_cast<int>(slicer.layers.size()))
            {
                continue;
            }
            coord_t width = mesh.settings_.get<coord_t>("mold_width");
            coord_t open_polyline_width = mesh.settings_.get<coord_t>("wall_line_width_0");
            if (layer_nr == 0)
            {
                const ExtruderTrain& train_wall_0 = mesh.settings_.get<ExtruderTrain&>("wall_0_extruder_nr");
                open_polyline_width *= train_wall_0.settings_.get<Ratio>("initial_layer_line_width_factor");
            }
            const AngleDegrees angle = mesh.settings_.get<AngleDegrees>("mold_angle");
            const coord_t roof_height = mesh.settings_.get<coord_t>("mold_roof_height");

            const coord_t inset = tan(angle / 180 * std::numbers::pi) * layer_height;
            const size_t roof_layer_count = roof_height / layer_height;


            SlicerLayer& layer = slicer.layers[layer_nr];
            Shape model_outlines = layer.polygons_.unionPolygons(layer.open_polylines_.offset(open_polyline_width / 2));
            layer.open_polylines_.clear();
            all_original_mold_outlines.push_back(model_outlines);

            if (angle >= 90)
            {
                layer.polygons_ = model_outlines.offset(width, ClipperLib::jtRound);
            }
            else
            {
                Shape& mold_outline_above = mold_outline_above_per_mesh[mesh_idx]; // the outside of the mold on the layer above
                layer.polygons_ = mold_outline_above.offset(-inset).unionPolygons(model_outlines.offset(width, ClipperLib::jtRound));
            }

            // add roofs
            if (roof_layer_count > 0 && layer_nr > 0)
            {
                LayerIndex layer_nr_below = std::max(0, static_cast<int>(layer_nr - roof_layer_count));
                Shape roofs = slicer.layers[layer_nr_below].polygons_.offset(width, ClipperLib::jtRound); // TODO: don't compute offset twice!
                layer.polygons_ = layer.polygons_.unionPolygons(roofs);
            }

            mold_outline_above_per_mesh[mesh_idx] = layer.polygons_;
        }
        all_original_mold_outlines = all_original_mold_outlines.unionPolygons();

        // cut out molds from all objects after generating mold outlines for all objects so that molds won't overlap into the casting cutout of another mold

        // carve molds out of all other models
        for (unsigned int mesh_idx = 0; mesh_idx < slicer_list.size(); mesh_idx++)
        {
            const Mesh& mesh = scene.current_mesh_group->meshes[mesh_idx];
            if (! mesh.settings_.get<bool>("mold_enabled"))
            {
                continue; // only cut original models out of all molds
            }
            Slicer& slicer = *slicer_list[mesh_idx];
            SlicerLayer& layer = slicer.layers[layer_nr];
            layer.polygons_ = layer.polygons_.difference(all_original_mold_outlines);
        }
    }
}


} // namespace cura

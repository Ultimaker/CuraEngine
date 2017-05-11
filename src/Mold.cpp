/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#include "Mold.h"
#include "utils/intpoint.h"
#include "sliceDataStorage.h"

namespace cura
{

void Mold::process(const SliceDataStorage& storage, std::vector<Slicer*>& slicer_list, coord_t layer_height)
{
    { // check whether we even need to process molds
        bool has_any_mold = false;
        for (unsigned int mesh_idx = 0; mesh_idx < slicer_list.size(); mesh_idx++)
        {
            const Mesh& mesh = storage.meshgroup->meshes[mesh_idx];
            if (mesh.getSettingBoolean("mold_enabled"))
            {
                has_any_mold = true;
                break;
            }
        }
        if (!has_any_mold)
        {
            return;
        }
    }

    unsigned int layer_count = 0;
    { // compute layer_count
        for (unsigned int mesh_idx = 0; mesh_idx < slicer_list.size(); mesh_idx++)
        {
            Slicer& slicer = *slicer_list[mesh_idx];
            unsigned int layer_count_here = slicer.layers.size();
            layer_count = std::max(layer_count, layer_count_here);
        }
    }

    std::vector<Polygons> mold_outline_above_per_mesh; // the outer outlines of the layer above without the original model(s) being cut out
    mold_outline_above_per_mesh.resize(slicer_list.size());
    for (int layer_nr = layer_count - 1; layer_nr >= 0; layer_nr--)
    {
        Polygons all_original_mold_outlines; // outlines of all models for which to generate a mold (insides of all molds)

        // first generate outlines
        for (unsigned int mesh_idx = 0; mesh_idx < slicer_list.size(); mesh_idx++)
        {
            const Mesh& mesh = storage.meshgroup->meshes[mesh_idx];
            Slicer& slicer = *slicer_list[mesh_idx];
            if (!mesh.getSettingBoolean("mold_enabled") || layer_nr >= static_cast<int>(slicer.layers.size()))
            {
                continue;
            }
            coord_t width = mesh.getSettingInMicrons("mold_width");
            coord_t open_polyline_width = mesh.getSettingInMicrons("wall_line_width_0");
            double angle = mesh.getSettingInAngleDegrees("mold_angle");
            coord_t inset = tan(angle / 180 * M_PI) * layer_height;


            SlicerLayer& layer = slicer.layers[layer_nr];
            Polygons model_outlines = layer.polygons.unionPolygons(layer.openPolylines.offsetPolyLine(open_polyline_width / 2));
            all_original_mold_outlines.add(model_outlines);

            if (angle >= 90)
            {
                layer.polygons = model_outlines.offset(width, ClipperLib::jtRound);
            }
            else
            {
                Polygons& mold_outline_above = mold_outline_above_per_mesh[mesh_idx]; // the outside of the mold on the layer above
                layer.polygons = mold_outline_above.offset(-inset).unionPolygons(model_outlines.offset(width, ClipperLib::jtRound));
            }
            layer.openPolylines.clear();

            mold_outline_above_per_mesh[mesh_idx] = layer.polygons;
        }
        all_original_mold_outlines = all_original_mold_outlines.unionPolygons();

        // cut out molds from all objects after generating mold outlines for all objects so that molds won't overlap into the casting cutout of another mold

        // carve molds out of all other models
        for (unsigned int mesh_idx = 0; mesh_idx < slicer_list.size(); mesh_idx++)
        {
            Slicer& slicer = *slicer_list[mesh_idx];
            SlicerLayer& layer = slicer.layers[layer_nr];
            layer.polygons = layer.polygons.difference(all_original_mold_outlines);
        }
    }

}


}//namespace cura

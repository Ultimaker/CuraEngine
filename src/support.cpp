//Copyright (C) 2013 David Braam
//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <cmath> // sqrt
#include <utility> // pair
#include <deque>
#include <cmath> // round

#ifdef _OPENMP
#include <omp.h>
#endif // _OPENMP

#include "support.h"

#include "utils/math.h"
#include "progress/Progress.h"

namespace cura 
{

bool AreaSupport::handleSupportModifierMesh(SliceDataStorage& storage, const SettingsBaseVirtual& mesh, const Slicer* slicer)
{
    if (!mesh.getSettingBoolean("anti_overhang_mesh") && !mesh.getSettingBoolean("support_mesh"))
    {
        return false;
    }
    enum ModifierType { ANTI_OVERHANG, SUPPORT_DROP_DOWN, SUPPORT_VANILLA };
    ModifierType modifier_type = (mesh.getSettingBoolean("anti_overhang_mesh"))? ANTI_OVERHANG : ((mesh.getSettingBoolean("support_mesh_drop_down"))? SUPPORT_DROP_DOWN : SUPPORT_VANILLA);
    for (unsigned int layer_nr = 0; layer_nr < slicer->layers.size(); layer_nr++)
    {
        SupportLayer& support_layer = storage.support.supportLayers[layer_nr];
        const SlicerLayer& slicer_layer = slicer->layers[layer_nr];
        switch (modifier_type)
        {
        case ANTI_OVERHANG:
            support_layer.anti_overhang.add(slicer_layer.polygons);
            break;
        case SUPPORT_DROP_DOWN:
            support_layer.support_mesh_drop_down.add(slicer_layer.polygons);
            break;
        case SUPPORT_VANILLA:
            support_layer.support_mesh.add(slicer_layer.polygons);
            break;
        }
    }
    return true;
}


Polygons AreaSupport::join(const Polygons& supportLayer_up, Polygons& supportLayer_this, int64_t supportJoinDistance, int64_t smoothing_distance, int max_smoothing_angle, bool conical_support, int64_t conical_support_offset, int64_t conical_smallest_breadth)
{
    Polygons joined;
    if (conical_support)
    {
        Polygons insetted = supportLayer_up.offset(-conical_smallest_breadth/2);
        Polygons small_parts = supportLayer_up.difference(insetted.offset(conical_smallest_breadth/2+20));
        joined = supportLayer_this.unionPolygons(supportLayer_up.offset(conical_support_offset))
                                .unionPolygons(small_parts);
    }
    else 
    {
        joined = supportLayer_this.unionPolygons(supportLayer_up);
    }
    // join different parts
    if (supportJoinDistance > 0)
    {
        joined = joined.offset(supportJoinDistance)
                        .offset(-supportJoinDistance);
    }

    // remove jagged line pieces introduced by unioning separate overhang areas for consectuive layers
    //
    // support may otherwise look like:
    //      _____________________      .
    //     /                     \      } dist_from_lower_layer
    //    /__                   __\    /
    //      /''--...........--''\        `\                                                 .
    //     /                     \         } dist_from_lower_layer
    //    /__                   __\      ./
    //      /''--...........--''\     `\                                                    .
    //     /                     \      } dist_from_lower_layer
    //    /_______________________\   ,/
    //            rather than
    //      _____________________
    //     /                     \                                                          .
    //    /                       \                                                         .
    //    |                       |
    //    |                       |
    //    |                       |
    //    |                       |
    //    |                       |
    //    |_______________________|
    //
    // dist_from_lower_layer may be up to max_dist_from_lower_layer (see below), but that value may be extremely high
    joined = joined.smooth_outward(max_smoothing_angle, smoothing_distance);

    return joined;
}

void AreaSupport::generateSupportAreas(SliceDataStorage& storage, unsigned int layer_count)
{
    int max_layer_nr_support_mesh_filled;
    for (max_layer_nr_support_mesh_filled = storage.support.supportLayers.size() - 1; max_layer_nr_support_mesh_filled >= 0; max_layer_nr_support_mesh_filled--)
    {
        const SupportLayer& support_layer = storage.support.supportLayers[max_layer_nr_support_mesh_filled];
        if (support_layer.supportAreas.size() > 0)
        {
            break;
        }
    }
    storage.support.layer_nr_max_filled_layer = std::max(storage.support.layer_nr_max_filled_layer, max_layer_nr_support_mesh_filled);
    for (int layer_nr = 0; layer_nr < max_layer_nr_support_mesh_filled; layer_nr++)
    {
        SupportLayer& support_layer = storage.support.supportLayers[max_layer_nr_support_mesh_filled];
        support_layer.anti_overhang = support_layer.anti_overhang.unionPolygons();
        support_layer.support_mesh_drop_down = support_layer.support_mesh_drop_down.unionPolygons();
        support_layer.support_mesh = support_layer.support_mesh.unionPolygons();
    }

    // initialization of supportAreasPerLayer
    if (layer_count > storage.support.supportLayers.size())
    { // there might already be anti_overhang_area data or support mesh data in the supportLayers
        storage.support.supportLayers.resize(layer_count);
    }

    // generate support areas
    bool support_meshes_drop_down_handled = false;
    bool support_meshes_handled = false;
    for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        SliceMeshStorage& mesh = storage.meshes[mesh_idx];
        if (mesh.getSettingBoolean("infill_mesh") || mesh.getSettingBoolean("anti_overhang_mesh"))
        {
            continue;
        }
        SettingsBaseVirtual* infill_settings = &storage.meshes[mesh_idx];
        SettingsBaseVirtual* roof_settings = &storage.meshes[mesh_idx];
        SettingsBaseVirtual* bottom_settings = &storage.meshes[mesh_idx];
        if (mesh.getSettingBoolean("support_mesh"))
        {
            if ((mesh.getSettingBoolean("support_mesh_drop_down") && support_meshes_drop_down_handled) ||
                (!mesh.getSettingBoolean("support_mesh_drop_down") && support_meshes_handled) )
            { // handle all support_mesh and support_mesh_drop_down areas only once
                continue;
            }
            // use extruder train settings rather than the per-object settings of the first support mesh encountered.
            // because all support meshes are processed at the same time it doesn't make sense to use the per-object settings of the first support mesh encountered.
            // instead we must use the support extruder settings, which is the settings base common to all support meshes.
            int roof_extruder_nr = storage.getSettingAsIndex("support_roof_extruder_nr");
            int bottom_extruder_nr = storage.getSettingAsIndex("support_bottom_extruder_nr");
            int infill_extruder_nr = storage.getSettingAsIndex("support_infill_extruder_nr");
            infill_settings = storage.meshgroup->getExtruderTrain(infill_extruder_nr);
            roof_settings = storage.meshgroup->getExtruderTrain(roof_extruder_nr);
            bottom_settings = storage.meshgroup->getExtruderTrain(bottom_extruder_nr);
            if (mesh.getSettingBoolean("support_mesh_drop_down"))
            {
                support_meshes_drop_down_handled = true;
            }
            else
            {
                support_meshes_handled = true;
            }
        }
        std::vector<Polygons> supportAreas;
        supportAreas.resize(layer_count, Polygons());
        generateSupportAreas(storage, *infill_settings, *roof_settings, *bottom_settings, mesh_idx, layer_count, supportAreas);

        for (unsigned int layer_idx = 0; layer_idx < layer_count; layer_idx++)
        {
            storage.support.supportLayers[layer_idx].supportAreas.add(supportAreas[layer_idx]);
        }
    }

    for (unsigned int layer_idx = 0; layer_idx < layer_count ; layer_idx++)
    {
        Polygons& support_areas = storage.support.supportLayers[layer_idx].supportAreas;
        support_areas = support_areas.unionPolygons();
    }

    // handle support interface
    for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        SliceMeshStorage& mesh = storage.meshes[mesh_idx];
        if (mesh.getSettingBoolean("infill_mesh") || mesh.getSettingBoolean("anti_overhang_mesh"))
        {
            continue;
        }

        if (mesh.getSettingBoolean("support_roof_enable"))
        {
            generateSupportRoof(storage, mesh);
        }
        if (mesh.getSettingBoolean("support_bottom_enable"))
        {
            generateSupportBottom(storage, mesh);
        }
    }
}

/* 
 * Algorithm:
 * From top layer to bottom layer:
 * - find overhang by looking at the difference between two consucutive layers
 * - join with support areas from layer above
 * - subtract current layer
 * - use the result for the next lower support layer (without doing XY-distance and Z bottom distance, so that a single support beam may move around the model a bit => more stability)
 * - perform inset using X/Y-distance and bottom Z distance
 * 
 * for support buildplate only: purge all support not connected to buildplate
 */
void AreaSupport::generateSupportAreas(SliceDataStorage& storage, const SettingsBaseVirtual& infill_settings, const SettingsBaseVirtual& roof_settings, const SettingsBaseVirtual& bottom_settings, unsigned int mesh_idx, unsigned int layer_count, std::vector<Polygons>& supportAreas)
{
    const SliceMeshStorage& mesh = storage.meshes[mesh_idx];
        
    // given settings
    ESupportType support_type = storage.getSettingAsSupportType("support_type");

    const bool is_support_modifier_place_holder = mesh.getSettingBoolean("support_mesh"); // whether this mesh is an empty mesh and this function is only called to generate support for support meshes
    const bool is_support_mesh_nondrop_place_holder = is_support_modifier_place_holder && !mesh.getSettingBoolean("support_mesh_drop_down");
    const bool is_support_mesh_drop_down_place_holder = is_support_modifier_place_holder && mesh.getSettingBoolean("support_mesh_drop_down");

    if (!mesh.getSettingBoolean("support_enable") && !is_support_modifier_place_holder)
    {
        return;
    }
    if (support_type == ESupportType::NONE && !is_support_modifier_place_holder)
    {
        return;
    }

    const int layerThickness = storage.getSettingInMicrons("layer_height");


    int support_infill_extruder_nr = storage.getSettingAsIndex("support_infill_extruder_nr");

    const double supportAngle = ((mesh.getSettingBoolean("support_roof_enable"))? roof_settings : infill_settings).getSettingInAngleRadians("support_angle");
    const bool supportOnBuildplateOnly = support_type == ESupportType::PLATFORM_ONLY;
    const int supportZDistanceBottom = ((mesh.getSettingBoolean("support_bottom_enable"))? bottom_settings : infill_settings).getSettingInMicrons("support_bottom_distance");
    const int supportZDistanceTop = ((mesh.getSettingBoolean("support_roof_enable"))? roof_settings : infill_settings).getSettingInMicrons("support_top_distance");
    const unsigned int tower_top_layer_count = 6; // number of layers after which to conclude that a tiny support area needs a tower
    const coord_t support_bottom_stair_step_height = std::max(static_cast<coord_t>(0), mesh.getSettingInMicrons("support_bottom_stair_step_height"));
    const coord_t support_bottom_stair_step_width = std::max(static_cast<coord_t>(0), mesh.getSettingInMicrons("support_bottom_stair_step_width"));

    const int join_distance = infill_settings.getSettingInMicrons("support_join_distance");
    const int extension_offset = infill_settings.getSettingInMicrons("support_offset");

    const int supportTowerDiameter = infill_settings.getSettingInMicrons("support_tower_diameter");
    const int supportMinAreaSqrt = infill_settings.getSettingInMicrons("support_minimal_diameter");
    const double supportTowerRoofAngle = infill_settings.getSettingInAngleRadians("support_tower_roof_angle");
    const bool use_towers = infill_settings.getSettingBoolean("support_use_towers") && supportMinAreaSqrt > 0;

    const int supportXYDistance = infill_settings.getSettingInMicrons("support_xy_distance");
    const int support_xy_distance_overhang = infill_settings.getSettingInMicrons("support_xy_distance_overhang");
    const bool use_support_xy_distance_overhang = infill_settings.getSettingAsSupportDistPriority("support_xy_overrides_z") == SupportDistPriority::Z_OVERRIDES_XY; // whether to use a different xy distance at overhangs

    const double conical_support_angle = infill_settings.getSettingInAngleRadians("support_conical_angle");
    const bool conical_support = infill_settings.getSettingBoolean("support_conical_enabled") && conical_support_angle != 0;
    const int64_t conical_smallest_breadth = infill_settings.getSettingInMicrons("support_conical_min_width");

    // derived settings:
    const int max_smoothing_angle = 135; // maximum angle of inner corners to be smoothed
    coord_t smoothing_distance;
    { // compute best smoothing_distance
        ExtruderTrain& infill_train = *storage.meshgroup->getExtruderTrain(support_infill_extruder_nr);
        const coord_t support_infill_line_width = infill_train.getSettingInMicrons("support_line_width");
        smoothing_distance = support_infill_line_width;
        if (mesh.getSettingBoolean("support_roof_enable"))
        {
            const int support_roof_extruder_nr = storage.getSettingAsIndex("support_roof_extruder_nr");
            const ExtruderTrain& roof_train = *storage.meshgroup->getExtruderTrain(support_roof_extruder_nr);
            const coord_t support_roof_line_width = roof_train.getSettingInMicrons("support_roof_line_width");
            smoothing_distance = std::max(smoothing_distance, support_roof_line_width);
        }

        if (mesh.getSettingBoolean("support_bottom_enable"))
        {
            const int support_bottom_extruder_nr = storage.getSettingAsIndex("support_bottom_extruder_nr");
            const ExtruderTrain& bottom_train = *storage.meshgroup->getExtruderTrain(support_bottom_extruder_nr);
            const coord_t support_bottom_line_width = bottom_train.getSettingInMicrons("support_bottom_line_width");
            smoothing_distance = std::max(smoothing_distance, support_bottom_line_width);
        }
    }

    const int z_layer_distance_tower = 1; // start tower directly below overhang point
    
    
    int supportLayerThickness = layerThickness;
    
    const unsigned int layerZdistanceTop = std::max(0U, round_up_divide(supportZDistanceTop, supportLayerThickness)) + 1; // support must always be 1 layer below overhang
    const unsigned int bottom_empty_layer_count = std::max(0U, round_up_divide(supportZDistanceBottom, supportLayerThickness)); // number of empty layers between support and model
    const unsigned int bottom_stair_step_layer_count = support_bottom_stair_step_height / supportLayerThickness + 1; // the difference in layers between two stair steps. One is normal support (not stair-like)

    double tanAngle = tan(supportAngle) - 0.01;  // the XY-component of the supportAngle
    int max_dist_from_lower_layer = tanAngle * supportLayerThickness; // max dist which can be bridged
    
    int64_t conical_support_offset;
    if (conical_support_angle > 0) 
    { // outward ==> wider base than overhang
        conical_support_offset = -(tan(conical_support_angle) - 0.01) * supportLayerThickness;
    }
    else 
    { // inward ==> smaller base than overhang
        conical_support_offset = (tan(-conical_support_angle) - 0.01) * supportLayerThickness;
    }
    
    unsigned int support_layer_count = layer_count;
    
    double tanTowerRoofAngle = tan(supportTowerRoofAngle);
    int towerRoofExpansionDistance = layerThickness / tanTowerRoofAngle;
    
    
    // early out
    
    if ( layerZdistanceTop + 1 > support_layer_count )
    {
        return;
    }
    
    
    // computation
        
    
    std::vector<std::vector<Polygons>> overhang_points; // stores overhang_points of each layer
    if (use_towers && !is_support_modifier_place_holder)
    {
        AreaSupport::detectOverhangPoints(storage, mesh, overhang_points, layer_count, supportMinAreaSqrt);
    }

    std::vector<Polygons> xy_disallowed_per_layer;
    std::vector<Polygons> full_overhang_per_layer;
    xy_disallowed_per_layer.resize(support_layer_count);
    full_overhang_per_layer.resize(support_layer_count);
    #pragma omp parallel for default(none) shared(xy_disallowed_per_layer, full_overhang_per_layer, support_layer_count, storage, mesh, max_dist_from_lower_layer, tanAngle) schedule(dynamic)
    for (unsigned int layer_idx = 1; layer_idx < support_layer_count; layer_idx++)
    {
        Polygons outlines = storage.getLayerOutlines(layer_idx, false);
        if (!is_support_modifier_place_holder)
        { // don't compute overhang for support meshes
            std::pair<Polygons, Polygons> basic_and_full_overhang = computeBasicAndFullOverhang(storage, mesh, layer_idx, max_dist_from_lower_layer);
            full_overhang_per_layer[layer_idx] = basic_and_full_overhang.second;

            Polygons basic_overhang = basic_and_full_overhang.first;
            if (use_support_xy_distance_overhang)
            {
                Polygons xy_overhang_disallowed = basic_overhang.offset(supportZDistanceTop * tanAngle);
                Polygons xy_non_overhang_disallowed = outlines.difference(basic_overhang.offset(supportXYDistance)).offset(supportXYDistance);

                xy_disallowed_per_layer[layer_idx] = xy_overhang_disallowed.unionPolygons(xy_non_overhang_disallowed.unionPolygons(outlines.offset(support_xy_distance_overhang)));
            }
        }
        if (is_support_modifier_place_holder || !use_support_xy_distance_overhang)
        {
            xy_disallowed_per_layer[layer_idx] = outlines.offset(supportXYDistance);
        }
    }

    std::vector<Polygons> towerRoofs;
    Polygons stair_removal; // polygons to subtract from support because of stair-stepping

    for (unsigned int layer_idx = support_layer_count - 1 - layerZdistanceTop; layer_idx != (unsigned int) -1 ; layer_idx--)
    {
        Polygons supportLayer_this = full_overhang_per_layer[layer_idx + layerZdistanceTop];;

        if (extension_offset && !is_support_modifier_place_holder)
        {
            supportLayer_this = supportLayer_this.offset(extension_offset);
        }

        if (use_towers && !is_support_modifier_place_holder)
        {
            // handle straight walls
            AreaSupport::handleWallStruts(supportLayer_this, supportMinAreaSqrt, supportTowerDiameter);
            // handle towers
            AreaSupport::handleTowers(supportLayer_this, towerRoofs, overhang_points, layer_idx, towerRoofExpansionDistance, supportTowerDiameter, supportMinAreaSqrt, layer_count, z_layer_distance_tower);
        }

        if (layer_idx + 1 < support_layer_count)
        { // join with support from layer up
            const Polygons empty;
            const Polygons* support_layer_above = (layer_idx < supportAreas.size())? &supportAreas[layer_idx + 1] : &empty;
            if (is_support_mesh_nondrop_place_holder)
            {
                support_layer_above = &empty;
                supportLayer_this = supportLayer_this.unionPolygons(storage.support.supportLayers[layer_idx].support_mesh);
            }
            supportLayer_this = AreaSupport::join(*support_layer_above, supportLayer_this, join_distance, smoothing_distance, max_smoothing_angle, conical_support, conical_support_offset, conical_smallest_breadth);
        }

        // make towers for small support
        if (use_towers)
        {
            for (ConstPolygonRef poly : supportLayer_this)
            {
                if (poly.area() < supportMinAreaSqrt * supportMinAreaSqrt)
                {
                    if (layer_idx < support_layer_count - tower_top_layer_count && layer_idx >= tower_top_layer_count + bottom_empty_layer_count)
                    {
                        const Polygons& support_layer_above = supportAreas[layer_idx + tower_top_layer_count];
                        Point middle = AABB(poly).getMiddle();
                        bool has_support_above = support_layer_above.inside(middle);
                        bool has_model_below = storage.getLayerOutlines(layer_idx - tower_top_layer_count - bottom_empty_layer_count, false).inside(middle);
                        if (has_support_above && !has_model_below)
                        {
                            Polygons tiny_tower_here;
                            tiny_tower_here.add(poly);
                            towerRoofs.emplace_back(tiny_tower_here);
                        }
                    }
                }
            }
        }

        if (is_support_mesh_drop_down_place_holder && storage.support.supportLayers[layer_idx].support_mesh_drop_down.size() > 0)
        { // handle support mesh which should be supported by more support
            supportLayer_this = supportLayer_this.unionPolygons(storage.support.supportLayers[layer_idx].support_mesh_drop_down);
        }

        // Enforce XY distance before bottom distance,
        // because xy_offset might have introduced overlap between model and support,
        // which makes stair stepping conclude the support already rests on the model,
        // so it thinks it can make a step.

        // inset using X/Y distance
        if (supportLayer_this.size() > 0)
        {
            supportLayer_this = supportLayer_this.difference(xy_disallowed_per_layer[layer_idx]);
        }

        // move up from model
        moveUpFromModel(storage, stair_removal, supportLayer_this, layer_idx, bottom_empty_layer_count, bottom_stair_step_layer_count, support_bottom_stair_step_width);

        supportAreas[layer_idx] = supportLayer_this;

        Progress::messageProgress(Progress::Stage::SUPPORT, support_layer_count * (mesh_idx + 1) - layer_idx, support_layer_count * storage.meshes.size());
    }
    
    // do stuff for when support on buildplate only
    if (supportOnBuildplateOnly)
    {
        Polygons touching_buildplate = supportAreas[0]; // TODO: not working for conical support!
        for (unsigned int layer_idx = 1 ; layer_idx < storage.support.supportLayers.size() ; layer_idx++)
        {
            Polygons& supportLayer = supportAreas[layer_idx];
            
            if (conical_support)
            { // with conical support the next layer is allowed to be larger than the previous
                touching_buildplate = touching_buildplate.offset(std::abs(conical_support_offset) + 10, ClipperLib::jtMiter, 10); 
                // + 10 and larger miter limit cause performing an outward offset after an inward offset can disregard sharp corners
                //
                // conical support can make
                //  layer above    layer below
                //    v              v
                //  |               : |
                //  |        ==>    : |__
                //  |____           :....
                // 
                // a miter limit would result in
                //  | :             : |
                //  | :..    <==    : |__
                //  .\___           :....
                //
                
            }
            
            touching_buildplate = supportLayer.intersection(touching_buildplate); // from bottom to top, support areas can only decrease!
            
            supportAreas[layer_idx] = touching_buildplate;
        }
    }

    //Enforce top Z distance.
    if (layerZdistanceTop > 1)
    {
        // this is performed after the main support generation loop above, because it affects the joining of polygons
        // if this would be performed in the main loop then some support would not have been generated under the overhangs and consequently no support is generated for that,
        // meaning almost no support would be generated in some cases which definitely need support.
        const int max_checking_layer_idx = std::min(static_cast<int>(storage.support.supportLayers.size())
                                                  , static_cast<int>(support_layer_count - (layerZdistanceTop - 1)));
        const size_t max_checking_idx_size_t = std::max(0, max_checking_layer_idx);
#pragma omp parallel for default(none) shared(supportAreas, support_layer_count, storage) schedule(dynamic)
        for (size_t layer_idx = 0; layer_idx < max_checking_idx_size_t; layer_idx++)
        {
            supportAreas[layer_idx] = supportAreas[layer_idx].difference(storage.getLayerOutlines(layer_idx + layerZdistanceTop - 1, false));
        }
    }

    for (unsigned int layer_idx = supportAreas.size() - 1; layer_idx != (unsigned int) std::max(-1, storage.support.layer_nr_max_filled_layer) ; layer_idx--)
    {
        const Polygons& support_here = supportAreas[layer_idx];
        if (support_here.size() > 0)
        {
            storage.support.layer_nr_max_filled_layer = layer_idx;
            break;
        }
    }

    storage.support.generated = true;
}

void AreaSupport::moveUpFromModel(const SliceDataStorage& storage, Polygons& stair_removal, Polygons& support_areas, const int layer_idx, const int bottom_empty_layer_count, const unsigned int bottom_stair_step_layer_count, const coord_t support_bottom_stair_step_width)
{
// The idea behing support bottom stairs:
//
//   LEGEND:
//   A: support resting on model
//   x: stair step width
//   C: to be removed from support untill the next stair step = intersection between model below and A offsetted by x
//
//     ALGORITHM                                                      RESULT
//
// ###########################                                     ###########################                              .
//    support                                                         support                                               .
// ###########################   ┐                                 ###########################                              .
// AAAAxxxxxxxxxxxxxx            │                                                                                          .
// CCCCCCCCCCCC                  │                                 ____        ###############                              .
// |   \      :                  │                                 |   \                                                    .
// |____\     :                  ├> stair step height              |____\      ###############                              .
// |     \    :                  │                                 |     \                                                  .
// |______\   :                  │                                 |______\    ###############                              .
// |       \  :                  │                                 |       \                                                .
// |________\ :                  │                                 |________\  ###############                              .
// |model    \:                  │                                 |model    \                                              .
// |__________\###############   ┘                                 |__________\###############                              .
// |           \                                                   |           \                                            .
// |____________\                                                  |____________\                                           .
//
//
//
//
//       for more horizontal surface, the stepping is (partly) negated
//
// ############################################################################
//                                                    support
// ############################################################################     ┐
// AAAAAxxxxxxxxxxxxx                                                               │
// CCCCCCCCCCCCCCCCCC##########################################################     │           >>>>>>>>>   result only applies stair step to first layer(s) of what woud normally be the stair step
//      ^^--..__                                                                    │
//              ^^--..__#######################################################     ├> stair step height
// ⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺  ^^--..__                                                    │
//                              ^^--..__#######################################     │
// ⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺ ^^--..__                                    │
//                                              ^^--..__#######################     │
// ⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺^^--..__                    │
//                                                              ^^--..__#######     ┘
// ⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺
    if (layer_idx < bottom_empty_layer_count)
    {
        return;
    }
    if (bottom_empty_layer_count <= 0 && bottom_stair_step_layer_count <= 1)
    {
        return;
    }

    int bottom_layer_nr = layer_idx - bottom_empty_layer_count;
    const Polygons bottom_outline = storage.getLayerOutlines(bottom_layer_nr, false);

    Polygons to_be_removed;
    if (bottom_stair_step_layer_count <= 1)
    {
        to_be_removed = bottom_outline;
    }
    else
    {
        to_be_removed = stair_removal.unionPolygons(bottom_outline);
        if (layer_idx % bottom_stair_step_layer_count == 0)
        { // update stairs for next step
            const Polygons supporting_bottom = storage.getLayerOutlines(bottom_layer_nr - 1, false);
            const Polygons allowed_step_width = support_areas.intersection(supporting_bottom).offset(support_bottom_stair_step_width);

            int step_bottom_layer_nr = bottom_layer_nr - bottom_stair_step_layer_count + 1;
            if (step_bottom_layer_nr >= 0)
            {
                const Polygons step_bottom_outline = storage.getLayerOutlines(step_bottom_layer_nr, false);
                stair_removal = step_bottom_outline.intersection(allowed_step_width);
            }
            else
            {
                stair_removal = allowed_step_width;
            }
        }
    }
    support_areas = support_areas.difference(to_be_removed);
}


/*            layer 2
 * layer 1 ______________|
 * _______|         ^^^^^ basic overhang
 * 
 * ^^^^^^^ supporter
 * ^^^^^^^^^^^^^^^^^ supported
 * ^^^^^^^^^^^^^^^^^^^^^^ supportee
 *         ^^^^^^^^^^^^^^^^^^^^^^^^ overhang extended
 *         ^^^^^^^^^      overhang extensions
 *         ^^^^^^^^^^^^^^ overhang
 */
std::pair<Polygons, Polygons> AreaSupport::computeBasicAndFullOverhang(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const unsigned int layer_idx, const int64_t max_dist_from_lower_layer)
{
    Polygons supportLayer_supportee = mesh.layers[layer_idx].getOutlines();
    Polygons supportLayer_supporter = storage.getLayerOutlines(layer_idx-1, false);

    Polygons supportLayer_supported =  supportLayer_supporter.offset(max_dist_from_lower_layer);
    Polygons basic_overhang = supportLayer_supportee.difference(supportLayer_supported);

    const SupportLayer& support_layer = storage.support.supportLayers[layer_idx];
    if (support_layer.anti_overhang.size())
    {
        basic_overhang = basic_overhang.difference(support_layer.anti_overhang);
    }

//     Polygons support_extension = basic_overhang.offset(max_dist_from_lower_layer);
//     support_extension = support_extension.intersection(supportLayer_supported);
//     support_extension = support_extension.intersection(supportLayer_supportee);
//     
//     Polygons overhang =  basic_overhang.unionPolygons(support_extension);
//         presumably the computation above is slower than the one below

    Polygons overhang_extented = basic_overhang.offset(max_dist_from_lower_layer + 100); // +100 for easier joining with support from layer above
    Polygons full_overhang = overhang_extented.intersection(supportLayer_supportee);
    return std::make_pair(basic_overhang, full_overhang);
}


void AreaSupport::detectOverhangPoints(
    const SliceDataStorage& storage,
    const SliceMeshStorage& mesh,
    std::vector<std::vector<Polygons>>& overhang_points, // stores overhang_points of each layer
    int layer_count,
    int supportMinAreaSqrt
)
{
    ExtruderTrain* infill_extr = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_infill_extruder_nr"));
    const unsigned int support_line_width = infill_extr->getSettingInMicrons("support_line_width");

    overhang_points.resize(layer_count);

    for (int layer_idx = 1; layer_idx < layer_count; layer_idx++)
    {
        const SliceLayer& layer = mesh.layers[layer_idx];
        for (const SliceLayerPart& part : layer.parts)
        {
            if (part.outline.outerPolygon().area() < supportMinAreaSqrt * supportMinAreaSqrt) 
            {
                const SliceLayer& layer_below = mesh.layers[layer_idx - 1];
                if (layer_below.getOutlines().intersection(part.outline).size() > 0)
                {
                    continue;
                }

                Polygons part_poly_computed;
                const Polygons& part_poly = (part.insets.size() > 0) ? part.insets[0] : part_poly_computed; // don't copy inset if its already computed
                if (part.insets.size() == 0)
                {
                    part_poly_computed = part.outline.offset(-support_line_width / 2);
                }

                if (part_poly.size() > 0)
                {
                    Polygons part_poly_recomputed = part_poly.difference(storage.support.supportLayers[layer_idx].anti_overhang);
                    if (part_poly_recomputed.size() == 0)
                    {
                        continue;
                    }
                    std::vector<Polygons>& layer_overhang_points = overhang_points[layer_idx];
                    layer_overhang_points.push_back(part_poly_recomputed);
                }
            }
        }
    }
}



void AreaSupport::handleTowers(
    Polygons& supportLayer_this,
    std::vector<Polygons>& towerRoofs,
    std::vector<std::vector<Polygons>>& overhang_points,
    int layer_idx,
    int towerRoofExpansionDistance,
    int supportTowerDiameter,
    int supportMinAreaSqrt,
    int layer_count,
    int z_layer_distance_tower
)
{
    int layer_overhang_point =  layer_idx + z_layer_distance_tower;
    if (layer_overhang_point >= layer_count - 1)
    {
        return;
    }
    std::vector<Polygons>& overhang_points_here = overhang_points[layer_overhang_point]; // may be changed if an overhang point has a (smaller) overhang point directly below
    // handle new tower roof tops
    if (overhang_points_here.size() > 0)
    {
        { // make sure we have the lowest point (make polys empty if they have small parts below)
            if (layer_overhang_point < layer_count && overhang_points[layer_overhang_point - 1].size() > 0)
            {
                std::vector<Polygons>& overhang_points_below = overhang_points[layer_overhang_point - 1];
                for (Polygons& poly_here : overhang_points_here)
                {
                    for (const Polygons& poly_below : overhang_points_below)
                    {
                        poly_here = poly_here.difference(poly_below.offset(supportMinAreaSqrt*2));
                    }
                }
            }
        }
        for (Polygons& poly : overhang_points_here)
        {
            if (poly.size() > 0)
            {
                towerRoofs.push_back(poly);
            }
        }
    }
    
    // make tower roofs
    for (unsigned int roof_idx = 0; roof_idx < towerRoofs.size(); roof_idx++)
    {
        Polygons& tower_roof = towerRoofs[roof_idx];
        if (tower_roof.size() > 0)
        {
            supportLayer_this = supportLayer_this.unionPolygons(tower_roof);

            if (tower_roof[0].area() < supportTowerDiameter * supportTowerDiameter)
            {
                tower_roof = tower_roof.offset(towerRoofExpansionDistance);
            }
            else
            {
                tower_roof.clear();
            }
        }
    }
}

void AreaSupport::handleWallStruts(
    Polygons& supportLayer_this,
    int supportMinAreaSqrt,
    int supportTowerDiameter
    )
{
    for (unsigned int p = 0; p < supportLayer_this.size(); p++)
    {
        PolygonRef poly = supportLayer_this[p];
        if (poly.size() < 6) // might be a single wall
        {
            PolygonRef poly = supportLayer_this[p];
            int best = -1;
            int best_length2 = -1;
            for (unsigned int i = 0; i < poly.size(); i++)
            {
                int length2 = vSize2(poly[i] - poly[(i+1) % poly.size()]);
                if (length2 > best_length2)
                {
                    best = i;
                    best_length2 = length2;
                }
            }
            
            if (best_length2 < supportMinAreaSqrt * supportMinAreaSqrt)
                break; // this is a small area, not a wall!
                
            
            // an estimate of the width of the area
            int width = sqrt( poly.area() * poly.area() / best_length2 ); // sqrt (a^2 / l^2) instead of a / sqrt(l^2)
            
            // add square tower (strut) in the middle of the wall
            if (width < supportMinAreaSqrt)
            {
                Point mid = (poly[best] + poly[(best+1) % poly.size()] ) / 2;
                Polygons struts;
                PolygonRef strut = struts.newPoly();
                strut.add(mid + Point( supportTowerDiameter/2,  supportTowerDiameter/2));
                strut.add(mid + Point(-supportTowerDiameter/2,  supportTowerDiameter/2));
                strut.add(mid + Point(-supportTowerDiameter/2, -supportTowerDiameter/2));
                strut.add(mid + Point( supportTowerDiameter/2, -supportTowerDiameter/2));
                supportLayer_this = supportLayer_this.unionPolygons(struts);
            }
        }
    }
}

void AreaSupport::generateSupportBottom(SliceDataStorage& storage, const SliceMeshStorage& mesh)
{
    const unsigned int bottom_layer_count = round_divide(mesh.getSettingInMicrons("support_bottom_height"), storage.getSettingInMicrons("layer_height")); //Number of layers in support bottom.
    if (bottom_layer_count <= 0)
    {
        return;
    }
    const unsigned int z_distance_bottom = round_up_divide(mesh.getSettingInMicrons("support_bottom_distance"), storage.getSettingInMicrons("layer_height")); //Number of layers between support bottom and model.
    const unsigned int skip_layer_count = std::max(1u, round_divide(mesh.getSettingInMicrons("support_interface_skip_height"), storage.getSettingInMicrons("layer_height"))); //Resolution of generating support bottoms above model.
    const coord_t bottom_line_width = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_bottom_extruder_nr"))->getSettingInMicrons("support_bottom_line_width");

    const unsigned int scan_count = std::max(1u, (bottom_layer_count - 1) / skip_layer_count); //How many measurements to take to generate bottom areas.
    const float z_skip = std::max(1.0f, float(bottom_layer_count - 1) / float(scan_count)); //How many layers to skip between measurements. Using float for better spread, but this is later rounded.

    std::vector<SupportLayer>& support_layers = storage.support.supportLayers;
    for (unsigned int layer_idx = z_distance_bottom; layer_idx < support_layers.size(); layer_idx++)
    {
        const unsigned int bottom_layer_idx_below = std::max(0, int(layer_idx) - int(bottom_layer_count) - int(z_distance_bottom));
        Polygons mesh_outlines;
        for (float layer_idx_below = bottom_layer_idx_below; std::round(layer_idx_below) < (int)(layer_idx - z_distance_bottom); layer_idx_below += z_skip)
        {
            mesh_outlines.add(mesh.layers[std::round(layer_idx_below)].getOutlines());
        }
        Polygons bottoms;
        generateSupportInterfaceLayer(support_layers[layer_idx].supportAreas, mesh_outlines, bottom_line_width, bottoms);
        support_layers[layer_idx].support_bottom.add(bottoms);
    }
}

void AreaSupport::generateSupportRoof(SliceDataStorage& storage, const SliceMeshStorage& mesh)
{
    const unsigned int roof_layer_count = round_divide(mesh.getSettingInMicrons("support_roof_height"), storage.getSettingInMicrons("layer_height")); //Number of layers in support roof.
    if (roof_layer_count <= 0)
    {
        return;
    }
    const unsigned int z_distance_top = round_up_divide(mesh.getSettingInMicrons("support_top_distance"), storage.getSettingInMicrons("layer_height")); //Number of layers between support roof and model.
    const unsigned int skip_layer_count = std::max(1u, round_divide(mesh.getSettingInMicrons("support_interface_skip_height"), storage.getSettingInMicrons("layer_height"))); //Resolution of generating support roof below model.
    const coord_t roof_line_width = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_roof_extruder_nr"))->getSettingInMicrons("support_roof_line_width");

    const unsigned int scan_count = std::max(1u, (roof_layer_count - 1) / skip_layer_count); //How many measurements to take to generate roof areas.
    const float z_skip = std::max(1.0f, float(roof_layer_count - 1) / float(scan_count)); //How many layers to skip between measurements. Using float for better spread, but this is later rounded.

    std::vector<SupportLayer>& support_layers = storage.support.supportLayers;
    for (unsigned int layer_idx = 0; static_cast<int>(layer_idx) < static_cast<int>(support_layers.size() - z_distance_top); layer_idx++)
    {
        const unsigned int top_layer_idx_above = std::min(static_cast<unsigned int>(support_layers.size() - 1), layer_idx + roof_layer_count + z_distance_top); //Maximum layer of the model that generates support roof.
        Polygons mesh_outlines;
        for (float layer_idx_above = top_layer_idx_above; layer_idx_above > layer_idx + z_distance_top; layer_idx_above -= z_skip)
        {
            mesh_outlines.add(mesh.layers[std::round(layer_idx_above)].getOutlines());
        }
        Polygons roofs;
        generateSupportInterfaceLayer(support_layers[layer_idx].supportAreas, mesh_outlines, roof_line_width, roofs);
        support_layers[layer_idx].support_roof.add(roofs);
    }
}

void AreaSupport::generateSupportInterfaceLayer(Polygons& support_areas, const Polygons colliding_mesh_outlines, const coord_t safety_offset, Polygons& interface_polygons)
{
    Polygons model = colliding_mesh_outlines.unionPolygons();
    interface_polygons = support_areas.intersection(model);
    interface_polygons = interface_polygons.offset(safety_offset).intersection(support_areas); //Make sure we don't generate any models that are not printable.
    interface_polygons.removeSmallAreas(1.0);
    support_areas = support_areas.difference(interface_polygons);
}

}//namespace cura

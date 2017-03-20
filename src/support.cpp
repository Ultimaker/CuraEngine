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
    
    
Polygons AreaSupport::join(Polygons& supportLayer_up, Polygons& supportLayer_this, int64_t supportJoinDistance, int64_t smoothing_distance, int max_smoothing_angle, bool conical_support, int64_t conical_support_offset, int64_t conical_smallest_breadth)
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
        support_layer.support_mesh = support_layer.support_mesh.unionPolygons();
    }

    // initialization of supportAreasPerLayer
    if (layer_count > storage.support.supportLayers.size())
    { // there might alsready be anti_overhang_area data in the supportLayers
        storage.support.supportLayers.resize(layer_count);
    }

    // generate support areas
    for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        SliceMeshStorage& mesh = storage.meshes[mesh_idx];
        if (mesh.getSettingBoolean("infill_mesh") || mesh.getSettingBoolean("anti_overhang_mesh"))
        {
            continue;
        }
        std::vector<Polygons> supportAreas;
        supportAreas.resize(layer_count, Polygons());
        generateSupportAreas(storage, mesh_idx, layer_count, supportAreas);

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

        if (mesh.getSettingBoolean("support_interface_enable"))
        {
            generateSupportRoof(storage, mesh);
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
void AreaSupport::generateSupportAreas(SliceDataStorage& storage, unsigned int mesh_idx, unsigned int layer_count, std::vector<Polygons>& supportAreas)
{
    SliceMeshStorage& mesh = storage.meshes[mesh_idx];
        
    // given settings
    ESupportType support_type = storage.getSettingAsSupportType("support_type");
    
    if (!mesh.getSettingBoolean("support_enable"))
        return;
    if (support_type == ESupportType::NONE)
        return;
    
    const double supportAngle = mesh.getSettingInAngleRadians("support_angle");
    const bool supportOnBuildplateOnly = support_type == ESupportType::PLATFORM_ONLY;
    const int supportZDistanceBottom = mesh.getSettingInMicrons("support_bottom_distance");
    const int supportZDistanceTop = mesh.getSettingInMicrons("support_top_distance");
    const int join_distance = mesh.getSettingInMicrons("support_join_distance");
    const int support_bottom_stair_step_height = mesh.getSettingInMicrons("support_bottom_stair_step_height");

    const int extension_offset = mesh.getSettingInMicrons("support_offset");

    const int supportTowerDiameter = mesh.getSettingInMicrons("support_tower_diameter");
    const int supportMinAreaSqrt = mesh.getSettingInMicrons("support_minimal_diameter");
    const double supportTowerRoofAngle = mesh.getSettingInAngleRadians("support_tower_roof_angle");
    const bool use_towers = mesh.getSettingBoolean("support_use_towers") && supportMinAreaSqrt > 0;

    const int layerThickness = storage.getSettingInMicrons("layer_height");
    const int supportXYDistance = mesh.getSettingInMicrons("support_xy_distance");
    const int support_xy_distance_overhang = mesh.getSettingInMicrons("support_xy_distance_overhang");

    const bool use_support_xy_distance_overhang = mesh.getSettingAsSupportDistPriority("support_xy_overrides_z") == SupportDistPriority::Z_OVERRIDES_XY; // whether to use a different xy distance at overhangs

    const double conical_support_angle = mesh.getSettingInAngleRadians("support_conical_angle");
    const bool conical_support = mesh.getSettingBoolean("support_conical_enabled") && conical_support_angle != 0;
    const int64_t conical_smallest_breadth = mesh.getSettingInMicrons("support_conical_min_width");

    int support_infill_extruder_nr = storage.getSettingAsIndex("support_infill_extruder_nr");
    bool interface_enable = mesh.getSettingBoolean("support_interface_enable");

    // derived settings:
    const int max_smoothing_angle = 135; // maximum angle of inner corners to be smoothed
    coord_t smoothing_distance;
    { // compute best smoothing_distance
        ExtruderTrain& infill_train = *storage.meshgroup->getExtruderTrain(support_infill_extruder_nr);
        const coord_t support_infill_line_width = infill_train.getSettingInMicrons("support_infill_line_width");
        smoothing_distance = support_infill_line_width;
        if (interface_enable)
        {
            const int support_roof_extruder_nr = storage.getSettingAsIndex("support_roof_extruder_nr");
            const ExtruderTrain& roof_train = *storage.meshgroup->getExtruderTrain(support_roof_extruder_nr);
            const coord_t support_roof_line_width = roof_train.getSettingInMicrons("support_roof_line_width");
            
            const int support_bottom_extruder_nr = storage.getSettingAsIndex("support_bottom_extruder_nr");
            const ExtruderTrain& bottom_train = *storage.meshgroup->getExtruderTrain(support_bottom_extruder_nr);
            const coord_t support_bottom_line_width = bottom_train.getSettingInMicrons("support_bottom_line_width");
            
            smoothing_distance = std::max({smoothing_distance, support_roof_line_width, support_bottom_line_width});
        }
    }

    const int z_layer_distance_tower = 1; // start tower directly below overhang point
    
    
    int supportLayerThickness = layerThickness;
    
    const unsigned int layerZdistanceTop = std::max(0U, round_up_divide(supportZDistanceTop, supportLayerThickness)) + 1; // support must always be 1 layer below overhang
    const unsigned int layerZdistanceBottom = std::max(0U, round_up_divide(supportZDistanceBottom, supportLayerThickness));

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
        
    
    std::vector<std::pair<int, std::vector<Polygons>>> overhang_points; // stores overhang_points along with the layer index at which the overhang point occurs
    if (use_towers)
    {
        AreaSupport::detectOverhangPoints(storage, mesh, overhang_points, layer_count, supportMinAreaSqrt);
    }

    std::vector<Polygons> xy_disallowed_per_layer;
    std::vector<Polygons> full_overhang_per_layer;
    xy_disallowed_per_layer.resize(support_layer_count);
    full_overhang_per_layer.resize(support_layer_count);
#pragma omp parallel for default(none) shared(xy_disallowed_per_layer, full_overhang_per_layer, support_layer_count, storage, mesh, max_dist_from_lower_layer, tanAngle) schedule(dynamic)
    for (unsigned int layer_idx = 0; layer_idx < support_layer_count; layer_idx++)
    {
        std::pair<Polygons, Polygons> basic_and_full_overhang = computeBasicAndFullOverhang(storage, mesh, layer_idx, max_dist_from_lower_layer);
        full_overhang_per_layer[layer_idx] = basic_and_full_overhang.second;

        Polygons basic_overhang = basic_and_full_overhang.first;
        Polygons outlines = storage.getLayerOutlines(layer_idx, false);
        if (use_support_xy_distance_overhang)
        {
            Polygons xy_overhang_disallowed = basic_overhang.offset(supportZDistanceTop * tanAngle);
            Polygons xy_non_overhang_disallowed = outlines.difference(basic_overhang.offset(supportXYDistance)).offset(supportXYDistance);

            xy_disallowed_per_layer[layer_idx] = xy_overhang_disallowed.unionPolygons(xy_non_overhang_disallowed.unionPolygons(outlines.offset(support_xy_distance_overhang)));
        }
        else
        {
            xy_disallowed_per_layer[layer_idx] = outlines.offset(supportXYDistance);
        }
    }

    int overhang_points_pos = overhang_points.size() - 1;
    Polygons supportLayer_last;
    std::vector<Polygons> towerRoofs;

    for (unsigned int layer_idx = support_layer_count - 1 - layerZdistanceTop; layer_idx != (unsigned int) -1 ; layer_idx--)
    {
        Polygons supportLayer_this = full_overhang_per_layer[layer_idx + layerZdistanceTop];;

        if (extension_offset)
        {
            supportLayer_this = supportLayer_this.offset(extension_offset);
        }

        if (use_towers)
        {
            // handle straight walls
            AreaSupport::handleWallStruts(supportLayer_this, supportMinAreaSqrt, supportTowerDiameter);
            // handle towers
            AreaSupport::handleTowers(supportLayer_this, towerRoofs, overhang_points, overhang_points_pos, layer_idx, towerRoofExpansionDistance, supportTowerDiameter, supportMinAreaSqrt, layer_count, z_layer_distance_tower);
        }
    
        if (layer_idx+1 < support_layer_count)
        { // join with support from layer up                
            supportLayer_this = AreaSupport::join(supportLayer_last, supportLayer_this, join_distance, smoothing_distance, max_smoothing_angle, conical_support, conical_support_offset, conical_smallest_breadth);
        }

        if (storage.support.supportLayers[layer_idx].support_mesh.size() > 0)
        { // handle support mesh
            supportLayer_this = supportLayer_this.unionPolygons(storage.support.supportLayers[layer_idx].support_mesh);
        }

        // move up from model
        if (layerZdistanceBottom > 0 && layer_idx >= layerZdistanceBottom)
        {
            int stepHeight = support_bottom_stair_step_height / supportLayerThickness + 1;
            int bottomLayer = ((layer_idx - layerZdistanceBottom) / stepHeight) * stepHeight;
            supportLayer_this = supportLayer_this.difference(storage.getLayerOutlines(bottomLayer, false));
        }


        supportLayer_last = supportLayer_this;
        
        
        // inset using X/Y distance
        if (supportLayer_this.size() > 0)
        {
            supportLayer_this = supportLayer_this.difference(xy_disallowed_per_layer[layer_idx]);
        }

        supportAreas[layer_idx] = supportLayer_this;

        Progress::messageProgress(Progress::Stage::SUPPORT, storage.meshes.size() * mesh_idx + support_layer_count - layer_idx, support_layer_count * storage.meshes.size());
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
    Polygons full_overhang = overhang_extented.intersection(supportLayer_supported.unionPolygons(supportLayer_supportee));
    return std::make_pair(basic_overhang, full_overhang);
}


void AreaSupport::detectOverhangPoints(
    SliceDataStorage& storage,
    SliceMeshStorage& mesh, 
    std::vector<std::pair<int, std::vector<Polygons>>>& overhang_points, // stores overhang_points along with the layer index at which the overhang point occurs)
    int layer_count,
    int supportMinAreaSqrt
)
{
    ExtruderTrain* infill_extr = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_infill_extruder_nr"));
    const unsigned int support_line_width = infill_extr->getSettingInMicrons("support_line_width");
    for (int layer_idx = 0; layer_idx < layer_count; layer_idx++)
    {
        SliceLayer& layer = mesh.layers[layer_idx];
        for (SliceLayerPart& part : layer.parts)
        {
            if (part.outline.outerPolygon().area() < supportMinAreaSqrt * supportMinAreaSqrt) 
            {
                Polygons part_poly_computed;
                Polygons& part_poly = (part.insets.size() > 0) ? part.insets[0] : part_poly_computed; // don't copy inset if its already computed
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
                    if (overhang_points.size() > 0 && overhang_points.back().first == layer_idx)
                        overhang_points.back().second.push_back(part_poly_recomputed);
                    else 
                    {
                        std::vector<Polygons> small_part_polys;
                        small_part_polys.push_back(part_poly_recomputed);
                        overhang_points.emplace_back<std::pair<int, std::vector<Polygons>>>(std::make_pair(layer_idx, small_part_polys));
                    }
                }
                
            }
        }
    }
}



void AreaSupport::handleTowers(
    Polygons& supportLayer_this,
    std::vector<Polygons>& towerRoofs,
    std::vector<std::pair<int, std::vector<Polygons>>>& overhang_points,
    int& overhang_points_pos,
    int layer_idx,
    int towerRoofExpansionDistance,
    int supportTowerDiameter,
    int supportMinAreaSqrt,
    int layer_count,
    int z_layer_distance_tower
)
{
    // handle new tower roof tops
    int layer_overhang_point =  layer_idx + z_layer_distance_tower;
    if (overhang_points_pos >= 0 && layer_overhang_point < layer_count && 
        overhang_points[overhang_points_pos].first == layer_overhang_point) 
    {
        std::vector<Polygons>& overhang_points_here = overhang_points[overhang_points_pos].second;
        { // make sure we have the lowest point (make polys empty if they have small parts below)
            if (overhang_points_pos > 0 && overhang_points[overhang_points_pos - 1].first == layer_overhang_point - 1)
            {
                std::vector<Polygons>& overhang_points_below = overhang_points[overhang_points_pos - 1].second;
                for (Polygons& poly_here : overhang_points_here)
                {
                    for (Polygons& poly_below : overhang_points_below)
                    {
                        poly_here = poly_here.difference(poly_below.offset(supportMinAreaSqrt*2));
                    }
                }
            }
        }
        for (Polygons& poly : overhang_points_here)
            if (poly.size() > 0)
                towerRoofs.push_back(poly);
        overhang_points_pos--;
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
    const unsigned int skip_layer_count = std::max(1u, round_divide(mesh.getSettingInMicrons("support_bottom_skip_height"), storage.getSettingInMicrons("layer_height"))); //Resolution of generating support bottoms above model.
    const coord_t bottom_line_width = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_bottom_extruder_nr"))->getSettingInMicrons("support_bottom_line_width");

    const unsigned int scan_count = std::max(1u, (bottom_layer_count - 1) / skip_layer_count); //How many measurements to take to generate bottom areas.
    const float z_skip = std::max(1.0f, float(bottom_layer_count - 1) / float(scan_count)); //How many layers to skip between measurements. Using float for better spread, but this is later rounded.

    std::vector<SupportLayer>& support_layers = storage.support.supportLayers;
    for (unsigned int layer_idx = z_distance_bottom; layer_idx < support_layers.size(); layer_idx++)
    {
        SupportLayer& layer = support_layers[layer_idx];
        const unsigned int bottom_layer_idx_below = std::max(0, int(layer_idx) - int(bottom_layer_count) - int(z_distance_bottom));
        //Find out what parts of the model are below the support, so where support bottom must be placed.
        Polygons model;
        for (float layer_idx_below = bottom_layer_idx_below; std::round(layer_idx_below) < (int)(layer_idx - z_distance_bottom); layer_idx_below += z_skip)
        {
            const Polygons outlines_below = mesh.layers[std::round(layer_idx_below)].getOutlines();
            model = model.unionPolygons(outlines_below);
        }
        Polygons bottoms = layer.supportAreas.intersection(model);
        bottoms = bottoms.offset(bottom_line_width).intersection(layer.supportAreas); //Expand support bottom a bit so that we're sure it's not too thin to be printed.
        bottoms.removeSmallAreas(1.0);
        layer.support_bottom.add(bottoms);
        layer.supportAreas = layer.supportAreas.difference(layer.support_bottom);
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
    const unsigned int skip_layer_count = std::max(1u, round_divide(mesh.getSettingInMicrons("support_roof_skip_height"), storage.getSettingInMicrons("layer_height"))); //Resolution of generating support roof below model.
    const coord_t roof_line_width = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_roof_extruder_nr"))->getSettingInMicrons("support_roof_line_width");

    const unsigned int scan_count = std::max(1u, (roof_layer_count - 1) / skip_layer_count); //How many measurements to take to generate roof areas.
    const float z_skip = std::max(1.0f, float(roof_layer_count - 1) / float(scan_count)); //How many layers to skip between measurements. Using float for better spread, but this is later rounded.

    std::vector<SupportLayer>& support_layers = storage.support.supportLayers;
    for (unsigned int layer_idx = 0; layer_idx < support_layers.size() - roof_layer_count - z_distance_top; layer_idx++)
    {
        SupportLayer& layer = support_layers[layer_idx];
        const unsigned int top_layer_idx_above = layer_idx + roof_layer_count + z_distance_top; //Maximum layer of the model that generates support roof.
        //Find out what parts of the model are above the support, so where support roof must be placed.
        Polygons model;
        for (float layer_idx_above = top_layer_idx_above; layer_idx_above > layer_idx + z_distance_top; layer_idx_above -= z_skip)
        {
            const Polygons outlines_above = mesh.layers[std::round(layer_idx_above)].getOutlines();
            model = model.unionPolygons(outlines_above);
        }
        Polygons roofs = layer.supportAreas.intersection(model);
        roofs = roofs.offset(roof_line_width).intersection(layer.supportAreas); //Expand support roof a bit so that we're sure it's not too thin to be printed.
        roofs.removeSmallAreas(1.0);
        layer.support_roof.add(roofs);
        layer.supportAreas = layer.supportAreas.difference(layer.support_roof);
    }
}



}//namespace cura

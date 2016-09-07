/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <cmath> // sqrt
#include <utility> // pair
#include <deque>
#include <cmath> // round

#include "support.h"

#include "utils/math.h"
#include "progress/Progress.h"

namespace cura 
{
    
    
Polygons AreaSupport::join(Polygons& supportLayer_up, Polygons& supportLayer_this, int64_t supportJoinDistance, int64_t smoothing_distance, int min_smoothing_area, bool conical_support, int64_t conical_support_offset, int64_t conical_smallest_breadth)
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
    if (smoothing_distance > 0)
        joined = joined.smooth(smoothing_distance, min_smoothing_area);
    
    return joined;
}

void AreaSupport::generateSupportAreas(SliceDataStorage& storage, unsigned int layer_count)
{
    // initialization of supportAreasPerLayer
    for (unsigned int layer_idx = 0; layer_idx < layer_count ; layer_idx++)
        storage.support.supportLayers.emplace_back();
    
    for(unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        SliceMeshStorage& mesh = storage.meshes[mesh_idx];
        if (mesh.getSettingBoolean("infill_mesh"))
        {
            continue;
        }
        std::vector<Polygons> supportAreas;
        supportAreas.resize(layer_count, Polygons());
        generateSupportAreas(storage, mesh_idx, layer_count, supportAreas);
        
        if (mesh.getSettingBoolean("support_interface_enable"))
        {
            generateSupportInterface(storage, mesh, supportAreas, layer_count);
        }
        else 
        {
            for (unsigned int layer_idx = 0; layer_idx < layer_count ; layer_idx++)
            {
                storage.support.supportLayers[layer_idx].supportAreas.add(supportAreas[layer_idx]);
            }
        }
    }
    
    for (unsigned int layer_idx = 0; layer_idx < layer_count ; layer_idx++)
    {
        storage.support.supportLayers[layer_idx].supportAreas = storage.support.supportLayers[layer_idx].supportAreas.unionPolygons();

        // get a support line width representative for all support
        int support_skin_extruder_nr = storage.getSettingAsIndex("support_interface_extruder_nr");
        int support_infill_extruder_nr = (layer_idx == 0)? storage.getSettingAsIndex("support_extruder_nr_layer_0") : storage.getSettingAsIndex("support_infill_extruder_nr");
        int interface_enable = storage.getSettingBoolean("support_interface_enable");
        int interface_extruder_nr = interface_enable? support_skin_extruder_nr : support_infill_extruder_nr;
        ExtruderTrain& train = *storage.meshgroup->getExtruderTrain(interface_extruder_nr);
        int support_line_width = train.getSettingInMicrons(interface_enable? "support_interface_line_width" : "support_line_width");

        // remove jagged line pieces introduced by unioning separate overhang areas for consectuive layers
        //
        // support may otherwise look like:
        //      _____________________      .
        //     /                     \      } dist_from_lower_layer
        //    /__                   __\    /
        //      /                   \        `\.
        //     /                     \         } dist_from_lower_layer
        //    /__                   __\      ./
        //      /                   \     `\.
        //     /                     \      } dist_from_lower_layer
        //    /_______________________\   ,/
        //            rather than
        //     _______________________
        //    |                       |
        //    |                       |
        //    |                       |
        //    |                       |
        //    |                       |
        //    |                       |
        //    |                       |
        //    |_______________________|
        //
        // dist_from_lower_layer may be up to max_dist_from_lower_layer (see below), but that value may be extremely high
        storage.support.supportLayers[layer_idx].supportAreas.simplify(support_line_width / 2);
        
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
    const int smoothing_distance = mesh.getSettingInMicrons("support_area_smoothing"); 

    const int extension_offset = mesh.getSettingInMicrons("support_offset");

    const int supportTowerDiameter = mesh.getSettingInMicrons("support_tower_diameter");
    const int supportMinAreaSqrt = mesh.getSettingInMicrons("support_minimal_diameter");
    const double supportTowerRoofAngle = mesh.getSettingInAngleRadians("support_tower_roof_angle");

    const int min_smoothing_area = 100 * 100; // minimal area for which to perform smoothing
    const int z_layer_distance_tower = 1; // start tower directly below overhang point

    const int layerThickness = storage.getSettingInMicrons("layer_height");
    const int supportXYDistance = mesh.getSettingInMicrons("support_xy_distance");
    const int support_xy_distance_overhang = mesh.getSettingInMicrons("support_xy_distance_overhang");

    const bool use_support_xy_distance_overhang = mesh.getSettingAsSupportDistPriority("support_xy_overrides_z") == SupportDistPriority::Z_OVERRIDES_XY; // whether to use a different xy distance at overhangs

    const double conical_support_angle = mesh.getSettingInAngleRadians("support_conical_angle");
    const bool conical_support = mesh.getSettingBoolean("support_conical_enabled") && conical_support_angle != 0;
    const int64_t conical_smallest_breadth = mesh.getSettingInMicrons("support_conical_min_width");
    
    // derived settings:
    
    
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
    AreaSupport::detectOverhangPoints(storage, mesh, overhang_points, layer_count, supportMinAreaSqrt);

    std::deque<std::pair<Polygons, Polygons>> basic_and_full_overhang_above;
    for (unsigned int layer_idx = support_layer_count - 1; layer_idx != support_layer_count - 1 - layerZdistanceTop ; layer_idx--)
    {
        basic_and_full_overhang_above.push_front(computeBasicAndFullOverhang(storage, mesh, layer_idx, max_dist_from_lower_layer));
    }

    bool still_in_upper_empty_layers = true;
    int overhang_points_pos = overhang_points.size() - 1;
    Polygons supportLayer_last;
    std::vector<Polygons> towerRoofs;

    for (unsigned int layer_idx = support_layer_count - 1 - layerZdistanceTop; layer_idx != (unsigned int) -1 ; layer_idx--)
    {
        basic_and_full_overhang_above.push_front(computeBasicAndFullOverhang(storage, mesh, layer_idx, max_dist_from_lower_layer));
        
        Polygons overhang;
        {
            // compute basic overhang and put in right layer ([layerZdistanceTOp] layers below)
            overhang = basic_and_full_overhang_above.back().second;
            basic_and_full_overhang_above.pop_back();
        }

        Polygons& supportLayer_this = overhang; 

        if (extension_offset)
        {
            supportLayer_this = supportLayer_this.offset(extension_offset);
        }

        if (supportMinAreaSqrt > 0)
        {
            // handle straight walls
            AreaSupport::handleWallStruts(supportLayer_this, supportMinAreaSqrt, supportTowerDiameter);
            // handle towers
            AreaSupport::handleTowers(supportLayer_this, towerRoofs, overhang_points, overhang_points_pos, layer_idx, towerRoofExpansionDistance, supportTowerDiameter, supportMinAreaSqrt, layer_count, z_layer_distance_tower);
        }
    
        if (layer_idx+1 < support_layer_count)
        { // join with support from layer up                
            supportLayer_this = AreaSupport::join(supportLayer_last, supportLayer_this, join_distance, smoothing_distance, min_smoothing_area, conical_support, conical_support_offset, conical_smallest_breadth);
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
            Polygons& basic_overhang = basic_and_full_overhang_above.front().first; // basic overhang on this layer
            Polygons outlines = storage.getLayerOutlines(layer_idx, false);

            if (use_support_xy_distance_overhang)
            {
                Polygons xy_overhang_disallowed = basic_overhang.offset(supportZDistanceTop * tanAngle);
                Polygons xy_non_overhang_disallowed = outlines.difference(basic_overhang.offset(supportXYDistance)).offset(supportXYDistance);

                Polygons xy_disallowed = xy_overhang_disallowed.unionPolygons(xy_non_overhang_disallowed.unionPolygons(outlines.offset(support_xy_distance_overhang)));
                supportLayer_this = supportLayer_this.difference(xy_disallowed);
            }
            else
            {
                supportLayer_this = supportLayer_this.difference(storage.getLayerOutlines(layer_idx, false).offset(supportXYDistance));
            }
        }

        supportAreas[layer_idx] = supportLayer_this;

        if (still_in_upper_empty_layers && supportLayer_this.size() > 0)
        {
            storage.support.layer_nr_max_filled_layer = std::max(storage.support.layer_nr_max_filled_layer, (int)layer_idx);
            still_in_upper_empty_layers = false;
        }
        
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
                    if (overhang_points.size() > 0 && overhang_points.back().first == layer_idx)
                        overhang_points.back().second.push_back(part_poly);
                    else 
                    {
                        std::vector<Polygons> small_part_polys;
                        small_part_polys.push_back(part_poly);
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
    //for (Polygons& tower_roof : towerRoofs)
    for (unsigned int r = 0; r < towerRoofs.size(); r++)
    {
        supportLayer_this = supportLayer_this.unionPolygons(towerRoofs[r]);
        
        Polygons& tower_roof = towerRoofs[r];
        if (tower_roof.size() > 0 && tower_roof[0].area() < supportTowerDiameter * supportTowerDiameter)
        {
            towerRoofs[r] = tower_roof.offset(towerRoofExpansionDistance);
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


void AreaSupport::generateSupportInterface(SliceDataStorage& storage, const SliceMeshStorage& mesh, std::vector<Polygons>& support_areas, const unsigned int layer_count)
{
    const unsigned int roof_layer_count = round_divide(mesh.getSettingInMicrons("support_roof_height"), storage.getSettingInMicrons("layer_height"));
    const unsigned int bottom_layer_count = round_divide(mesh.getSettingInMicrons("support_bottom_height"), storage.getSettingInMicrons("layer_height"));
    const unsigned int z_distance_bottom = round_up_divide(mesh.getSettingInMicrons("support_bottom_distance"), storage.getSettingInMicrons("layer_height"));
    const unsigned int z_distance_top = round_up_divide(mesh.getSettingInMicrons("support_top_distance"), storage.getSettingInMicrons("layer_height"));

    const int skip_layer_count = std::max(1u, round_divide(mesh.getSettingInMicrons("support_interface_skip_height"), storage.getSettingInMicrons("layer_height")));
    const int interface_line_width = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_interface_extruder_nr"))->getSettingInMicrons("support_interface_line_width");

    std::vector<SupportLayer>& supportLayers = storage.support.supportLayers;
    for (unsigned int layer_idx = 0; layer_idx < layer_count; layer_idx++)
    {
        SupportLayer& layer = supportLayers[layer_idx];

        const unsigned int top_layer_idx_above = layer_idx + roof_layer_count + z_distance_top;
        const unsigned int bottom_layer_idx_below = std::max(0, int(layer_idx) - int(bottom_layer_count) - int(z_distance_bottom));
        if (top_layer_idx_above < supportLayers.size())
        {
            Polygons roofs;
            if (roof_layer_count > 0)
            {
                Polygons model;
                unsigned int n_scans = std::max(1u, (roof_layer_count - 1) / skip_layer_count);
                float z_skip = float(roof_layer_count - 1) / float(n_scans);
                for (float layer_idx_above = top_layer_idx_above; layer_idx_above > layer_idx + z_distance_top; layer_idx_above -= z_skip)
                {
                    const Polygons outlines_above = mesh.layers[std::round(layer_idx_above)].getOutlines();
                    model = model.unionPolygons(outlines_above);
                }
                roofs = support_areas[layer_idx].intersection(model);
            }
            Polygons bottoms;
            if (bottom_layer_count > 0)
            {
                Polygons model;
                unsigned int n_scans = std::max(1u, (bottom_layer_count - 1) / skip_layer_count);
                float z_skip = float(bottom_layer_count - 1) / float(n_scans);
                for (float layer_idx_below = bottom_layer_idx_below; std::round(layer_idx_below) < (int)(layer_idx - z_distance_bottom); layer_idx_below += z_skip)
                {
                    const Polygons outlines_below = mesh.layers[std::round(layer_idx_below)].getOutlines();
                    model = model.unionPolygons(outlines_below);
                }
                bottoms = support_areas[layer_idx].intersection(model);
            }
            // expand skin a bit so that we're sure it's not too thin to be printed.
            Polygons skin = roofs.unionPolygons(bottoms).offset(interface_line_width).intersection(support_areas[layer_idx]);
            skin.removeSmallAreas(1.0);
            layer.skin.add(skin);
            layer.supportAreas.add(support_areas[layer_idx].difference(layer.skin));
        }
        else 
        {
            layer.skin.add(layer.supportAreas);
        }
    }
}



}//namespace cura

/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "support.h"

#include <cmath> // sqrt
#include <utility> // pair
#include "Progress.h"

namespace cura 
{

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
void generateSupportAreas(SliceDataStorage& storage, SliceMeshStorage* object, unsigned int layer_count, CommandSocket* commandSocket)
{
    // given settings
    ESupportType support_type = object->settings->getSettingAsSupportType("support_type");
    
    storage.support.generated = false;
    if (!object->settings->getSettingBoolean("support_enable"))
        return;
    if (support_type == Support_None)
        return;
    
    double supportAngle = object->settings->getSettingInAngleRadians("support_angle");
    bool supportOnBuildplateOnly = support_type == Support_PlatformOnly;
    int supportXYDistance = object->settings->getSettingInMicrons("support_xy_distance");
    int supportZDistance = object->settings->getSettingInMicrons("support_z_distance");
    int supportZDistanceBottom = object->settings->getSettingInMicrons("support_bottom_distance");
    int supportZDistanceTop = object->settings->getSettingInMicrons("support_top_distance");
    int supportJoinDistance = object->settings->getSettingInMicrons("support_join_distance");
    int support_bottom_stair_step_height = object->settings->getSettingInMicrons("support_bottom_stair_step_height");
    int smoothing_distance = object->settings->getSettingInMicrons("support_area_smoothing"); 
    
    int supportTowerDiameter = object->settings->getSettingInMicrons("support_tower_diameter");
    int supportMinAreaSqrt = object->settings->getSettingInMicrons("support_minimal_diameter");
    double supportTowerRoofAngle = object->settings->getSettingInAngleRadians("support_tower_roof_angle");
    
    //std::cerr <<" towerDiameter=" << towerDiameter <<", supportMinAreaSqrt=" << supportMinAreaSqrt << std::endl;
    
    int min_smoothing_area = 100*100; // minimal area for which to perform smoothing
    int z_layer_distance_tower = 1; // start tower directly below overhang point
        
    int layerThickness = object->settings->getSettingInMicrons("layer_height");
    int extrusionWidth = object->settings->getSettingInMicrons("support_line_width"); 
    

    
    // derived settings:
    
    if (supportZDistanceBottom < 0) supportZDistanceBottom = supportZDistance;
    if (supportZDistanceTop < 0)    supportZDistanceTop = supportZDistance;
    
    
    int supportLayerThickness = layerThickness;
    
    int layerZdistanceTop       = supportZDistanceTop / supportLayerThickness + 1; // support must always be 1 layer below overhang
    unsigned int layerZdistanceBottom    = std::max(0, supportZDistanceBottom / supportLayerThickness); 

    double tanAngle = tan(supportAngle) - 0.01;  // the XY-component of the supportAngle
    int maxDistFromLowerLayer = tanAngle * supportLayerThickness; // max dist which can be bridged
    
    unsigned int support_layer_count = layer_count;
    
    double tanTowerRoofAngle = tan(supportTowerRoofAngle);
    int towerRoofExpansionDistance = layerThickness / tanTowerRoofAngle;
    
    
    // early out
    
    if ( layerZdistanceTop + 1 > (int) support_layer_count )
    {
        storage.support.generated = false; // no (first layer) support can be generated 
        return;
    }
    
    
    // computation
        
    
    std::vector<Polygons> joinedLayers; // join model layers of all meshes into polygons and store small areas which need tower support
    std::vector<std::pair<int, std::vector<Polygons>>> overhang_points; // stores overhang_points along with the layer index at which the overhang point occurs
    AreaSupport::joinMeshesAndDetectOverhangPoints(storage, joinedLayers, overhang_points, layer_count, supportMinAreaSqrt, extrusionWidth);
        
    
    // initialization of supportAreasPerLayer
    for (unsigned int layer_idx = 0; layer_idx < layer_count ; layer_idx++)
        storage.support.supportLayers.emplace_back();

    bool still_in_upper_empty_layers = true;
    int overhang_points_pos = overhang_points.size() - 1;
    Polygons supportLayer_last;
    std::vector<Polygons> towerRoofs;
    for (unsigned int layer_idx = support_layer_count - 1 - layerZdistanceTop; layer_idx != (unsigned int) -1 ; layer_idx--)
    {
        
        
        // compute basic overhang and put in right layer ([layerZdistanceTOp] layers below)
        Polygons& supportLayer_supportee =  joinedLayers[layer_idx+layerZdistanceTop];
        Polygons& supportLayer_supporter =  joinedLayers[layer_idx-1+layerZdistanceTop];
        Polygons supportLayer_supported =  supportLayer_supporter.offset(maxDistFromLowerLayer);
        Polygons basic_overhang = supportLayer_supportee.difference(supportLayer_supported);
     
//         Polygons support_extension = basic_overhang.offset(maxDistFromLowerLayer);
//         support_extension = support_extension.intersection(supportLayer_supported);
//         support_extension = support_extension.intersection(supportLayer_supportee);
//         
//         Polygons overhang =  basic_overhang.unionPolygons(support_extension);
//         presumably the computation above is slower than the one below
        
        Polygons overhang_extented = basic_overhang.offset(maxDistFromLowerLayer + 100); // +100 for easier joining with support from layer above
        Polygons overhang = overhang_extented.intersection(supportLayer_supported.unionPolygons(supportLayer_supportee));
        
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

        
        Polygons& supportLayer_this = overhang; 
        
        supportLayer_this = supportLayer_this.simplify(50); // TODO: hardcoded value!
        
        if (supportMinAreaSqrt > 0)
        {
            // handle straight walls
            AreaSupport::handleWallStruts(supportLayer_this, supportMinAreaSqrt, supportTowerDiameter);
            // handle towers
            AreaSupport::handleTowers(supportLayer_this, towerRoofs, overhang_points, overhang_points_pos, layer_idx, towerRoofExpansionDistance, supportTowerDiameter, supportMinAreaSqrt, layer_count, z_layer_distance_tower);
        }
        
        
        if (layer_idx+1 < support_layer_count)
        { // join with support from layer up
            Polygons& supportLayer_up = supportLayer_last;
            
            Polygons joined = supportLayer_this.unionPolygons(supportLayer_up);
            // join different parts
            if (supportJoinDistance > 0)
            {
                joined = joined.offset(supportJoinDistance); 
                joined = joined.offset(-supportJoinDistance);
            }
            if (smoothing_distance > 0)
                joined = joined.smooth(smoothing_distance, min_smoothing_area);
        
            // remove layer parts
//             Polygons insetted = joined.difference(joinedLayers[layer_idx]);
//             supportLayer_this = insetted;                
            supportLayer_this = joined;                
            
        }
        
        // move up from model
        if (layerZdistanceBottom > 0 && layer_idx >= layerZdistanceBottom)
        {
            int stepHeight = support_bottom_stair_step_height / supportLayerThickness + 1;
            int bottomLayer = ((layer_idx - layerZdistanceBottom) / stepHeight) * stepHeight;
            supportLayer_this = supportLayer_this.difference(joinedLayers[bottomLayer]);
        }
        
        
        supportLayer_last = supportLayer_this;
        
        
        // inset using X/Y distance
        if (supportLayer_this.size() > 0)
            supportLayer_this = supportLayer_this.difference(joinedLayers[layer_idx].offset(supportXYDistance));
        
        storage.support.supportLayers[layer_idx].supportAreas = supportLayer_this;
        
        if (still_in_upper_empty_layers && supportLayer_this.size() > 0)
        {
            storage.support.layer_nr_max_filled_layer = layer_idx;
            still_in_upper_empty_layers = false;
        }
        
        Progress::messageProgress(Progress::Stage::SUPPORT, support_layer_count - layer_idx, support_layer_count, commandSocket);
    }
    
    // do stuff for when support on buildplate only
    if (supportOnBuildplateOnly)
    {
        Polygons touching_buildplate = storage.support.supportLayers[0].supportAreas;
        for (unsigned int layer_idx = 1 ; layer_idx < storage.support.supportLayers.size() ; layer_idx++)
        {
            Polygons& supportLayer = storage.support.supportLayers[layer_idx].supportAreas;
            
            touching_buildplate = supportLayer.intersection(touching_buildplate); // from bottom to top, support areas can only decrease!
            
            storage.support.supportLayers[layer_idx].supportAreas = touching_buildplate;
        }
    }

    storage.support.generated = true;
}

void AreaSupport::joinMeshesAndDetectOverhangPoints(
    SliceDataStorage& storage,
    std::vector<Polygons>& joinedLayers,
    std::vector<std::pair<int, std::vector<Polygons>>>& overhang_points, // stores overhang_points along with the layer index at which the overhang point occurs)
    int layer_count,
    int supportMinAreaSqrt,
    int extrusionWidth
                  )
{
    for (int layer_idx = 0 ; layer_idx < layer_count ; layer_idx++)
    {
        joinedLayers.emplace_back();
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            SliceLayer& layer = mesh.layers[layer_idx];
            for (SliceLayerPart& part : layer.parts)
            {
                
                if (part.outline.outerPolygon().area() < supportMinAreaSqrt * supportMinAreaSqrt) 
                {
                    Polygons part_poly = part.outline.offset(-extrusionWidth/2);
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
                joinedLayers.back() = joinedLayers.back().unionPolygons(part.outline);
                
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


void generateSupportRoofs(SliceDataStorage& storage, CommandSocket* commandSocket, int layerThickness, int support_roof_height)
{
    int roof_layer_count = support_roof_height / layerThickness;
    
    std::vector<SupportLayer>& supportLayers = storage.support.supportLayers;
    for (unsigned int layer_idx = 0; layer_idx < supportLayers.size(); layer_idx++)
    {
        SupportLayer& layer = supportLayers[layer_idx];
        
        Polygons non_roof;
        if (layer_idx + roof_layer_count < supportLayers.size())
        {
            non_roof = supportLayers[layer_idx + roof_layer_count].supportAreas;
        }
        
        layer.roofs = layer.supportAreas.difference(non_roof);
        layer.roofs.removeSmallAreas(1.0);
        layer.supportAreas = layer.supportAreas.difference(layer.roofs);
        
    }
}



}//namespace cura

/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "support.h"

#include <cmath> // sqrt
#include <utility> // pair

namespace cura {

void AreaSupport::joinMeshesAndDetectOverhangPoints(
    SliceDataStorage& storage,
    std::vector<Polygons>& joinedLayers,
    std::vector<std::pair<int, std::vector<Polygons>>>& overhang_points, // stores overhang_points along with the layer index at which the overhang point occurs)
    int layer_count,
    int supportMinAreaSqrt,
    int extrusionWidth
                  )
{
    for (int l = 0 ; l < layer_count ; l++)
    {
        joinedLayers.emplace_back();
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            SliceLayer& layer = mesh.layers[l];
            for (SliceLayerPart& part : layer.parts)
            {
                
                if (part.outline[0].area() < supportMinAreaSqrt * supportMinAreaSqrt) 
                {
                    Polygons part_poly = part.outline.offset(-extrusionWidth/2);
                    if (part_poly.size() > 0)
                    {
                        if (overhang_points.size() > 0 && overhang_points.back().first == l)
                            overhang_points.back().second.push_back(part_poly);
                        else 
                        {
                            std::vector<Polygons> small_part_polys;
                            small_part_polys.push_back(part_poly);
                            overhang_points.emplace_back<std::pair<int, std::vector<Polygons>>>(std::make_pair(l, small_part_polys));
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
                    for (Polygons& poly_below : overhang_points_below)
                    {
                        poly_here = poly_here.difference(poly_below.offset(supportMinAreaSqrt*2));
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
    for (int r = 0; r < towerRoofs.size(); r++)
    {
        supportLayer_this = supportLayer_this.unionPolygons(towerRoofs[r]);
        
        Polygons& tower_roof = towerRoofs[r];
        if (tower_roof[0].area() < supportTowerDiameter * supportTowerDiameter)
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
    for (int p = 0; p < supportLayer_this.size(); p++)
    {
        PolygonRef poly = supportLayer_this[p];
        if (poly.size() < 6) // might be a single wall
        {
            PolygonRef poly = supportLayer_this[p];
            int best = -1;
            int best_length2 = -1;
            for (int i = 0; i < poly.size(); i++)
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

void generateSupportAreas(SliceDataStorage& storage, PrintObject* object, int layer_count)
{
    bool logStage = false; // whther to log at which stage of the support area generation we are (for debug)
    // given settings
    int supportAngle = object->getSettingInt("supportAngle");
    
    storage.support.generated = false;
    if (supportAngle < 0)
        return;
    
    bool supportOnBuildplateOnly = object->getSettingInt("supportOnBuildplateOnly") > 0;
    int supportXYDistance = object->getSettingInt("supportXYDistance");
    int supportZDistance = object->getSettingInt("supportZDistance");
    int supportZDistanceBottom = object->getSettingInt("supportZDistanceBottom");
    int supportZDistanceTop = object->getSettingInt("supportZDistanceTop");
    int supportJoinDistance = object->getSettingInt("supportJoinDistance");
    int supportBottomStairDistance = object->getSettingInt("supportBottomStairDistance");
    int smoothing_distance = object->getSettingInt("supportAreaSmoothing"); 
    
    int supportTowerDiameter = object->getSettingInt("supportTowerDiameter");
    int supportMinAreaSqrt = object->getSettingInt("supportMinimalAreaSqrt");
    int supportTowerRoofAngle = object->getSettingInt("supportTowerRoofAngle");
    
    //std::cerr <<" towerDiameter=" << towerDiameter <<", supportMinAreaSqrt=" << supportMinAreaSqrt << std::endl;
    
    int min_smoothing_area = 100*100;
    int z_layer_distance_tower = 1;
        
    int layerThickness = object->getSettingInt("layerThickness");
    int extrusionWidth = object->getSettingInt("extrusionWidth"); // TODO check for layer0extrusionWidth!
    
    

    
    // derived settings:
    
    if (supportZDistanceBottom < 0) supportZDistanceBottom = supportZDistance;
    if (supportZDistanceTop < 0)    supportZDistanceTop = supportZDistance;
    
    
    int supportLayerThickness = layerThickness;
    
    int layerZdistanceTop       = supportZDistanceTop / supportLayerThickness + 1; // support must always be 1 layer below overhang
    int layerZdistanceBottom    = supportZDistanceBottom / supportLayerThickness; 

    double tanAngle = tan(double(supportAngle) / 180.0 * M_PI) - 0.01;
    int maxDistFromLowerLayer = tanAngle * supportLayerThickness; // max dist which can be bridged
    
    int support_layer_count = layer_count;
    
    
    double tanTowerRoofAngle = tan(double(supportTowerRoofAngle) / 180.0 * M_PI);
    int towerRoofExpansionDistance = layerThickness / tanTowerRoofAngle;
    
    
    // computation
    
    if (logStage) log("joining model layers\n");
    
    
    std::vector<Polygons> joinedLayers; // join model layers of all meshes into polygons and store small areas which need tower support
    std::vector<std::pair<int, std::vector<Polygons>>> overhang_points; // stores overhang_points along with the layer index at which the overhang point occurs
    AreaSupport::joinMeshesAndDetectOverhangPoints(storage, joinedLayers, overhang_points, layer_count, supportMinAreaSqrt, extrusionWidth);
        
    
    // initialization of supportAreasPerLayer
    for (int l = 0; l < layer_count ; l++)
        storage.support.supportAreasPerLayer.emplace_back();


    if (logStage) log("computing support");
    
    //auto roundToNearestDivisibleBy = [](int in, int divisor) { return in - in % divisor; };
    int overhang_points_pos = overhang_points.size() - 1;
    Polygons supportLayer_last;
    std::vector<Polygons> towerRoofs;
    for (int layer_idx = support_layer_count - 1 - layerZdistanceTop; layer_idx >= 0 ; layer_idx--)
    {
        
        
        // compute basic overhang and put in right layer ([layerZdistanceTOp] layers below)
        Polygons supportLayer_supportee =  joinedLayers[layer_idx+layerZdistanceTop];
        Polygons supportLayer_supported =  joinedLayers[layer_idx-1+layerZdistanceTop].offset(maxDistFromLowerLayer);
        Polygons basic_overhang = supportLayer_supportee.difference(supportLayer_supported);
        
        Polygons support_extension = basic_overhang.offset(maxDistFromLowerLayer);
        support_extension = support_extension.intersection(supportLayer_supported);
        support_extension = support_extension.intersection(supportLayer_supportee);
        
        Polygons overhang =  basic_overhang.unionPolygons(support_extension);
        
        /* supported
         * .................
         *         ______________|
         * _______|         ^^^^^ basic overhang
         * 
         *         ^^^^^^^^^      overhang extensions
         *         ^^^^^^^^^^^^^^ overhang
         */

        
        Polygons& supportLayer_this = overhang; 
        
        supportLayer_this = supportLayer_this.simplify(2500);
        
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
        
            // remove layer
            Polygons insetted = joined.difference(joinedLayers[layer_idx]);
            supportLayer_this = insetted;                
            
        }
        
        
        supportLayer_last = supportLayer_this;
        
        // inset using X/Y distance
        if (supportLayer_this.size() > 0)
            supportLayer_this = supportLayer_this.difference(joinedLayers[layer_idx].offset(supportXYDistance));
        
        // move up from model
        if (layerZdistanceBottom > 0 && layer_idx >= layerZdistanceBottom)
        {
            int stepHeight = supportBottomStairDistance / supportLayerThickness + 1;
            int bottomLayer = ((layer_idx - layerZdistanceBottom) / stepHeight) * stepHeight;
            supportLayer_this = supportLayer_this.difference(joinedLayers[bottomLayer]);
        }
        
        storage.support.supportAreasPerLayer[layer_idx] = supportLayer_this;
        
    }
    
    // do stuff for when support on buildplate only
    if (supportOnBuildplateOnly)
    {
        if (logStage) log("supporting on buildplate only");
        Polygons touching_buildplate = storage.support.supportAreasPerLayer[0];
        for (int l = 1 ; l < storage.support.supportAreasPerLayer.size() ; l++)
        {
            Polygons& supportLayer = storage.support.supportAreasPerLayer[l];
            
            touching_buildplate = supportLayer.intersection(touching_buildplate); // from bottom to top, support areas can only decrease!
            
            storage.support.supportAreasPerLayer[l] = touching_buildplate;
        }
    }

    
    joinedLayers.clear();
    if (logStage) log("finished area support");
    
    storage.support.generated = true;
}




}//namespace cura

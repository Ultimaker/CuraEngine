/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "skin.h"
#include "utils/polygonUtils.h"

#define MIN_AREA_SIZE (0.4 * 0.4) 

namespace cura 
{

        
void generateSkins(int layerNr, SliceMeshStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount, int innermost_wall_extrusion_width, int insetCount, bool no_small_gaps_heuristic, bool avoidOverlappingPerimeters_0, bool avoidOverlappingPerimeters)
{
    generateSkinAreas(layerNr, storage, innermost_wall_extrusion_width, downSkinCount, upSkinCount, no_small_gaps_heuristic);

    SliceLayer* layer = &storage.layers[layerNr];
    for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
    {
        SliceLayerPart* part = &layer->parts[partNr];
        generateSkinInsets(part, extrusionWidth, insetCount, avoidOverlappingPerimeters_0, avoidOverlappingPerimeters);
    }
}

void generateSkinAreas(int layer_nr, SliceMeshStorage& storage, int innermost_wall_extrusion_width, int downSkinCount, int upSkinCount, bool no_small_gaps_heuristic)
{
    SliceLayer& layer = storage.layers[layer_nr];
    
    if (downSkinCount == 0 && upSkinCount == 0)
    {
        return;
    }
    
    for(unsigned int partNr = 0; partNr < layer.parts.size(); partNr++)
    {
        SliceLayerPart& part = layer.parts[partNr];
        
        Polygons upskin = part.insets.back().offset(-innermost_wall_extrusion_width/2);
        Polygons downskin = (downSkinCount == 0)? Polygons() : upskin;
        if (upSkinCount == 0) upskin = Polygons();

        auto getInsidePolygons = [&part](SliceLayer& layer2)
            {
                Polygons result;
                for(SliceLayerPart& part2 : layer2.parts)
                {
                    if (part.boundaryBox.hit(part2.boundaryBox))
                        result.add(part2.insets.back());
                }
                return result;
            };
            
        if (no_small_gaps_heuristic)
        {
            if (static_cast<int>(layer_nr - downSkinCount) >= 0)
            {
                downskin = downskin.difference(getInsidePolygons(storage.layers[layer_nr - downSkinCount])); // skin overlaps with the walls
            }
            
            if (static_cast<int>(layer_nr + upSkinCount) < static_cast<int>(storage.layers.size()))
            {
                upskin = upskin.difference(getInsidePolygons(storage.layers[layer_nr + upSkinCount])); // skin overlaps with the walls
            }
        }
        else 
        {
            if (layer_nr > 0 && downSkinCount > 0)
            {
                Polygons not_air = getInsidePolygons(storage.layers[layer_nr - 1]);
                for (int downskin_layer_nr = std::max(0, layer_nr - downSkinCount); downskin_layer_nr < layer_nr - 1; downskin_layer_nr++)
                {
                    not_air = not_air.intersection(getInsidePolygons(storage.layers[downskin_layer_nr]));
                }
                downskin = downskin.difference(not_air); // skin overlaps with the walls
            }
            
            if (layer_nr < static_cast<int>(storage.layers.size()) - 1 && upSkinCount > 0)
            {
                Polygons not_air = getInsidePolygons(storage.layers[layer_nr + 1]);
                for (int upskin_layer_nr = layer_nr + 2; upskin_layer_nr < std::min(static_cast<int>(storage.layers.size()) - 1, layer_nr + upSkinCount); upskin_layer_nr++)
                {
                    not_air = not_air.intersection(getInsidePolygons(storage.layers[upskin_layer_nr]));
                }
                upskin = upskin.difference(not_air); // skin overlaps with the walls
            }
        }
        
        Polygons skin = upskin.unionPolygons(downskin);
          
        skin.removeSmallAreas(MIN_AREA_SIZE);
        
        for (PolygonsPart& skin_area_part : skin.splitIntoParts())
        {
            part.skin_parts.emplace_back();
            part.skin_parts.back().outline = skin_area_part;
        }
    }
}


void generateSkinInsets(SliceLayerPart* part, int extrusionWidth, int insetCount, bool avoidOverlappingPerimeters_0, bool avoidOverlappingPerimeters)
{
    if (insetCount == 0)
    {
        return;
    }
    
    for (SkinPart& skin_part : part->skin_parts)
    {
        for(int i=0; i<insetCount; i++)
        {
            skin_part.insets.push_back(Polygons());
            if (i == 0)
            {
                PolygonUtils::offsetSafe(skin_part.outline, - extrusionWidth/2, extrusionWidth, skin_part.insets[0], avoidOverlappingPerimeters_0);
                Polygons in_between = skin_part.outline.difference(skin_part.insets[0].offset(extrusionWidth/2)); 
                skin_part.perimeterGaps.add(in_between);
            } else
            {
                PolygonUtils::offsetExtrusionWidth(skin_part.insets[i-1], true, extrusionWidth, skin_part.insets[i], &skin_part.perimeterGaps, avoidOverlappingPerimeters);
            }
            
            // optimize polygons: remove unnnecesary verts
            skin_part.insets[i].simplify();
            if (skin_part.insets[i].size() < 1)
            {
                skin_part.insets.pop_back();
                break;
            }
        }
    }
}

void generateInfill(int layerNr, SliceMeshStorage& storage, int extrusionWidth, int infill_skin_overlap)
{
    SliceLayer& layer = storage.layers[layerNr];

    for(SliceLayerPart& part : layer.parts)
    {
        Polygons infill = part.insets.back().offset(-extrusionWidth / 2 - infill_skin_overlap);

        for(SliceLayerPart& part2 : layer.parts)
        {
            if (part.boundaryBox.hit(part2.boundaryBox))
            {
                for(SkinPart& skin_part : part2.skin_parts)
                {
                    infill = infill.difference(skin_part.outline);
                }
            }
        }
        infill.removeSmallAreas(MIN_AREA_SIZE);
        
        part.infill_area.push_back(infill.offset(infill_skin_overlap));
    }
}

void combineInfillLayers(int layerNr, SliceMeshStorage& storage, int amount)
{
    SliceLayer* layer = &storage.layers[layerNr];

    for(int n=1; n<amount; n++)
    {
        if (layerNr < n)
            break;
        
        SliceLayer* layer2 = &storage.layers[layerNr - n];
        for(SliceLayerPart& part : layer->parts)
        {
            Polygons result;
            for(SliceLayerPart& part2 : layer2->parts)
            {
                if (part.boundaryBox.hit(part2.boundaryBox))
                {
                    Polygons intersection = part.infill_area[n - 1].intersection(part2.infill_area[0]).offset(-200).offset(200);
                    result.add(intersection);
                    part.infill_area[n - 1] = part.infill_area[n - 1].difference(intersection);
                    part2.infill_area[0] = part2.infill_area[0].difference(intersection);
                }
            }
            
            part.infill_area.push_back(result);
        }
    }
}


void generatePerimeterGaps(int layer_nr, SliceMeshStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount)
{
    SliceLayer& layer = storage.layers[layer_nr];
    
    for (SliceLayerPart& part : layer.parts) 
    { // handle gaps between perimeters etc.
        if (downSkinCount > 0 && upSkinCount > 0 && // note: if both are zero or less, then all gaps will be used
            layer_nr >= downSkinCount && layer_nr < static_cast<int>(storage.layers.size() - upSkinCount)) // remove gaps which appear within print, i.e. not on the bottom most or top most skin
        {
            Polygons outlines_above;
            for (SliceLayerPart& part_above : storage.layers[layer_nr + upSkinCount].parts)
            {
                if (part.boundaryBox.hit(part_above.boundaryBox))
                {
                    outlines_above.add(part_above.outline);
                }
            }
            Polygons outlines_below;
            for (SliceLayerPart& part_below : storage.layers[layer_nr - downSkinCount].parts)
            {
                if (part.boundaryBox.hit(part_below.boundaryBox))
                {
                    outlines_below.add(part_below.outline);
                }
            }
            part.perimeterGaps = part.perimeterGaps.intersection(outlines_above.xorPolygons(outlines_below));
        }
        part.perimeterGaps.removeSmallAreas(MIN_AREA_SIZE);
    }
}

}//namespace cura

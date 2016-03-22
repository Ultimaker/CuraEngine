/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "skin.h"
#include "utils/polygonUtils.h"

#define MIN_AREA_SIZE (0.4 * 0.4) 

namespace cura 
{

        
void generateSkins(int layerNr, SliceMeshStorage& storage, int extrusionWidth, int downSkinCount, int upSkinCount, int wall_line_count, int innermost_wall_extrusion_width, int insetCount, bool no_small_gaps_heuristic, bool avoidOverlappingPerimeters_0, bool avoidOverlappingPerimeters)
{
    generateSkinAreas(layerNr, storage, innermost_wall_extrusion_width, downSkinCount, upSkinCount, wall_line_count, no_small_gaps_heuristic);

    SliceLayer* layer = &storage.layers[layerNr];
    for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
    {
        SliceLayerPart* part = &layer->parts[partNr];
        generateSkinInsets(part, extrusionWidth, insetCount, avoidOverlappingPerimeters_0, avoidOverlappingPerimeters);
    }
}

void generateSkinAreas(int layer_nr, SliceMeshStorage& storage, int innermost_wall_extrusion_width, int downSkinCount, int upSkinCount, int wall_line_count, bool no_small_gaps_heuristic)
{
    SliceLayer& layer = storage.layers[layer_nr];
    
    if (downSkinCount == 0 && upSkinCount == 0)
    {
        return;
    }
    
    for(unsigned int partNr = 0; partNr < layer.parts.size(); partNr++)
    {
        SliceLayerPart& part = layer.parts[partNr];

        if (int(part.insets.size()) < wall_line_count)
        {
            continue; // the last wall is not present, the part should only get inter preimeter gaps, but no skin.
        }

        Polygons upskin = part.insets.back().offset(-innermost_wall_extrusion_width/2);
        Polygons downskin = (downSkinCount == 0)? Polygons() : upskin;
        if (upSkinCount == 0) upskin = Polygons();

        auto getInsidePolygons = [&part, wall_line_count](SliceLayer& layer2)
            {
                Polygons result;
                for(SliceLayerPart& part2 : layer2.parts)
                {
                    if (part.boundaryBox.hit(part2.boundaryBox))
                        result.add(part2.insets[wall_line_count - 1]);
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
            if (layer_nr >= downSkinCount && downSkinCount > 0)
            {
                Polygons not_air = getInsidePolygons(storage.layers[layer_nr - 1]);
                for (int downskin_layer_nr = layer_nr - downSkinCount; downskin_layer_nr < layer_nr - 1; downskin_layer_nr++)
                {
                    not_air = not_air.intersection(getInsidePolygons(storage.layers[downskin_layer_nr]));
                }
                downskin = downskin.difference(not_air); // skin overlaps with the walls
            }
            
            if (layer_nr < static_cast<int>(storage.layers.size()) - downSkinCount && upSkinCount > 0)
            {
                Polygons not_air = getInsidePolygons(storage.layers[layer_nr + 1]);
                for (int upskin_layer_nr = layer_nr + 2; upskin_layer_nr < layer_nr + upSkinCount + 1; upskin_layer_nr++)
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

void generateInfill(int layerNr, SliceMeshStorage& storage, int innermost_wall_extrusion_width, int infill_skin_overlap, int wall_line_count)
{
    SliceLayer& layer = storage.layers[layerNr];

    for(SliceLayerPart& part : layer.parts)
    {
        if (int(part.insets.size()) < wall_line_count)
        {
            part.infill_area.emplace_back(); // put empty polygon as (uncombined) infill
            continue; // the last wall is not present, the part should only get inter preimeter gaps, but no infill.
        }
        Polygons infill = part.insets.back().offset(-innermost_wall_extrusion_width / 2 - infill_skin_overlap);

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

void combineInfillLayers(SliceMeshStorage& storage,unsigned int amount)
{
    if(amount <= 1) //If we must combine 1 layer, nothing needs to be combined. Combining 0 layers is invalid.
    {
        return;
    }
    if(storage.layers.empty() || storage.layers.size() - 1 < static_cast<size_t>(storage.getSettingAsCount("top_layers")) || storage.getSettingAsCount("infill_line_distance") <= 0) //No infill is even generated.
    {
        return;
    }
    /* We need to round down the layer index we start at to the nearest
    divisible index. Otherwise we get some parts that have infill at divisible
    layers and some at non-divisible layers. Those layers would then miss each
    other. */
    size_t min_layer = storage.getSettingAsCount("bottom_layers") + amount - 1;
    min_layer -= min_layer % amount; //Round upwards to the nearest layer divisible by infill_sparse_combine.
    size_t max_layer = storage.layers.size() - 1 - storage.getSettingAsCount("top_layers");
    max_layer -= max_layer % amount; //Round downwards to the nearest layer divisible by infill_sparse_combine.
    for(size_t layer_idx = min_layer;layer_idx <= max_layer;layer_idx += amount) //Skip every few layers, but extrude more.
    {
        SliceLayer* layer = &storage.layers[layer_idx];

        for(unsigned int n = 1;n < amount;n++)
        {
            if(layer_idx < n)
            {
                break;
            }

            SliceLayer* layer2 = &storage.layers[layer_idx - n];
            for(SliceLayerPart& part : layer->parts)
            {
                Polygons result;
                for(SliceLayerPart& part2 : layer2->parts)
                {
                    if(part.boundaryBox.hit(part2.boundaryBox))
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

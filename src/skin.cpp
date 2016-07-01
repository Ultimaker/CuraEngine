/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include "skin.h"
#include "utils/polygonUtils.h"

#define MIN_AREA_SIZE (0.4 * 0.4) 

namespace cura 
{

        
void generateSkins(int layerNr, SliceMeshStorage& mesh, int extrusionWidth, int downSkinCount, int upSkinCount, int wall_line_count, int innermost_wall_extrusion_width, int insetCount, bool no_small_gaps_heuristic)
{
    generateSkinAreas(layerNr, mesh, innermost_wall_extrusion_width, downSkinCount, upSkinCount, wall_line_count, no_small_gaps_heuristic);

    SliceLayer* layer = &mesh.layers[layerNr];
    for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
    {
        SliceLayerPart* part = &layer->parts[partNr];
        generateSkinInsets(part, extrusionWidth, insetCount);
    }
}

void generateSkinAreas(int layer_nr, SliceMeshStorage& mesh, int innermost_wall_extrusion_width, int downSkinCount, int upSkinCount, int wall_line_count, bool no_small_gaps_heuristic)
{
    SliceLayer& layer = mesh.layers[layer_nr];
    
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
                    {
                        unsigned int wall_idx = std::max(0, std::min(wall_line_count, (int) part2.insets.size()) - 1);
                        result.add(part2.insets[wall_idx]);
                    }
                }
                return result;
            };
            
        if (no_small_gaps_heuristic)
        {
            if (static_cast<int>(layer_nr - downSkinCount) >= 0)
            {
                downskin = downskin.difference(getInsidePolygons(mesh.layers[layer_nr - downSkinCount])); // skin overlaps with the walls
            }
            
            if (static_cast<int>(layer_nr + upSkinCount) < static_cast<int>(mesh.layers.size()))
            {
                upskin = upskin.difference(getInsidePolygons(mesh.layers[layer_nr + upSkinCount])); // skin overlaps with the walls
            }
        }
        else 
        {
            if (layer_nr >= downSkinCount && downSkinCount > 0)
            {
                Polygons not_air = getInsidePolygons(mesh.layers[layer_nr - 1]);
                for (int downskin_layer_nr = layer_nr - downSkinCount; downskin_layer_nr < layer_nr - 1; downskin_layer_nr++)
                {
                    not_air = not_air.intersection(getInsidePolygons(mesh.layers[downskin_layer_nr]));
                }
                downskin = downskin.difference(not_air); // skin overlaps with the walls
            }
            
            if (layer_nr < static_cast<int>(mesh.layers.size()) - 1 && upSkinCount > 0)
            {
                Polygons not_air = getInsidePolygons(mesh.layers[layer_nr + 1]);
                for (int upskin_layer_nr = layer_nr + 2; upskin_layer_nr < layer_nr + upSkinCount + 1; upskin_layer_nr++)
                {
                    not_air = not_air.intersection(getInsidePolygons(mesh.layers[upskin_layer_nr]));
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


void generateSkinInsets(SliceLayerPart* part, int extrusionWidth, int insetCount)
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
                skin_part.insets[0] = skin_part.outline.offset(- extrusionWidth/2);
            } else
            {
                skin_part.insets[i] = skin_part.insets[i - 1].offset(-extrusionWidth);
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

void generateInfill(int layerNr, SliceMeshStorage& mesh, int innermost_wall_extrusion_width, int infill_skin_overlap, int wall_line_count)
{
    SliceLayer& layer = mesh.layers[layerNr];

    for(SliceLayerPart& part : layer.parts)
    {
        if (int(part.insets.size()) < wall_line_count)
        {
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
        
        part.infill_area = infill.offset(infill_skin_overlap);
    }
}

void SkinInfillAreaComputation::generateGradualInfill(SliceMeshStorage& mesh)
{
    int top_layers = mesh.getSettingAsCount("top_layers");
    unsigned int layer_skip_count = top_layers;
    unsigned int gradual_infill_step_height = 5000 / mesh.getSettingInMicrons("layer_height"); // The difference in layer count between consecutive density infill areas
    layer_skip_count = gradual_infill_step_height / ((gradual_infill_step_height - 1) / layer_skip_count + 1); // make gradual_infill_step_height divisable by layer_skip_count

    size_t min_layer = mesh.getSettingAsCount("bottom_layers");
    size_t max_layer = mesh.layers.size() - 1 - top_layers;

    for (size_t layer_idx = 0; layer_idx < mesh.layers.size(); layer_idx++)
    { // loop also over layers which don't contain infill cause of bottom_ and top_layer to initialize their infill_area_per_combine_per_density
        SliceLayer& layer = mesh.layers[layer_idx];

        for (SliceLayerPart& part : layer.parts)
        {
            assert(part.infill_area_per_combine_per_density.size() == 0 && "infill_area_per_combine_per_density is supposed to be uninitialized");

            if (part.infill_area.size() == 0 || layer_idx < min_layer || layer_idx > max_layer)
            { // initialize infill_area_per_combine_per_density empty
                part.infill_area_per_combine_per_density.emplace_back(); // create a new infill_area_per_combine
                part.infill_area_per_combine_per_density.back().emplace_back(); // put empty infill area in the newly contructed infill_area_per_combine
                continue;
            }

            Polygons infill_current_density = part.infill_area;
            for (unsigned int infill_step = 0; infill_current_density.size() != 0; infill_step++)
            {
                // add new infill_area_per_combine for the current density
                part.infill_area_per_combine_per_density.emplace_back();
                std::vector<Polygons>& infill_area_per_combine_current_density = part.infill_area_per_combine_per_density.back();
                infill_area_per_combine_current_density.push_back(infill_current_density);

                size_t min_layer = layer_idx + infill_step * gradual_infill_step_height + layer_skip_count;
                size_t max_layer = layer_idx + (infill_step + 1) * gradual_infill_step_height;

                for (size_t upper_layer_idx = min_layer; upper_layer_idx < max_layer; upper_layer_idx++)
                {
                    if (upper_layer_idx >= mesh.layers.size())
                    {
                        infill_current_density.clear();
                        break;
                    }
                    SliceLayer& upper_layer = mesh.layers[upper_layer_idx];
                    Polygons relevent_upper_polygons;
                    for (SliceLayerPart& upper_layer_part : upper_layer.parts)
                    {
                        if (!upper_layer_part.boundaryBox.hit(part.boundaryBox))
                        {
                            continue;
                        }
                        relevent_upper_polygons.add(upper_layer_part.infill_area);
                    }
                    infill_current_density = infill_current_density.intersection(relevent_upper_polygons);
                }
                mesh.max_gradual_infill_steps = std::max(mesh.max_gradual_infill_steps, infill_step);
            }
            assert(part.infill_area_per_combine_per_density.size() != 0 && "infill_area_per_combine_per_density is now initialized");
        }
    }
}

void combineInfillLayers(SliceMeshStorage& mesh, unsigned int amount)
{
    if (mesh.layers.empty() || mesh.layers.size() - 1 < static_cast<size_t>(mesh.getSettingAsCount("top_layers")) || mesh.getSettingAsCount("infill_line_distance") <= 0) //No infill is even generated.
    {
        return;
    }
    if(amount <= 1) //If we must combine 1 layer, nothing needs to be combined. Combining 0 layers is invalid.
    {
        return;
    }
    /* We need to round down the layer index we start at to the nearest
    divisible index. Otherwise we get some parts that have infill at divisible
    layers and some at non-divisible layers. Those layers would then miss each
    other. */
    size_t min_layer = mesh.getSettingAsCount("bottom_layers") + amount - 1;
    min_layer -= min_layer % amount; //Round upwards to the nearest layer divisible by infill_sparse_combine.
    size_t max_layer = mesh.layers.size() - 1 - mesh.getSettingAsCount("top_layers");
    max_layer -= max_layer % amount; //Round downwards to the nearest layer divisible by infill_sparse_combine.
    for(size_t layer_idx = min_layer;layer_idx <= max_layer;layer_idx += amount) //Skip every few layers, but extrude more.
    {
        SliceLayer* layer = &mesh.layers[layer_idx];

        for(unsigned int combine_count_here = 1; combine_count_here < amount; combine_count_here++)
        {
            if(layer_idx < combine_count_here)
            {
                break;
            }

            SliceLayer* lower_layer = &mesh.layers[layer_idx - combine_count_here];
            for (SliceLayerPart& upper_layer_part : layer->parts)
            {
                Polygons result;
                for (SliceLayerPart& lower_layer_part : lower_layer->parts)
                {
                    // TODO change combine algorithm so that it works gradual infill
                    if (upper_layer_part.boundaryBox.hit(lower_layer_part.boundaryBox))
                    {
                        Polygons intersection = upper_layer_part.infill_area_per_combine_per_density[0][combine_count_here - 1].intersection(lower_layer_part.infill_area_per_combine_per_density[0][0]).offset(-200).offset(200);
                        result.add(intersection);
                        upper_layer_part.infill_area_per_combine_per_density[0][combine_count_here - 1] = upper_layer_part.infill_area_per_combine_per_density[0][combine_count_here - 1].difference(intersection);
                        lower_layer_part.infill_area_per_combine_per_density[0][0] = lower_layer_part.infill_area_per_combine_per_density[0][0].difference(intersection);
                    }
                }

                upper_layer_part.infill_area_per_combine_per_density[0].push_back(result);
            }
        }
    }
}


}//namespace cura

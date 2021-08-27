//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "bridge.h"
#include "sliceDataStorage.h"
#include "settings/types/Ratio.h"
#include "utils/AABB.h"
#include "utils/polygon.h"

namespace cura
{

int bridgeAngle(const Settings& settings, const Polygons& skin_outline, const SliceDataStorage& storage, const unsigned layer_nr, const unsigned bridge_layer, const SupportLayer* support_layer, Polygons& supported_regions)
{
    assert(! skin_outline.empty());
    AABB boundary_box(skin_outline);

    //To detect if we have a bridge, first calculate the intersection of the current layer with the previous layer.
    // This gives us the islands that the layer rests on.
    Polygons islands;

    Polygons prev_layer_outline; // we also want the complete outline of the previous layer

    const Ratio sparse_infill_max_density = settings.get<Ratio>("bridge_sparse_infill_max_density");

    // include parts from all meshes
    for (const SliceMeshStorage& mesh : storage.meshes)
    {
        if (mesh.isPrinted())
        {
            const coord_t infill_line_distance = mesh.settings.get<coord_t>("infill_line_distance");
            const coord_t infill_line_width = mesh.settings.get<coord_t>("infill_line_width");
            const bool part_has_sparse_infill = (infill_line_distance == 0) || ((float)infill_line_width / infill_line_distance) <= sparse_infill_max_density;

            for (const SliceLayerPart& prev_layer_part : mesh.layers[layer_nr - bridge_layer].parts)
            {
                Polygons solid_below(prev_layer_part.outline);
                if (bridge_layer == 1 && part_has_sparse_infill)
                {
                    solid_below = solid_below.difference(prev_layer_part.getOwnInfillArea());
                }
                prev_layer_outline.add(solid_below); // not intersected with skin

                if (!boundary_box.hit(prev_layer_part.boundaryBox))
                    continue;

                islands.add(skin_outline.intersection(solid_below));
            }
        }
    }
    supported_regions = islands;

    if (support_layer)
    {
        // add the regions of the skin that have support below them to supportedRegions
        // but don't add these regions to islands because that can actually cause the code
        // below to consider the skin a bridge when it isn't (e.g. a skin that is supported by
        // the model on one side but the remainder of the skin is above support would look like
        // a bridge because it would have two islands) - FIXME more work required here?

        if (!support_layer->support_roof.empty())
        {
            AABB support_roof_bb(support_layer->support_roof);
            if (boundary_box.hit(support_roof_bb))
            {
                prev_layer_outline.add(support_layer->support_roof); // not intersected with skin

                Polygons supported_skin(skin_outline.intersection(support_layer->support_roof));
                if (!supported_skin.empty())
                {
                    supported_regions.add(supported_skin);
                }
            }
        }
        else
        {
            for (const SupportInfillPart& support_part : support_layer->support_infill_parts)
            {
                AABB support_part_bb(support_part.getInfillArea());
                if (boundary_box.hit(support_part_bb))
                {
                    prev_layer_outline.add(support_part.getInfillArea()); // not intersected with skin

                    Polygons supported_skin(skin_outline.intersection(support_part.getInfillArea()));
                    if (!supported_skin.empty())
                    {
                        supported_regions.add(supported_skin);
                    }
                }
            }
        }
    }

    const bool bridge_settings_enabled = settings.get<bool>("bridge_settings_enabled");
    const Ratio support_threshold = bridge_settings_enabled ? settings.get<Ratio>("bridge_skin_support_threshold") : 0.0_r;

    // if the proportion of the skin region that is supported is less than supportThreshold, it's considered a bridge and we
    // determine the best angle for the skin lines - the current heuristic is that the skin lines should be parallel to the
    // direction of the skin area's longest unsupported edge - if the skin has no unsupported edges, we fall through to the
    // original code

    if (support_threshold > 0 && (supported_regions.area() / (skin_outline.area() + 1)) < support_threshold)
    {
        Polygons bb_poly;
        bb_poly.add(boundary_box.toPolygon());

        // airBelow is the region below the skin that is not supported, it extends well past the boundary of the skin.
        // It needs to be shrunk slightly so that the vertices of the skin polygon that would otherwise fall exactly on
        // the air boundary do appear to be supported

        const int bb_max_dim = std::max(boundary_box.max.X - boundary_box.min.X, boundary_box.max.Y - boundary_box.min.Y);
        const Polygons air_below(bb_poly.offset(bb_max_dim).difference(prev_layer_outline).offset(-10));

        Polygons skin_perimeter_lines;
        for (ConstPolygonRef poly : skin_outline)
        {
            Point p0 = poly[0];
            for (unsigned i = 1; i < poly.size(); ++i)
            {
                Point p1 = poly[i];
                skin_perimeter_lines.addLine(p0, p1);
                p0 = p1;
            }
            skin_perimeter_lines.addLine(p0, poly[0]);
        }

        Polygons skin_perimeter_lines_over_air(air_below.intersectionPolyLines(skin_perimeter_lines));

        if (skin_perimeter_lines_over_air.size())
        {
            // one or more edges of the skin region are unsupported, determine the longest
            double max_dist2 = 0;
            double line_angle = -1;
            for (PolygonRef air_line : skin_perimeter_lines_over_air)
            {
                Point p0 = air_line[0];
                for (unsigned i = 1; i < air_line.size(); ++i)
                {
                    const Point& p1(air_line[i]);
                    double dist2 = vSize2(p0 - p1);
                    if (dist2 > max_dist2)
                    {
                        max_dist2 = dist2;
                        line_angle = angle(p0 - p1);
                    }
                    p0 = p1;
                }
            }
            return line_angle;
        }
    }
    else
    {
        // as the proportion of the skin region that is supported is >= supportThreshold, it's not
        // considered to be a bridge and the original bridge detection code below is skipped
        return -1;
    }

    if (islands.size() > 5 || islands.size() < 1)
    {
        return -1;
    }

    //Next find the 2 largest islands that we rest on.
    double area1 = 0;
    double area2 = 0;
    int idx1 = -1;
    int idx2 = -1;
    for(unsigned int n=0; n<islands.size(); n++)
    {
        //Skip internal holes
        if (!islands[n].orientation())
            continue;
        double area = fabs(islands[n].area());
        if (area > area1)
        {
            if (area1 > area2)
            {
                area2 = area1;
                idx2 = idx1;
            }
            area1 = area;
            idx1 = n;
        }
        else if (area > area2)
        {
            area2 = area;
            idx2 = n;
        }
    }
    
    if (idx1 < 0 || idx2 < 0)
        return -1;
    
    Point center1 = islands[idx1].centerOfMass();
    Point center2 = islands[idx2].centerOfMass();

    return angle(center2 - center1);
}

}//namespace cura


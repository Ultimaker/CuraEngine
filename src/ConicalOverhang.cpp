// Copyright (c) 2016 Tim Kuipers
// Copyright (c) 2022 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ConicalOverhang.h"

#include "geometry/Polygon.h"
#include "geometry/SingleShape.h"
#include "mesh.h"
#include "settings/types/Angle.h" //To process the overhang angle.
#include "settings/types/LayerIndex.h"
#include "slicer.h"
#include "utils/Simplify.h" //Simplifying at every step to prevent getting lots of vertices from all the insets.

namespace cura
{

void ConicalOverhang::apply(Slicer* slicer, const Mesh& mesh)
{
    const AngleRadians angle = mesh.settings_.get<AngleRadians>("conical_overhang_angle");
    const double max_hole_area = mesh.settings_.get<double>("conical_overhang_hole_size");
    const double tan_angle = tan(angle); // the XY-component of the angle
    const coord_t layer_thickness = mesh.settings_.get<coord_t>("layer_height");
    coord_t max_dist_from_lower_layer = std::llround(tan_angle * static_cast<double>(layer_thickness)); // max dist which can be bridged

    for (LayerIndex layer_nr = LayerIndex(slicer->layers.size()) - 2; layer_nr >= 0; layer_nr--)
    {
        SlicerLayer& layer = slicer->layers[static_cast<size_t>(layer_nr)];
        SlicerLayer& layer_above = slicer->layers[static_cast<size_t>(layer_nr) + 1ul];
        if (std::abs(max_dist_from_lower_layer) < 5)
        { // magically nothing happens when max_dist_from_lower_layer == 0
            // below magic code solves that
            constexpr coord_t safe_dist = 20;
            Shape diff = layer_above.polygons_.difference(layer.polygons_.offset(-safe_dist));
            layer.polygons_ = layer.polygons_.unionPolygons(diff);
            layer.polygons_ = layer.polygons_.smooth(safe_dist);
            layer.polygons_ = Simplify(safe_dist, safe_dist / 2, 0).polygon(layer.polygons_);
            // somehow layer.polygons get really jagged lines with a lot of vertices
            // without the above steps slicing goes really slow
        }
        else
        {
            // Get the current layer and split it into parts
            std::vector<SingleShape> layer_parts = layer.polygons_.splitIntoParts();
            // Get a copy of the layer above to prune away before we shrink it
            Shape above = layer_above.polygons_;

            // Now go through all the holes in the current layer and check if they intersect anything in the layer above
            // If not, then they're the top of a hole and should be cut from the layer above before the union
            for (unsigned int part = 0; part < layer_parts.size(); part++)
            {
                if (layer_parts[part].size() > 1) // first poly is the outer contour, 1..n are the holes
                {
                    for (unsigned int hole_nr = 1; hole_nr < layer_parts[part].size(); ++hole_nr)
                    {
                        Shape hole_poly;
                        hole_poly.push_back(layer_parts[part][hole_nr]);
                        if (max_hole_area > 0.0 && INT2MM2(std::abs(hole_poly.area())) < max_hole_area)
                        {
                            Shape hole_with_above = hole_poly.intersection(above);
                            if (! hole_with_above.empty())
                            {
                                // The hole had some intersection with the above layer, check if it's a complete overlap
                                Shape hole_difference = hole_poly.xorPolygons(hole_with_above);
                                if (hole_difference.empty())
                                {
                                    // The hole was returned unchanged, so the layer above must completely cover it.  Remove the hole from the layer above.
                                    above = above.difference(hole_poly);
                                }
                            }
                        }
                    }
                }
            }
            // And now union with offset of the resulting above layer
            layer.polygons_ = layer.polygons_.unionPolygons(above.offset(-max_dist_from_lower_layer));
        }
    }
}

} // namespace cura

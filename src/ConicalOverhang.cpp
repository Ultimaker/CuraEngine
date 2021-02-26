//Copyright (c) 2016 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ConicalOverhang.h"
#include "mesh.h"
#include "slicer.h"
#include "settings/types/AngleRadians.h" //To process the overhang angle.

namespace cura {

	void ConicalOverhang::apply(Slicer* slicer, const Mesh& mesh)
	{
		const AngleRadians angle = mesh.settings.get<AngleRadians>("conical_overhang_angle");
		const double maxHoleArea = mesh.settings.get<double>("conical_overhang_hole_size");
		const double tan_angle = tan(angle);  // the XY-component of the angle
		const coord_t layer_thickness = mesh.settings.get<coord_t>("layer_height");
		coord_t max_dist_from_lower_layer = tan_angle * layer_thickness; // max dist which can be bridged

		for (unsigned int layer_nr = slicer->layers.size() - 2; static_cast<int>(layer_nr) >= 0; layer_nr--)
		{
			SlicerLayer& layer = slicer->layers[layer_nr];
			SlicerLayer& layer_above = slicer->layers[layer_nr + 1];
			if (std::abs(max_dist_from_lower_layer) < 5)
			{ // magically nothing happens when max_dist_from_lower_layer == 0
				// below magic code solves that
				int safe_dist = 20;
				Polygons diff = layer_above.polygons.difference(layer.polygons.offset(-safe_dist));
				layer.polygons = layer.polygons.unionPolygons(diff);
				layer.polygons = layer.polygons.smooth(safe_dist);
				layer.polygons.simplify(safe_dist, safe_dist * safe_dist / 4);
				// somehow layer.polygons get really jagged lines with a lot of vertices
				// without the above steps slicing goes really slow
			}
			else
			{
				// Get the current layer and split it into parts
				std::vector<PolygonsPart> layerParts = layer.polygons.splitIntoParts();
				// Get a copy of the layer above to prune away before we shrink it
				Polygons above = layer_above.polygons;

				// Now go through all the holes in the current layer and check if they intersect anything in the layer above
				// If not, then they're the top of a hole and should be cut from the layer above before the union
				for (unsigned int part = 0; part < layerParts.size(); part++)
				{
					if (layerParts[part].size() > 1)		// first poly is the outer contour, 1..n are the holes
					{
						for (unsigned int hole_nr = 1; hole_nr < layerParts[part].size(); ++hole_nr)
						{
							Polygons holePoly;
							holePoly.add(layerParts[part][hole_nr]);
							if (maxHoleArea > 0.0 && INT2MM(INT2MM(fabs(holePoly.area()))) < maxHoleArea)
							{
								Polygons holeWithAbove = holePoly.intersection(above);
								if (!holeWithAbove.empty())
								{
									// The hole had some intersection with the above layer, check if it's a complete overlap
									Polygons holeDifference = holePoly.xorPolygons(holeWithAbove);
									if (holeDifference.empty())
									{
										// The hole was returned unchanged, so the layer above must completely cover it.  Remove the hole from the layer above.
										above = above.difference(holePoly);
									}
								}
							}
						}
					}
				}
				// And now union with offset of the resulting above layer 
				layer.polygons = layer.polygons.unionPolygons(above.offset(-max_dist_from_lower_layer));
			}
		}
	}

}//namespace cura

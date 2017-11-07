//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "utils/polygonUtils.h" //For moveInside.

#include "TreeSupport.h"

#define SQRT_2 1.4142135623730950488 //Square root of 2.

namespace cura
{

TreeSupport::TreeSupport()
{
}

void TreeSupport::generateSupportAreas(SliceDataStorage& storage)
{
    std::vector<std::vector<Point>> contact_points;
    contact_points.reserve(storage.support.supportLayers.size());
    for (size_t layer_nr = 0; layer_nr < storage.support.supportLayers.size(); layer_nr++) //Generate empty layers to store the points in.
    {
        contact_points.emplace_back();
    }
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        if (!mesh.getSettingBoolean("support_tree_enable"))
        {
            return;
        }
        generateContactPoints(mesh, contact_points);
    }

    //Placeholder to test with that generates a simple diamond at each contact point (without generating any trees yet).
    for (size_t layer_nr = 0; layer_nr < contact_points.size(); layer_nr++)
    {
        for (Point point : contact_points[layer_nr])
        {
            PolygonsPart outline;
            Polygon diamond;
            diamond.add(point + Point(0, 1000));
            diamond.add(point + Point(-1000, 0));
            diamond.add(point + Point(0, -1000));
            diamond.add(point + Point(1000, 0));
            outline.add(diamond);
            storage.support.supportLayers[layer_nr].support_infill_parts.emplace_back(outline, 350);
        }
        if (!contact_points[layer_nr].empty())
        {
            storage.support.layer_nr_max_filled_layer = layer_nr;
        }
    }

    //TODO: Use Minimum Spanning Tree to connect the points on each layer.
    //TODO: Drop down and contract the leaves of the tree.
    //TODO: Create a pyramid out of the mesh and move points away from that.
    //TODO: When reaching the bottom, cut away all edges of the MST that are still not contracted.
    //TODO: Do a second pass of dropping down but with leftover edges removed.
    //TODO: Apply some diameter to the tree branches.
    
    storage.support.generated = true;
}

void TreeSupport::generateContactPoints(const SliceMeshStorage& mesh, std::vector<std::vector<Point>>& contact_points)
{
    for (size_t layer_nr = 0; layer_nr < mesh.overhang_areas.size(); layer_nr++)
    {
        const Polygons& overhang = mesh.overhang_areas[layer_nr];
        if (overhang.empty())
        {
            continue;
        }

        //First generate a lot of points in a grid pattern.
        Polygons outside_polygons = overhang.getOutsidePolygons();
        AABB bounding_box(outside_polygons); //To know how far we should generate points.
        coord_t point_spread = mesh.getSettingInMicrons("support_tree_branch_distance");
        point_spread *= SQRT_2; //We'll rotate these points 45 degrees, so this is the point distance when axis-aligned.
        bounding_box.round(point_spread);
        for (PolygonRef overhang_part : outside_polygons)
        {
            AABB bounding_box(outside_polygons);
            bounding_box.round(point_spread);
            bool added = false; //Did we add a point this way?
            for (coord_t x = bounding_box.min.X; x <= bounding_box.max.X; x += point_spread << 1)
            {
                for (coord_t y = bounding_box.min.Y + (point_spread << 1) * (x % 2); y <= bounding_box.max.Y; y += point_spread) //This produces points in a 45-degree rotated grid.
                {
                    Point candidate(x, y);
                    if (overhang_part.inside(candidate))
                    {
                        contact_points[layer_nr].push_back(candidate);
                        added = true;
                    }
                }
            }
            if (!added) //If we didn't add any points due to bad luck, we want to add one anyway such that loose parts are also supported.
            {
                Point candidate = bounding_box.getMiddle();
                PolygonUtils::moveInside(overhang_part, candidate);
                contact_points[layer_nr].push_back(candidate);
            }
        }
    }
}

}
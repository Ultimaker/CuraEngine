//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "TreeSupport.h"

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
    }

    //TODO: Use Minimum Spanning Tree to connect the points on each layer.
    //TODO: Drop down and contract the leaves of the tree.
    //TODO: Create a pyramid out of the mesh and move points away from that.
    //TODO: When reaching the bottom, cut away all edges of the MST that are still not contracted.
    //TODO: Do a second pass of dropping down but with leftover edges removed.
    //TODO: Apply some diameter to the tree branches.
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

        contact_points[layer_nr].push_back(overhang.back().centerOfMass()); //TODO: Place points in these areas.
    }
}

}
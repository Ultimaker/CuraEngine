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
    if (!storage.getSettingBoolean("support_tree_enable"))
    {
        return;
    }
    std::vector<std::vector<Point>> contact_points;
    generateContactPoints(storage, contact_points);

    //Placeholder to test with that generates a simple diamond at each contact point (without generating any trees yet).
    for (size_t layer_nr = 0; layer_nr < contact_points.size(); layer_nr++)
    {
        for (Point point : contact_points[layer_nr])
        {
            PolygonsPart outline;
            Polygon diamond;
            diamond.add(point + Point(1000, 0));
            diamond.add(point + Point(0, -1000));
            diamond.add(point + Point(-1000, 0));
            diamond.add(point + Point(0, 1000));
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

void TreeSupport::generateContactPoints(const SliceDataStorage& storage, std::vector<std::vector<Point>>& contact_points)
{
    //TODO: Implement this.
}

}
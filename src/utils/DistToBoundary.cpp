//Copyright (c) 2017 Tim Kuipers
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "DistToBoundary.h"
#include "../sliceDataStorage.h"
#include "polygonUtils.h"
#include "linearAlg2D.h"
#include "optional.h"

#include "VoxelGrid.h" // TODO remove this line


namespace cura
{
 


DistToBoundary::DistToBoundary(const SliceDataStorage& storage)
: storage(storage)
, layer_height(storage.getSettingInMicrons("layer_height"))
{
}

DistToBoundary::~DistToBoundary()
{
}

coord_t getDistance(int checking_layer_nr, Point location)
{
    
}

const coord_t& DistToBoundary::getValue(int layer_nr, Point location) const
{
    /*
    coord_t min_dist2_to_boundary = std::numeric_limits<coord_t>::max();
    const int layer_radius = radius / layer_height;
    std::optional<bool> is_inside_prev_layer; // unknown yet
    for (int checking_layer_nr = std::max(0, layer_nr - layer_radius); checking_layer_nr < std::min(int(storage.print_layer_count), layer_nr + layer_radius); checking_layer_nr++)
    {
        const coord_t layer_diff = (checking_layer_nr - layer_nr) / layer_height;
        constexpr bool include_layer_outlines = false;
        const Polygons outlines = storage.getLayerOutlines(checking_layer_nr, include_layer_outlines);
        const ClosestPolygonPoint cpp = PolygonUtils::findClosest(location, outlines);
        const coord_t dist2_2d = vSize2(cpp.location - location);
        const coord_t dist2_3d = dist2_2d + layer_diff * layer_diff;
        min_dist2_to_boundary = std::min(min_dist2_to_boundary, dist2_3d);

        const bool is_inside = outlines.inside(location);
        if (is_inside_prev_layer && is_inside != *is_inside_prev_layer)
        {
            min_dist2_to_boundary = std::min(min_dist2_to_boundary, layer_diff * layer_diff);
        }
        is_inside_prev_layer = is_inside;
    }
    if (min_dist2_to_boundary == std::numeric_limits<coord_t>::max())
    {
    */
        return std::numeric_limits<coord_t>::max();
        /*
    }
    coord_t dist = sqrt(min_dist2_to_boundary);
    return std::min(coord_t(0), dist - radius); // the sphere described by the radius is closer to the model than the center
    */
}

} // namespace cura

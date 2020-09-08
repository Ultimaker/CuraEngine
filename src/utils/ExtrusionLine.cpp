//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ExtrusionLine.h"

namespace cura
{

coord_t ExtrusionLine::getLength() const
{
    if (junctions.empty())
    {
        return 0;
    }
    coord_t len = 0;
    ExtrusionJunction prev = junctions.front();
    for (const ExtrusionJunction& next : junctions)
    {
        len += vSize(next.p - prev.p);
        prev = next;
    }
    return len;
}

void ExtrusionLine::appendJunctionsTo(std::vector<ExtrusionJunction>& result) const
{
    result.insert(result.end(), junctions.begin(), junctions.end());
}

BinWallJunctions getBinWallJunctions(const size_t num_insets, const WallToolPaths& wall_toolpaths)
{
    BinWallJunctions insets(num_insets); // Vector of insets (bins). Each inset is a vector of paths. Each path is a vector of lines.
    for(const std::list<ExtrusionLine>& path : wall_toolpaths)
    {
        if(path.empty()) //Don't bother printing these.
        {
            continue;
        }
        const size_t inset_index = path.front().inset_idx;

        //Convert list of extrusion lines to vectors of extrusion junctions, and add those to the binned insets.
        for(const ExtrusionLine& line : path)
        {
            insets[inset_index].emplace_back(line.junctions.begin(), line.junctions.end());
        }
    }
    return insets;
}

}
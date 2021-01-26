//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <unordered_set>

#include <gtest/gtest.h>
#include <clipper.hpp>
#include <../src/utils/VoxelUtils.h>
#include <../src/utils/AABB3D.h>
#include <../src/utils/SVG.h>

namespace cura
{

TEST(VoxelUtilsTest, TestConstructEmpty)
{
    Point3 cell_size = Point3(15, 10, 10);
    VoxelUtils vu(cell_size);
    GridPoint3 kernel(4, 8, 1);
    std::vector<GridPoint3> cells = vu.dilationKernel(kernel, true);
    std::cerr << cells.size() << '\n';
    
    Polygons cell_borders;
    for (GridPoint3 p : cells)
    {
        if (p.z != 0) continue;
        PolygonRef poly = cell_borders.newPoly();
        Point3 corner3d = vu.toLowerCorner(p);
        Point corner(corner3d.x, corner3d.y);
        poly.add(corner);
        poly.add(corner + Point(cell_size.x, 0));
        poly.add(corner + Point(cell_size.x, cell_size.y));
        poly.add(corner + Point(0, cell_size.y));
    }
    
    {
        AABB aabb(cell_borders);
        aabb.expand(10);
        SVG svg("VoxelTest.svg", aabb);
        svg.writePolygons(cell_borders);
    }
    
    // EXPECT_TRUE(cells.size() == 7);
}

} // namespace cura

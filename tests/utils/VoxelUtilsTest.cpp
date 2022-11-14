//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <unordered_set>

#include <gtest/gtest.h>
#include <../src/utils/VoxelUtils.h>

// #define TEST_SVG_OUTPUT
#ifdef TEST_SVG_OUTPUT
#include "../src/utils/polygon.h"
#include <cstdlib>
#include "../src/utils/SVG.h"
#endif //TEST_SVG_OUTPUT

namespace cura
{

class VoxelUtilsTest: public testing::Test
{
public:
    std::vector<Polygons> test_polys;
    std::vector<Polygons> test_polylines;
    
    static constexpr coord_t e = 5u; // should be less than half the cell_size
    static constexpr double allowed_area_error = 20.0 * 20.0;
    static constexpr int n_tests = 50;
    static constexpr int min_n_poly_points = 5;
    static constexpr int max_n_poly_points = 50;
    static constexpr coord_t poly_size = 2000u;
    static constexpr coord_t z = 0u;

    void SetUp()
    {
        Point3 cell_size = Point3(30, 20, 1);

        VoxelUtils vu(cell_size);
        for (int seed = 0; seed < n_tests; seed++)
        {
            std::srand(seed);
            test_polys.emplace_back();
            Polygons& polys = test_polys.back();
            {
                polys.emplace_back();
                PolygonRef rand_points = polys.back();
                int n_poly_points = min_n_poly_points + rand() % max_n_poly_points;
                for (int i = 0; i < n_poly_points; i++)
                {
                    rand_points.emplace_back(rand() % poly_size, rand() % poly_size);
                }
                polys = polys.processEvenOdd(); // xor with itself
            }
            test_polylines.emplace_back();
            Polygons& polylines = test_polylines.back();
            polylines = polys;
            for (PolygonRef poly : polylines)
            {
                if (poly.empty()) continue;
                poly.emplace_back(poly.front());
            }
        }
    }
};

TEST_F(VoxelUtilsTest, TestWalkLine)
{
    Point3 start(22, 16, 0);
    Point3 end(0, 20, 0);
    Point3 cell_size(30, 20, 1);
    VoxelUtils vu(cell_size);
    std::unordered_set<Point3> voxels;
    vu.walkLine(start, end, [&voxels](GridPoint3 v) { voxels.emplace(v); return true; } );
    ASSERT_LE(voxels.size(), 4)
        << "A line ending in the cross-section between 4 voxels can cover 1 to 4 voxels at that end point";
}
TEST_F(VoxelUtilsTest, TestWalkBasic)
{
    
    for (size_t poly_idx = 0; poly_idx < test_polys.size(); poly_idx++)
    {
        Point3 cell_size;
        switch (poly_idx * 5 / test_polys.size())
        {
            case 0: cell_size = Point3(30, 20, 1); break;
            case 1: cell_size = Point3(40, 40, 1); break;
            case 2: cell_size = Point3(25, 40, 1); break;
            case 3: cell_size = Point3(400, 400, 1); break;
            default: cell_size = Point3(poly_size, poly_size, 1); break;
        }
        coord_t max_dist_from_poly = vSize(Point(cell_size.x, cell_size.y)) + e;

        VoxelUtils vu(cell_size);
        
        
        const Polygons& polys = test_polys[poly_idx];
        const Polygons& polylines = test_polylines[poly_idx];
        
        
#ifdef TEST_SVG_OUTPUT
        {
            SVG svg("/tmp/VoxelUtilsTest_before.svg", AABB(polys));
            svg.writePolygons(polys);
        }
#endif // TEST_SVG_OUTPUT
        
        // test walkPolygons
        std::unordered_set<GridPoint3> voxels;
        vu.walkPolygons(polys, z, [&voxels](GridPoint3 v){ voxels.emplace(v); return true; } );
        
        Polygons voxel_area;
        for (const GridPoint3& v : voxels)
        {
            voxel_area.add(vu.toPolygon(v));
        }
        voxel_area.unionPolygons();

#ifdef TEST_SVG_OUTPUT
        {
            AABB aabb(polys);
            aabb.expand(cell_size.vSize());
            SVG svg("/tmp/VoxelUtilsTest_walkPolygons.svg", aabb);
            svg.writeAreas(voxel_area, SVG::Color::RED);
            svg.nextLayer();
            svg.writePolygons(polys);
            svg.nextLayer();
            svg.writePolygons(voxel_area, SVG::Color::MAGENTA);
            svg.nextLayer();
            svg.writePolygons(voxel_area.offset(e), SVG::Color::ORANGE);
            svg.nextLayer();
            svg.writePolygons(polylines.offsetPolyLine(e / 2, ClipperLib::jtRound), SVG::Color::GRAY);
            svg.nextLayer();
            svg.writePolygons(polylines.offsetPolyLine(max_dist_from_poly), SVG::Color::GRAY);
            svg.nextLayer();
            svg.writePolygons(voxel_area.difference(polylines.offsetPolyLine(max_dist_from_poly)), SVG::Color::BLUE);
        }
#endif // TEST_SVG_OUTPUT

        // check completeness
        ASSERT_GE(voxel_area.area() + allowed_area_error, voxel_area.offset(e).unionPolygons(polylines.offsetPolyLine(e / 2, ClipperLib::jtRound)).offset(-e).area())
            << "walkPolygons should provide voxels covering all segments of the input polygons. Seed: " << poly_idx;
        // check accuracy
        ASSERT_LE(voxel_area.difference(polylines.offsetPolyLine(max_dist_from_poly)).area(), allowed_area_error)
            << "walkPolygons should not include voxels which don't overlap with the polygon Seed: " << poly_idx;


        // test walkAreas
        vu.walkAreas(polys, z, [&voxels](GridPoint3 v) { voxels.emplace(v); return true; } ); // add area voxels to the boundary voxels, cause otherwise small areas might have missed 
        
        voxel_area.clear();
        for (const GridPoint3& v : voxels)
        {
            voxel_area.add(vu.toPolygon(v));
        }
        voxel_area.unionPolygons();

#ifdef TEST_SVG_OUTPUT
        {
            AABB aabb(polys);
            aabb.expand(cell_size.vSize());
            SVG svg("/tmp/VoxelUtilsTest_walkAreas.svg", aabb);
            svg.writeAreas(polys.offset(-e), SVG::Color::GRAY);
            svg.nextLayer();
            svg.writeAreas(voxel_area, SVG::Color::ORANGE);
            svg.nextLayer();
            svg.writePolygons(polys);
            svg.nextLayer();
            svg.writePolygons(voxel_area, SVG::Color::MAGENTA);
            svg.nextLayer();
            svg.writePolygons(polys.offset(max_dist_from_poly), SVG::Color::GRAY);
        }
#endif // TEST_SVG_OUTPUT

        ASSERT_GE(voxel_area.area() + allowed_area_error, voxel_area.unionPolygons(polys).area())
            << "walkAreas should provide voxels covering all areas of the input polygons. Seed: " << poly_idx;
        // check accuracy
        Polygons covered_area = polys.offset(max_dist_from_poly);
        covered_area = covered_area.unionPolygons(polylines.offsetPolyLine(max_dist_from_poly)); // prevent clipper bug which sometimes discards outside colinear lines as if they are holes.
        ASSERT_LE(voxel_area.difference(covered_area).area(), allowed_area_error)
            << "walkAreas should not include voxels which don't overlap with the polygon area. Seed: " << poly_idx;

    }
}

TEST_F(VoxelUtilsTest, TestWalkDilated)
{
    for (size_t poly_idx = 0; poly_idx < test_polys.size(); poly_idx++)
    {
        Point3 cell_size;
        switch (poly_idx * 5 / test_polys.size())
        {
            case 0: cell_size = Point3(30, 20, 1); break;
            case 1: cell_size = Point3(40, 40, 1); break;
            case 2: cell_size = Point3(25, 40, 1); break;
            case 3: cell_size = Point3(400, 400, 1); break;
            default: cell_size = Point3(poly_size, poly_size, 1); break;
        }
        coord_t kernel_s = 1 + (poly_idx % 5);
        GridPoint3 kernel_size(kernel_s, kernel_s, 1);
        DilationKernel kernel(kernel_size, DilationKernel::Type::CUBE);

        Point3 applied_offset = cell_size * kernel_size / 2 - cell_size / 2;
        coord_t max_dist_from_poly = vSize(Point(cell_size.x, cell_size.y) + Point(applied_offset.x, applied_offset.y)) + e;
        coord_t min_dist_from_poly = std::min(applied_offset.x, applied_offset.y);
        // divide by 2, because the kernel is centered around the points in the poly
        // again divided by 2, because 

        VoxelUtils vu(cell_size);
        
        const Polygons& polys = test_polys[poly_idx];
        const Polygons& polylines = test_polylines[poly_idx];
        
        
#ifdef TEST_SVG_OUTPUT
        {
            SVG svg("/tmp/VoxelUtilsTest_before.svg", AABB(polys));
            svg.writePolygons(polys);
        }
#endif // TEST_SVG_OUTPUT
        
        // test walkPolygons
        std::unordered_set<GridPoint3> voxels;
        vu.walkDilatedPolygons(polys, z, kernel, [&voxels](GridPoint3 v){ voxels.emplace(v); return true; } );
        
        Polygons voxel_area;
        for (const GridPoint3& v : voxels)
        {
            voxel_area.add(vu.toPolygon(v));
        }
        voxel_area.unionPolygons();

#ifdef TEST_SVG_OUTPUT
        {
            AABB aabb(polys);
            aabb.expand(cell_size.vSize()*2);
            SVG svg("/tmp/VoxelUtilsTest_walkPolygons.svg", aabb);
            svg.writeAreas(voxel_area, SVG::Color::RED);
            svg.nextLayer();
            svg.writePolygons(polys);
            svg.nextLayer();
            // reference polygon on which the walk is performed
            const Point3 translation = (Point3(1,1,1) - kernel.kernel_size % 2) * cell_size / 2;
            Polygons translated = polys;
            translated.translate(Point(translation.x, translation.y));
            svg.writePolygons(translated, SVG::Color::GREEN);
            svg.nextLayer();
            svg.writePolygons(voxel_area, SVG::Color::MAGENTA);
            svg.nextLayer();
            svg.writePolygons(polylines.offsetPolyLine(min_dist_from_poly, ClipperLib::jtRound), SVG::Color::GRAY);
            svg.nextLayer();
            svg.writePolygons(polylines.offsetPolyLine(max_dist_from_poly, ClipperLib::jtRound), SVG::Color::GRAY);
        }
#endif // TEST_SVG_OUTPUT

        // check completeness
        ASSERT_GE(voxel_area.area() + allowed_area_error, voxel_area.unionPolygons(polylines.offsetPolyLine(min_dist_from_poly, ClipperLib::jtRound)).area()) 
            << "walkPolygons should provide voxels covering all segments of the input polygons. Seed: " << poly_idx;
        // check accuracy
        ASSERT_LE(voxel_area.difference(polylines.offsetPolyLine(max_dist_from_poly, ClipperLib::jtRound)).area(), allowed_area_error)
            << "walkPolygons should not include voxels which don't overlap with the polygon Seed: " << poly_idx;


        // test walkAreas
        vu.walkDilatedAreas(polys, z, kernel, [&voxels](GridPoint3 v) { voxels.emplace(v); return true; } ); // add area voxels to the boundary voxels, cause otherwise small areas might have missed 
        
        voxel_area.clear();
        for (const GridPoint3& v : voxels)
        {
            voxel_area.add(vu.toPolygon(v));
        }
        voxel_area.unionPolygons();

#ifdef TEST_SVG_OUTPUT
        {
            AABB aabb(polys);
            aabb.expand(cell_size.vSize());
            SVG svg("/tmp/VoxelUtilsTest_walkAreas.svg", aabb);
            svg.writeAreas(polys.offset(-e), SVG::Color::GRAY);
            svg.nextLayer();
            svg.writeAreas(voxel_area, SVG::Color::ORANGE);
            svg.nextLayer();
            svg.writePolygons(polys);
            svg.nextLayer();
            svg.writePolygons(voxel_area, SVG::Color::MAGENTA);
            svg.nextLayer();
            svg.writePolygons(polys.offset(max_dist_from_poly, ClipperLib::jtRound), SVG::Color::GRAY);
            svg.nextLayer();
            svg.writePolygons(polys.offset(min_dist_from_poly, ClipperLib::jtRound), SVG::Color::GRAY);
        }
#endif // TEST_SVG_OUTPUT

        // check completeness
        ASSERT_GE(voxel_area.area() + allowed_area_error, voxel_area.unionPolygons(polys.offset(min_dist_from_poly, ClipperLib::jtRound)).area())
            << "walkAreas should provide voxels covering all areas of the input polygons. Seed: " << poly_idx;
        // check accuracy
        Polygons covered_area = polys.offset(max_dist_from_poly, ClipperLib::jtRound);
        covered_area = covered_area.unionPolygons(polylines.offsetPolyLine(max_dist_from_poly, ClipperLib::jtRound)); // prevent clipper bug which sometimes discards outside colinear lines as if they are holes.
        ASSERT_LE(voxel_area.difference(covered_area).area(), allowed_area_error)
            << "walkAreas should not include voxels which don't overlap with the polygon area. Seed: " << poly_idx;

    }
}

} // namespace cura

//Copyright (C) 2017 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SierpinskiFillProvider.h"

#include "../utils/math.h"
#include "../utils/SVG.h"
#include "../utils/STL.h"
#include "../Application.h"

namespace cura
{


constexpr bool SierpinskiFillProvider::get_constructor;
constexpr bool SierpinskiFillProvider::use_dithering;


MappingFunction SierpinskiFillProvider::getMappingFunction()
{
    std::vector<float> points({
 0.000, 0.002662, 0.008818, 0.014887, 0.021019, 0.027313, 0.033307, 0.038593, 0.043910, 0.049691, 0.055140, 0.060631, 0.066417, 0.071953, 0.076510, 0.080970, 0.086806, 0.094257, 0.100854, 0.105132, 0.108166, 0.111723, 0.117065, 0.124854, 0.134681, 0.143140, 0.148724, 0.152095, 0.154462, 0.156856, 0.160018, 0.164663, 0.171561, 0.180989, 0.190973, 0.199343, 0.205496, 0.209640, 0.212211, 0.213890, 0.215596, 0.218246, 0.222164, 0.226870, 0.232264, 0.239152, 0.248723, 0.261276, 0.274136, 0.284127, 0.290987, 0.295821, 0.299175, 0.301572, 0.303633, 0.305909, 0.308615, 0.311745, 0.315138, 0.318612, 0.322047, 0.325530, 0.329401, 0.334455, 0.343000, 0.359046, 0.382365, 0.397266, 0.403027, 0.404615, 0.406568, 0.410453, 0.415120, 0.419277, 0.422437, 0.424425, 0.425377, 0.425831, 0.426371, 0.427580, 0.430026, 0.433895, 0.438849, 0.444178, 0.449262, 0.453948, 0.458472, 0.463607, 0.470298, 0.478548, 0.488017, 0.498175, 0.509372, 0.518936, 0.527263, 0.539938, 0.553643, 0.563154, 0.569208, 0.574194, 0.579800, 0.585535, 0.590089, 0.593461, 0.596103, 0.598337, 0.600244, 0.601748, 0.602682, 0.603174, 0.603423, 0.603630, 0.603993, 0.604713, 0.605987, 0.607893, 0.610264, 0.612984, 0.616131, 0.619590, 0.622988, 0.626305, 0.629410, 0.631584, 0.632012, 0.632792, 0.639466, 0.664523, 0.732594
    });
    return MappingFunction(points, 0.0, 0.77);
}

SierpinskiFillProvider::SierpinskiFillProvider(const SliceMeshStorage* mesh_data, const AABB3D aabb_3d, coord_t min_line_distance, const coord_t line_width, float density,
                                               bool dense_at_top, float cross_infill_top_density, bool use_skin)
: aabb_3d(aabb_3d)
, fractal_config(getFractalConfig(aabb_3d, min_line_distance, /* make_3d = */ true))
, average_density_provider(new UniformDensityProvider(density))
, skin_density_provider((dense_at_top && mesh_data)? new TopSkinDensityProvider(*mesh_data, cross_infill_top_density, use_skin) : nullptr)
, combined_density_provider(skin_density_provider? new CombinedDensityProvider(*average_density_provider, *skin_density_provider) : nullptr)
, in_out_density_correction(getMappingFunction())
, compensated_density_provider(new CompensatedDensityProvider(combined_density_provider? *combined_density_provider : *average_density_provider, in_out_density_correction.getFunction()))
, density_provider(compensated_density_provider? compensated_density_provider : (combined_density_provider? combined_density_provider : average_density_provider))
// , fill_pattern_for_all_layers(get_constructor, *density_provider, fractal_config.aabb.flatten(), fractal_config.depth, line_width, use_dithering)
, subdivision_structure_3d(get_constructor, *density_provider, fractal_config.aabb, fractal_config.depth, line_width)
{
    subdivision_structure_3d->initialize();
    if (density >= 1.0 || density >= static_cast<float>(line_width) / static_cast<float>(min_line_distance))
    {
        logDebug("Creating max depth pattern.\n");
        subdivision_structure_3d->createMaxDepthPattern();
    }
    else
    {
        logDebug("Creating dithered pattern.\n");
        subdivision_structure_3d->createDitheredPattern();
        if (dense_at_top)
        {
            subdivision_structure_3d->createMinimalDensityPattern(); // based on minimal required density based on top skin
        }
    }
    z_to_start_cell_cross3d = subdivision_structure_3d->getSequenceStarts();
    edge_network.emplace(*subdivision_structure_3d);
}

SierpinskiFillProvider::SierpinskiFillProvider(const SliceMeshStorage* mesh_data, const AABB3D aabb_3d, coord_t min_line_distance, coord_t line_width,
                                               std::string cross_subdisivion_spec_image_file, float min_density, float max_density, float transparency_density,
                                               bool dense_at_top, float cross_infill_top_density, bool use_skin)
: aabb_3d(aabb_3d)
, fractal_config(getFractalConfig(aabb_3d, min_line_distance, /* make_3d = */ true))
, average_density_provider(new ImageBasedDensityProvider(cross_subdisivion_spec_image_file, aabb_3d, min_density, max_density, transparency_density))
, skin_density_provider((dense_at_top && mesh_data)? new TopSkinDensityProvider(*mesh_data, cross_infill_top_density, use_skin) : nullptr)
, combined_density_provider(skin_density_provider? new CombinedDensityProvider(*average_density_provider, *skin_density_provider) : nullptr)
, in_out_density_correction(getMappingFunction())
, compensated_density_provider(new CompensatedDensityProvider(combined_density_provider? *combined_density_provider : *average_density_provider, in_out_density_correction.getFunction()))
, density_provider(compensated_density_provider? compensated_density_provider : (combined_density_provider? combined_density_provider : average_density_provider))
// , fill_pattern_for_all_layers(get_constructor, *density_provider, fractal_config.aabb.flatten(), fractal_config.depth, line_width, use_dithering)
, subdivision_structure_3d(get_constructor, *density_provider, fractal_config.aabb, fractal_config.depth, line_width)
{
    subdivision_structure_3d->initialize();
    subdivision_structure_3d->createDitheredPattern();
//     subdivision_structure_3d->sanitize();
    if (dense_at_top)
    {
        subdivision_structure_3d->createMinimalDensityPattern(); // based on minimal required density based on top skin
    }
    z_to_start_cell_cross3d = subdivision_structure_3d->getSequenceStarts();
    edge_network.emplace(*subdivision_structure_3d);
}

SierpinskiFillProvider::SierpinskiFillProvider(const SliceMeshStorage* mesh_data, const AABB3D aabb_3d, coord_t min_line_distance, const coord_t line_width,
                                               std::string cross_subdisivion_spec_image_file, float min_density, float max_density, float transparency_density,
                                               bool dense_at_top, float cross_infill_top_density, bool use_skin, bool)
: aabb_3d(aabb_3d)
, fractal_config(getFractalConfig(aabb_3d, min_line_distance, /* make_3d = */ true))
, average_density_provider(new ImageBasedDensityProvider(cross_subdisivion_spec_image_file, aabb_3d, min_density, max_density, transparency_density))
, skin_density_provider((dense_at_top && mesh_data)? new TopSkinDensityProvider(*mesh_data, cross_infill_top_density, use_skin) : nullptr)
, combined_density_provider(skin_density_provider? new CombinedDensityProvider(*average_density_provider, *skin_density_provider) : nullptr)
, in_out_density_correction(getMappingFunction())
, compensated_density_provider(new CompensatedDensityProvider(combined_density_provider? *combined_density_provider : *average_density_provider, in_out_density_correction.getFunction()))
, density_provider(compensated_density_provider? compensated_density_provider : (combined_density_provider? combined_density_provider : average_density_provider))
, subdivision_structure_3d(get_constructor, *density_provider, fractal_config.aabb, fractal_config.depth, line_width)
{
    subdivision_structure_3d->initialize();
    subdivision_structure_3d->createDitheredPattern();
//     subdivision_structure_3d->sanitize();
    if (dense_at_top)
    {
        TimeKeeper tk;
        subdivision_structure_3d->createMinimalDensityPattern(); // based on minimal required density based on top skin
        logDebug("Top skin density minimum enforced in %5.2fs.\n", tk.restart());
    }
    z_to_start_cell_cross3d = subdivision_structure_3d->getSequenceStarts();
    edge_network.emplace(*subdivision_structure_3d);
    writeToSTL("output/generated.stl");
}

Polygon SierpinskiFillProvider::generate(EFillMethod pattern, coord_t z, coord_t line_width, coord_t pocket_size) const
{
    TimeKeeper tk;
    Polygon ret;
    z = std::min(z, aabb_3d.max.z - 1); // limit the z to where the pattern is generated; layer heights can go higher than the model...
    if (fill_pattern_for_all_layers)
    {
        if (pattern == EFillMethod::CROSS_3D)
        {
            ret = fill_pattern_for_all_layers->generateCross(z, line_width / 2, pocket_size);
        }
        else
        {
            ret = fill_pattern_for_all_layers->generateCross();
        }
    }
    else if (subdivision_structure_3d)
    {
        std::map<coord_t, const Cross3D::Cell*>::const_iterator start_cell_iter = z_to_start_cell_cross3d.upper_bound(z);
        if (start_cell_iter != z_to_start_cell_cross3d.begin())
        { // don't get a start cell below the bottom one
            start_cell_iter--; // map.upper_bound always rounds up, while the map contains the min of the z_range of the cells
        }
        Cross3D::SliceWalker slicer_walker = subdivision_structure_3d->getSequence(*start_cell_iter->second, z);
        if (pattern == EFillMethod::CROSS_3D)
        {
            ret = subdivision_structure_3d->generateCross3D(slicer_walker, *edge_network, z);
        }
        else
        {
            ret = subdivision_structure_3d->generateCross(slicer_walker);
        }
    }
    else
    {
        logError("Different density sierpinski fill for different layers is not implemented yet!\n");
        std::exit(-1);
    }
    const_cast<SierpinskiFillProvider*>(this)->polygon_creation_time += tk.restart();
    if (false)
    {
        char filename[1024];
        coord_t layer_height = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<coord_t>("layer_height");
        std::sprintf(filename, "output/fractal_dithering_layers/overview/layer_%lli.svg", z / layer_height);
        SVG svg(filename, aabb_3d.flatten(), /*Point canvas_size =*/ Point(512, 512), /*Color background =*/ SVG::Color::WHITE, SVG::OMIT_BORDERS);
        float drawing_line_width = line_width * svg.getScale();
        svg.writePolygon(ret, SVG::Color::BLACK, drawing_line_width);
    }
    return ret;
}


void SierpinskiFillProvider::generateSubdivStructureLines(EFillMethod pattern, coord_t z, coord_t line_width, Polygons& result_polygons, Polygons& result_lines, bool closed) const
{
    z = std::min(z, aabb_3d.max.z - 1); // limit the z to where the pattern is generated; layer heights can go higher than the model...
    assert(subdivision_structure_3d);
    std::map<coord_t, const Cross3D::Cell*>::const_iterator start_cell_iter = z_to_start_cell_cross3d.upper_bound(z);
    if (start_cell_iter != z_to_start_cell_cross3d.begin())
    { // don't get a start cell below the bottom one
        start_cell_iter--; // map.upper_bound always rounds up, while the map contains the min of the z_range of the cells
    }
    Cross3D::SliceWalker slicer_walker = subdivision_structure_3d->getSequence(*start_cell_iter->second, z);
    subdivision_structure_3d->generateSubdivisionEdges(slicer_walker, z, result_polygons, result_lines, closed);
}

SierpinskiFillProvider::~SierpinskiFillProvider()
{
    if (average_density_provider)
    {
        delete average_density_provider;
    }
    if (skin_density_provider)
    {
        delete skin_density_provider;
    }
    if (combined_density_provider)
    {
        delete combined_density_provider;
    }
    if (compensated_density_provider)
    {
        delete compensated_density_provider;
    }
}

SierpinskiFillProvider::FractalConfig SierpinskiFillProvider::getFractalConfig(const AABB3D aabb_3d, coord_t min_line_distance, bool make_3d)
{
    Point3 model_aabb_size = aabb_3d.max - aabb_3d.min;
    coord_t max_side_length = std::max(model_aabb_size.x, model_aabb_size.y);
    if (make_3d)
    {
        max_side_length = std::max(max_side_length, model_aabb_size.z);
    }
    Point3 model_middle = aabb_3d.getMiddle();

    int depth = 0;
    float aabb_size = min_line_distance * sqrt2;
    while (aabb_size < max_side_length)
    {
        aabb_size *= 2;
        depth += 2;
    }
    const float half_sqrt2 = .5 * sqrt2;
    if (aabb_size * half_sqrt2 >= max_side_length - 1)
    {
        aabb_size *= half_sqrt2;
        depth--;
    }

    Point3 radius(aabb_size / 2, aabb_size / 2, aabb_size / 2);
    AABB3D aabb(model_middle - radius, model_middle + radius);
    return FractalConfig{depth, aabb};
}


void SierpinskiFillProvider::writeToSTL(const std::string filename)
{
    STL stl(filename);
    
    using Cell = Cross3D::Cell;
    for (const auto& cells : subdivision_structure_3d->getDepthOrdered())
    {
        for (const Cell* cell : cells)
        {
            Range<coord_t> z_range = cell->elem.z_range;
            coord_t bottom_quarter_z = (z_range.middle() + z_range.min) / 2;
            coord_t top_quarter_z = (z_range.middle() + z_range.max) / 2;
            const Cell* left_neighbors[2] = {nullptr, nullptr};
            const Cell* right_neighbors[2] = {nullptr, nullptr};
            { // set left_neighbors and right_neighbors
                for (const Cross3D::Link& left_neighbor_link : cell->adjacent_cells[static_cast<size_t>(Cross3D::InfillFractal2D::Direction::LEFT)])
                {
                    const Cell& neighbor = subdivision_structure_3d->cell_data[left_neighbor_link.to_index];
                    if (neighbor.elem.z_range.inside(bottom_quarter_z))
                    {
                        left_neighbors[0] = &neighbor;
                    }
                    if (neighbor.elem.z_range.inside(top_quarter_z))
                    {
                        left_neighbors[1] = &neighbor;
                    }
                }
                for (const Cross3D::Link& right_neighbor_link : cell->adjacent_cells[static_cast<size_t>(Cross3D::InfillFractal2D::Direction::RIGHT)])
                {
                    const Cell& neighbor = subdivision_structure_3d->cell_data[right_neighbor_link.to_index];
                    if (neighbor.elem.z_range.inside(bottom_quarter_z))
                    {
                        right_neighbors[0] = &neighbor;
                    }
                    if (neighbor.elem.z_range.inside(top_quarter_z))
                    {
                        right_neighbors[1] = &neighbor;
                    }
                }
            }
            assert(left_neighbors[0]);
            assert(left_neighbors[1]);
            assert(right_neighbors[0]);
            assert(right_neighbors[1]);
            
            for (int half = 0; half <= 1; half++)
            {
                coord_t z_min = z_range.min + half * z_range.size() / 2;
                coord_t z_max = z_range.middle() + half * z_range.size() / 2;
                Point3 tlb =     toPoint3(edge_network->getCellEdgeLocation(*left_neighbors[half], *cell, z_min), z_min);
                Point3 tlt =        toPoint3(edge_network->getCellEdgeLocation(*left_neighbors[half], *cell, z_max), z_max);
                Point3 trb =    toPoint3(edge_network->getCellEdgeLocation(*cell, *right_neighbors[half], z_min), z_min);
                Point3 trt =       toPoint3(edge_network->getCellEdgeLocation(*cell, *right_neighbors[half], z_max), z_max);
                bool coplanar = false;
                {
                    coord_t a1 = tlt.x - tlb.x;
                    coord_t b1 = tlt.y - tlb.y;
                    coord_t c1 = tlt.z - tlb.z;
                    coord_t a2 = trb.x - tlb.x;
                    coord_t b2 = trb.y - tlb.y;
                    coord_t c2 = trb.z - tlb.z;
                    coord_t a = b1 * c2 - b2 * c1;
                    coord_t b = a2 * c1 - a1 * c2;
                    coord_t c = a1 * b2 - b1 * a2;
                    coord_t d = (- a * tlb.x - b * tlb.y - c * tlb.z);
                    if (std::abs(a * trt.x + b * trt.y + c * trt.z + d) <= 100)
                    {
                        coplanar = true;
                    }
                }
                
                if (coplanar)
                {
                    stl.writeFace(tlb, tlt, trb);
                    stl.writeFace(trb, tlt, trt);
                }
                else
                { // make double curved surface
                    float vertical_step_size = (static_cast<float>(z_range.max) - static_cast<float>(z_range.min)) / 8;
                    for (float z = z_min; z < z_max - 10; z += vertical_step_size)
                    {
                        coord_t z_bottom = static_cast<coord_t>(z);
                        coord_t z_top = static_cast<coord_t>(z + vertical_step_size);
                        Point3 left_bottom =     toPoint3(edge_network->getCellEdgeLocation(*left_neighbors[half], *cell, z_bottom), z_bottom);
                        Point3 left_top =        toPoint3(edge_network->getCellEdgeLocation(*left_neighbors[half], *cell, z_top), z_top);
                        Point3 right_bottom =    toPoint3(edge_network->getCellEdgeLocation(*cell, *right_neighbors[half], z_bottom), z_bottom);
                        Point3 right_top =       toPoint3(edge_network->getCellEdgeLocation(*cell, *right_neighbors[half], z_top), z_top);
                        Point3 bottom_dir = right_bottom - left_bottom;
                        Point3 top_dir = right_top - left_top;
                        float horizontal_step_size = 1.0 / 4.0;
                        for (float horizontal_part = 0; horizontal_part < .99; horizontal_part += horizontal_step_size)
                        {
                            Point3 lb = left_bottom + bottom_dir * horizontal_part;
                            Point3 lt = left_top + top_dir * horizontal_part;
                            Point3 rb = left_bottom + bottom_dir * (horizontal_part + horizontal_step_size);
                            Point3 rt = left_top + top_dir * (horizontal_part + horizontal_step_size);
                            stl.writeFace(lb, lt, rb);
                            stl.writeFace(rb, lt, rt);
                        }
                    }
                }
            }
        }
    }
}


}; // namespace cura

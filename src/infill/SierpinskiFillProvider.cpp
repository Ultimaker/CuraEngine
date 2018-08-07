//Copyright (C) 2017 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SierpinskiFillProvider.h"

#include "../utils/math.h"

namespace cura
{


constexpr bool SierpinskiFillProvider::get_constructor;
constexpr bool SierpinskiFillProvider::use_dithering;

SierpinskiFillProvider::SierpinskiFillProvider(const AABB3D aabb_3d, coord_t min_line_distance, const coord_t line_width)
: aabb_3d(aabb_3d)
, fractal_config(getFractalConfig(aabb_3d, min_line_distance, /* make_3d = */ true))
, density_provider(new UniformDensityProvider((float)line_width / min_line_distance))
// , fill_pattern_for_all_layers(get_constructor, *density_provider, fractal_config.aabb.flatten(), fractal_config.depth, line_width, use_dithering)
, subdivision_structure_3d(get_constructor, *density_provider, fractal_config.aabb, fractal_config.depth, line_width)
{
    subdivision_structure_3d->initialize();
    subdivision_structure_3d->createMaxDepthPattern();
//     subdivision_structure_3d->sanitize();
    z_to_start_cell_cross3d = subdivision_structure_3d->getSequenceStarts();
}

SierpinskiFillProvider::SierpinskiFillProvider(const AABB3D aabb_3d, coord_t min_line_distance, coord_t line_width, std::string cross_subdisivion_spec_image_file)
: aabb_3d(aabb_3d)
, fractal_config(getFractalConfig(aabb_3d, min_line_distance, /* make_3d = */ true))
, density_provider(new ImageBasedDensityProvider(cross_subdisivion_spec_image_file, aabb_3d))
// , fill_pattern_for_all_layers(get_constructor, *density_provider, fractal_config.aabb.flatten(), fractal_config.depth, line_width, use_dithering)
, subdivision_structure_3d(get_constructor, *density_provider, fractal_config.aabb, fractal_config.depth, line_width)
{
    subdivision_structure_3d->initialize();
//     subdivision_structure_3d->createMinimalDensityPattern();
    subdivision_structure_3d->createDitheredPattern();
//     subdivision_structure_3d->sanitize();
    z_to_start_cell_cross3d = subdivision_structure_3d->getSequenceStarts();
}

SierpinskiFillProvider::SierpinskiFillProvider(const AABB3D aabb_3d, coord_t min_line_distance, const coord_t line_width, std::string cross_subdisivion_spec_image_file, bool)
: aabb_3d(aabb_3d)
, fractal_config(getFractalConfig(aabb_3d, min_line_distance, /* make_3d = */ true))
, density_provider(new ImageBasedDensityProvider(cross_subdisivion_spec_image_file, aabb_3d))
, subdivision_structure_3d(get_constructor, *density_provider, fractal_config.aabb, fractal_config.depth, line_width)
{
    subdivision_structure_3d->initialize();
//     subdivision_structure_3d->createMinimalDensityPattern();
    subdivision_structure_3d->createDitheredPattern();
//     subdivision_structure_3d->sanitize();
    z_to_start_cell_cross3d = subdivision_structure_3d->getSequenceStarts();
}

Polygon SierpinskiFillProvider::generate(EFillMethod pattern, coord_t z, coord_t line_width, coord_t pocket_size) const
{
    z = std::min(z, aabb_3d.max.z); // limit the z to where the pattern is generated; layer heights can go higher than the model...
    if (fill_pattern_for_all_layers)
    {
        if (pattern == EFillMethod::CROSS_3D)
        {
            return fill_pattern_for_all_layers->generateCross(z, line_width / 2, pocket_size);
        }
        else
        {
            return fill_pattern_for_all_layers->generateCross();
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
            return subdivision_structure_3d->generateCross3D(slicer_walker, z);
        }
        else
        {
            return subdivision_structure_3d->generateCross(slicer_walker);
        }
    }
    else
    {
        Polygon ret;
        logError("Different density sierpinski fill for different layers is not implemented yet!\n");
        std::exit(-1);
        return ret;
    }
}

SierpinskiFillProvider::~SierpinskiFillProvider()
{
    if (density_provider)
    {
        delete density_provider;
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
    coord_t aabb_size = min_line_distance;
    while (aabb_size < max_side_length)
    {
        aabb_size *= 2;
        depth += 2;
    }
    const float half_sqrt2 = .5 * sqrt2;
    if (!make_3d && aabb_size * half_sqrt2 >= max_side_length)
    {
        aabb_size *= half_sqrt2;
        depth--;
    }

    Point3 radius(aabb_size / 2, aabb_size / 2, aabb_size / 2);
    AABB3D aabb(model_middle - radius, model_middle + radius);
    return FractalConfig{depth, aabb};
}



}; // namespace cura

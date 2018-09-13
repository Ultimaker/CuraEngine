//Copyright (C) 2017 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "SierpinskiFillProvider.h"

#include "../utils/math.h"

namespace cura
{


constexpr bool SierpinskiFillProvider::get_constructor;
constexpr bool SierpinskiFillProvider::use_dithering;

SierpinskiFillProvider::SierpinskiFillProvider(const SliceMeshStorage* mesh_data, const AABB3D aabb_3d, coord_t min_line_distance, const coord_t line_width, float density, bool dense_at_top, bool use_skin)
: aabb_3d(aabb_3d)
, fractal_config(getFractalConfig(aabb_3d, min_line_distance, /* make_3d = */ true))
, average_density_provider(new UniformDensityProvider(density))
, skin_density_provider((dense_at_top && mesh_data)? new TopSkinDensityProvider(*mesh_data, use_skin) : nullptr)
, combined_density_provider(skin_density_provider? new CombinedDensityProvider(*average_density_provider, *skin_density_provider) : nullptr)
, density_provider(combined_density_provider? combined_density_provider : average_density_provider)
// , fill_pattern_for_all_layers(get_constructor, *density_provider, fractal_config.aabb.flatten(), fractal_config.depth, line_width, use_dithering)
, subdivision_structure_3d(get_constructor, *density_provider, fractal_config.aabb, fractal_config.depth, line_width)
{
    subdivision_structure_3d->initialize();
    if (density >= 1.0 || density >= line_width / min_line_distance)
    {
        logDebug("Creating max depth pattern.\n");
        subdivision_structure_3d->createMaxDepthPattern();
    }
    else
    {
        logDebug("Creating dithered pattern.\n");
        subdivision_structure_3d->createDitheredPattern();
        subdivision_structure_3d->createMinimalDensityPattern(); // based on minimal required density based on top skin
    }
    z_to_start_cell_cross3d = subdivision_structure_3d->getSequenceStarts();
}

SierpinskiFillProvider::SierpinskiFillProvider(const SliceMeshStorage* mesh_data, const AABB3D aabb_3d, coord_t min_line_distance, coord_t line_width, std::string cross_subdisivion_spec_image_file, float min_density, float max_density, float transparency_density, bool dense_at_top, bool use_skin)
: aabb_3d(aabb_3d)
, fractal_config(getFractalConfig(aabb_3d, min_line_distance, /* make_3d = */ true))
, average_density_provider(new ImageBasedDensityProvider(cross_subdisivion_spec_image_file, aabb_3d, min_density, max_density, transparency_density))
, skin_density_provider((dense_at_top && mesh_data)? new TopSkinDensityProvider(*mesh_data, use_skin) : nullptr)
, combined_density_provider(skin_density_provider? new CombinedDensityProvider(*average_density_provider, *skin_density_provider) : nullptr)
, density_provider(combined_density_provider? combined_density_provider : average_density_provider)
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
}

SierpinskiFillProvider::SierpinskiFillProvider(const SliceMeshStorage* mesh_data, const AABB3D aabb_3d, coord_t min_line_distance, const coord_t line_width, std::string cross_subdisivion_spec_image_file, float min_density, float max_density, float transparency_density, bool dense_at_top, bool use_skin, bool)
: aabb_3d(aabb_3d)
, fractal_config(getFractalConfig(aabb_3d, min_line_distance, /* make_3d = */ true))
, average_density_provider(new ImageBasedDensityProvider(cross_subdisivion_spec_image_file, aabb_3d, min_density, max_density, transparency_density))
, skin_density_provider((dense_at_top && mesh_data)? new TopSkinDensityProvider(*mesh_data, use_skin) : nullptr)
, combined_density_provider(skin_density_provider? new CombinedDensityProvider(*average_density_provider, *skin_density_provider) : nullptr)
, density_provider(combined_density_provider? combined_density_provider : average_density_provider)
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
}

Polygon SierpinskiFillProvider::generate(EFillMethod pattern, coord_t z, coord_t line_width, coord_t pocket_size) const
{
    z = std::min(z, aabb_3d.max.z - 1); // limit the z to where the pattern is generated; layer heights can go higher than the model...
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



}; // namespace cura

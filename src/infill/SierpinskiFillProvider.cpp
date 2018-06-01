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
: fractal_config(getFractalConfig(aabb_3d, min_line_distance, false))
, density_provider(new UniformDensityProvider((float)line_width / min_line_distance))
, fill_pattern_for_all_layers(get_constructor, *density_provider, fractal_config.aabb.flatten(), fractal_config.depth, line_width, use_dithering)
{
}

SierpinskiFillProvider::SierpinskiFillProvider(const AABB3D aabb_3d, coord_t min_line_distance, coord_t line_width, std::string cross_subdisivion_spec_image_file)
: fractal_config(getFractalConfig(aabb_3d, min_line_distance, false))
, density_provider(new ImageBasedDensityProvider(cross_subdisivion_spec_image_file, aabb_3d.flatten()))
, fill_pattern_for_all_layers(get_constructor, *density_provider, fractal_config.aabb.flatten(), fractal_config.depth, line_width, use_dithering)
{
}

float SierpinskiFillProvider::CombinedDensityProvider::operator()(const AABB3D& aabb) const
{
    float density = (*existing_density_provider)(aabb);
    float height_proportion = std::min(1.0f, std::max(0.0f, static_cast<float>(aabb.getMiddle().z) / total_aabb.size().z));
    float height_modified = square(density * (1.0 - height_proportion) + height_proportion) * 0.9;

    Point3 middle = Point3(total_aabb.max.x, total_aabb.min.y, total_aabb.max.z);;
    float distance_density = std::min(1.0, 1.0 / INT2MM((middle - aabb.getMiddle()).vSize() / 10));
    return (1 - distance_density) * height_modified;
}

SierpinskiFillProvider::SierpinskiFillProvider(const AABB3D aabb_3d, coord_t min_line_distance, const coord_t line_width, bool)
: fractal_config(getFractalConfig(aabb_3d, min_line_distance, true))
, density_provider(new UniformDensityProvider((float)line_width / min_line_distance))
, subdivision_structure_3d(get_constructor, *density_provider, fractal_config.aabb, fractal_config.depth, line_width)
{
    subdivision_structure_3d->initialize();
    subdivision_structure_3d->createMinimalDensityPattern();
    subdivision_structure_3d->sanitize();
    slice_walker_cross3d = subdivision_structure_3d->getSequence(/* z = */ 0);
}

SierpinskiFillProvider::SierpinskiFillProvider(const AABB3D aabb_3d, coord_t min_line_distance, const coord_t line_width, std::string cross_subdisivion_spec_image_file, bool)
: fractal_config(getFractalConfig(aabb_3d, min_line_distance, true))
, density_provider(new CombinedDensityProvider(new ImageBasedDensityProvider(cross_subdisivion_spec_image_file, aabb_3d.flatten()), aabb_3d))
, subdivision_structure_3d(get_constructor, *density_provider, fractal_config.aabb, fractal_config.depth, line_width)
{
    subdivision_structure_3d->initialize();
    subdivision_structure_3d->createMinimalDensityPattern();
    subdivision_structure_3d->sanitize();
    slice_walker_cross3d = subdivision_structure_3d->getSequence(/* z = */ 0);
}

Polygon SierpinskiFillProvider::generate(EFillMethod pattern, coord_t z, coord_t line_width, coord_t pocket_size) const
{
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
        assert(slice_walker_cross3d);
        subdivision_structure_3d->advanceSequence(*slice_walker_cross3d, z);
//         return subdivision_structure_3d->generateSierpinski(*slice_walker_cross3d);
        return subdivision_structure_3d->generateCross(*slice_walker_cross3d, z);
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

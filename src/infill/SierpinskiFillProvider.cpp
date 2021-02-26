//Copyright (C) 2017 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "ImageBasedDensityProvider.h"
#include "UniformDensityProvider.h"
#include "SierpinskiFillProvider.h"
#include "../utils/AABB3D.h"
#include "../utils/logoutput.h"
#include "../utils/math.h"
#include "../utils/polygon.h"

namespace cura
{


constexpr bool SierpinskiFillProvider::get_constructor;
constexpr bool SierpinskiFillProvider::use_dithering;

SierpinskiFillProvider::SierpinskiFillProvider(const AABB3D aabb_3d, coord_t min_line_distance, const coord_t line_width)
: fractal_config(getFractalConfig(aabb_3d, min_line_distance))
, density_provider(new UniformDensityProvider((float)line_width / min_line_distance))
, fill_pattern_for_all_layers(get_constructor, *density_provider, fractal_config.aabb, fractal_config.depth, line_width, use_dithering)
{
}

SierpinskiFillProvider::SierpinskiFillProvider(const AABB3D aabb_3d, coord_t min_line_distance, coord_t line_width, std::string cross_subdisivion_spec_image_file)
: fractal_config(getFractalConfig(aabb_3d, min_line_distance))
, density_provider(new ImageBasedDensityProvider(cross_subdisivion_spec_image_file, aabb_3d.flatten()))
, fill_pattern_for_all_layers(get_constructor, *density_provider, fractal_config.aabb, fractal_config.depth, line_width, use_dithering)
{
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

SierpinskiFillProvider::FractalConfig SierpinskiFillProvider::getFractalConfig(const AABB3D aabb_3d, coord_t min_line_distance)
{
    AABB model_aabb = aabb_3d.flatten();
    Point model_aabb_size = model_aabb.max - model_aabb.min;
    coord_t max_side_length = std::max(model_aabb_size.X, model_aabb_size.Y);
    Point model_middle = model_aabb.getMiddle();

    int depth = 0;
    coord_t aabb_size = min_line_distance;
    while (aabb_size < max_side_length)
    {
        aabb_size *= 2;
        depth += 2;
    }
    const float half_sqrt2 = .5 * sqrt2;
    if (depth > 0 && aabb_size * half_sqrt2 >= max_side_length)
    {
        aabb_size *= half_sqrt2;
        depth--;
    }

    Point radius(aabb_size / 2, aabb_size / 2);
    AABB aabb(model_middle - radius, model_middle + radius);

    return FractalConfig{depth, aabb};
}



} // namespace cura

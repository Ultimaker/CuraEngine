/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef INFILL_SIERPINSKI_FILL_PROVIDER_H
#define INFILL_SIERPINSKI_FILL_PROVIDER_H

#include "../utils/optional.h"

#include "SierpinskiFill.h"
#include "DensityProvider.h"
#include "ImageBasedDensityProvider.h"
#include "UniformDensityProvider.h"

namespace cura
{

/*!
 * TODO
 */
class SierpinskiFillProvider
{
    static constexpr bool get_constructor = true;
protected:
    struct FractalConfig
    {
        int depth;
        AABB aabb; 
    };
public:
    FractalConfig fractal_config;
    DensityProvider* density_provider;
    std::optional<SierpinskiFill> fill_pattern_for_all_layers;
    
    SierpinskiFillProvider(const AABB3D aabb_3d, coord_t min_line_distance, const coord_t line_width)
    : fractal_config(getFractalConfig(aabb_3d, min_line_distance))
    , density_provider(new UniformDensityProvider((float)line_width / min_line_distance))
    , fill_pattern_for_all_layers(get_constructor, *density_provider, fractal_config.aabb, fractal_config.depth, line_width, true) // TODO hardcoded value
    {
        
    }

    SierpinskiFillProvider(const AABB3D aabb_3d, coord_t min_line_distance, coord_t line_width, std::string cross_subdisivion_spec_image_file)
    : fractal_config(getFractalConfig(aabb_3d, min_line_distance))
    , density_provider(new ImageBasedDensityProvider(cross_subdisivion_spec_image_file, aabb_3d.getAABB()))
    , fill_pattern_for_all_layers(get_constructor, *density_provider, fractal_config.aabb, fractal_config.depth, line_width, true) // TODO hardcoded value
    {
    }

    Polygon generate(EFillMethod pattern, coord_t z, coord_t line_width) const
    {
        if (fill_pattern_for_all_layers)
        {
            if (pattern == EFillMethod::CROSS_3D)
            {
                return fill_pattern_for_all_layers->generateCross(z, line_width / 2);
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

    ~SierpinskiFillProvider()
    {
        if (density_provider)
        {
            delete density_provider;
        }
    }
protected:
    FractalConfig getFractalConfig(const AABB3D aabb_3d, coord_t min_line_distance)
    {
        AABB model_aabb = aabb_3d.getAABB();
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
        const float sqrt2 = .5 * std::sqrt(2.0);
        if (aabb_size * sqrt2 >= max_side_length)
        {
            aabb_size *= sqrt2;
            depth--;
        }
        
        Point radius(aabb_size / 2, aabb_size / 2);
        AABB aabb(model_middle - radius, model_middle + radius);
        return FractalConfig{depth, aabb};
    }
};
} // namespace cura


#endif // INFILL_SIERPINSKI_FILL_PROVIDER_H

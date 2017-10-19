/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef INFILL_SIERPINSKI_FILL_PROVIDER_H
#define INFILL_SIERPINSKI_FILL_PROVIDER_H

#include "../utils/optional.h"

#include "SierpinskiFill.h"
#include "Subdivider.h"
#include "ImageBasedSubdivider.h"
#include "UniformSubdivider.h"

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
    Subdivider* subdivider;
    std::optional<SierpinskiFill> fill_pattern_for_all_layers;
    
    SierpinskiFillProvider(const AABB3D aabb_3d, coord_t min_line_distance)
    : fractal_config(getFractalConfig(aabb_3d, min_line_distance))
    , subdivider(new UniformSubdivider())
    , fill_pattern_for_all_layers(get_constructor, *subdivider, fractal_config.aabb, fractal_config.depth)
    {
        
    }

    SierpinskiFillProvider(const AABB3D aabb_3d, coord_t min_line_distance, coord_t line_width, std::string cross_subdisivion_spec_image_file)
    : fractal_config(getFractalConfig(aabb_3d, min_line_distance))
    , subdivider(new ImageBasedSubdivider(cross_subdisivion_spec_image_file, fractal_config.aabb, line_width))
    , fill_pattern_for_all_layers(get_constructor, *subdivider, fractal_config.aabb, fractal_config.depth)
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
    }

    ~SierpinskiFillProvider()
    {
        if (subdivider)
        {
            delete subdivider;
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
        constexpr float sqrt2 = .5 * std::sqrt(2.0);
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

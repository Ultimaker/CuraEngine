// Copyright (C) 2026 Tim Kuipers
// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <optional>

#include "SierpinskiFill.h"
#include "types/EFillMethod.h" //For EFillMethod.

namespace curaengine::utils
{
struct AABB3D;
}

namespace curaengine::infill
{


class DensityProvider;

/*!
 * Class for generating infill patterns using the SierpinskiFill class.
 *
 * This class handles determining the maximum recursion depth, the initial triangle
 * and in general the configuration the SierpinskiFill class requires to be used as fill pattern.
 *
 * This class also handles the density provider which is used to determine the local density at each location - if there is one.
 */
class SierpinskiFillProvider
{
    static constexpr bool get_constructor = true;
    static constexpr bool use_dithering = true; // !< Whether to employ dithering and error propagation
protected:
    //! Basic parameters from which to start constructing the sierpinski fractal
    struct FractalConfig
    {
        int depth; //!< max recursion depth
        utils::AABB aabb; //!< The bounding box of the initial Triangles in the Sierpinski curve
    };

public:
    FractalConfig fractal_config;
    DensityProvider* density_provider; //!< The object which determines the requested density at each region
    std::optional<SierpinskiFill> fill_pattern_for_all_layers; //!< The fill pattern if one and the same pattern is used on all layers

    SierpinskiFillProvider(const utils::AABB3D aabb_3d, utils::coord_t min_line_distance, const utils::coord_t line_width);

    SierpinskiFillProvider(const utils::AABB3D aabb_3d, utils::coord_t min_line_distance, utils::coord_t line_width, std::string cross_subdisivion_spec_image_file);

    geometry::Polygon generate(types::EFillMethod pattern, utils::coord_t z, utils::coord_t line_width, utils::coord_t pocket_size) const;

    ~SierpinskiFillProvider();

protected:
    /*!
     * Get the parameters with which to generate a sierpinski fractal for this object
     */
    FractalConfig getFractalConfig(const utils::AABB3D aabb_3d, utils::coord_t min_line_distance);
};
} // namespace curaengine::infill

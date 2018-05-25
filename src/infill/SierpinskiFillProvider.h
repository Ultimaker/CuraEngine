//Copyright (C) 2017 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_SIERPINSKI_FILL_PROVIDER_H
#define INFILL_SIERPINSKI_FILL_PROVIDER_H

#include "../utils/optional.h"

#include "SierpinskiFill.h"
#include "Cross3D.h"
#include "DensityProvider.h"
#include "ImageBasedDensityProvider.h"
#include "UniformDensityProvider.h"
#include "../settings/Settings.h"

namespace cura
{

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
        AABB3D aabb; //!< The bounding box of the initial Triangles in the Sierpinski curve
    };
public:
    FractalConfig fractal_config;
    DensityProvider* density_provider; //!< The object which determines the requested density at each region
    std::optional<SierpinskiFill> fill_pattern_for_all_layers; //!< The fill pattern if one and the same pattern is used on all layers
    std::optional<Cross3D> subdivision_structure_3d; //!< The 3D prism subdivision structure from which to generate the patterns with varying density across Z
    std::optional<Cross3D::SliceWalker> slice_walker_cross3d; //!< An iterator which walks through the slices of the subdivision structure

    SierpinskiFillProvider(const AABB3D aabb_3d, coord_t min_line_distance, const coord_t line_width);

    SierpinskiFillProvider(const AABB3D aabb_3d, coord_t min_line_distance, coord_t line_width, std::string cross_subdisivion_spec_image_file);

    SierpinskiFillProvider(const AABB3D aabb_3d, coord_t min_line_distance, const coord_t line_width, bool this_constructor_is_for_cross3d);

    SierpinskiFillProvider(const AABB3D aabb_3d, coord_t min_line_distance, coord_t line_width, std::string cross_subdisivion_spec_image_file, bool this_constructor_is_for_cross3d);

    Polygon generate(EFillMethod pattern, coord_t z, coord_t line_width, coord_t pocket_size) const;

    ~SierpinskiFillProvider();
protected:
    class CombinedDensityProvider : public DensityProvider
    {
    public:
        const DensityProvider* existing_density_provider;
        AABB3D total_aabb;
        CombinedDensityProvider(const DensityProvider* rerouted, AABB3D total_aabb)
        : existing_density_provider(rerouted)
        , total_aabb(total_aabb)
        {
        };

        virtual ~CombinedDensityProvider()
        {
            delete existing_density_provider;
        };

        virtual float operator()(const AABB3D& aabb) const
        {
            float density = (*existing_density_provider)(aabb);
            float height_proportion = std::min(1.0f, std::max(0.0f, static_cast<float>(aabb.getMiddle().z) / total_aabb.size().z));
            return density * (1.0 - height_proportion) + height_proportion * 0.9;
        };
    };
    /*!
     * Get the parameters with which to generate a sierpinski fractal for this object
     * \param make_3d Whether to include z in the calculations for the 3D pattern
     */
    FractalConfig getFractalConfig(const AABB3D aabb_3d, coord_t min_line_distance, bool make_3d);
};
} // namespace cura


#endif // INFILL_SIERPINSKI_FILL_PROVIDER_H

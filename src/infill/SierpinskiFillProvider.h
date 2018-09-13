//Copyright (C) 2017 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_SIERPINSKI_FILL_PROVIDER_H
#define INFILL_SIERPINSKI_FILL_PROVIDER_H

#include "../utils/optional.h"
#include "../utils/math.h"

#include "../settings/Settings.h"

#include "SierpinskiFill.h"
#include "Cross3D.h"
#include "DensityProvider.h"
#include "ImageBasedDensityProvider.h"
#include "UniformDensityProvider.h"
#include "CombinedDensityProvider.h"
#include "TopSkinDensityProvider.h"

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
    const AABB3D aabb_3d;
    FractalConfig fractal_config;
    DensityProvider* average_density_provider; //!< The object which determines the average requested density at each region

    TopSkinDensityProvider* skin_density_provider; //!< The object which determines the minimal density based on being at the surface 
    CombinedDensityProvider* combined_density_provider; //!< The combination of the average density provider and the minimal density provider in a single density provider

    DensityProvider* density_provider; //!< The object which determines the requested density at each region

    std::optional<SierpinskiFill> fill_pattern_for_all_layers; //!< The fill pattern if one and the same pattern is used on all layers
    std::optional<Cross3D> subdivision_structure_3d; //!< The 3D prism subdivision structure from which to generate the patterns with varying density across Z
    std::map<coord_t, const Cross3D::Cell*> z_to_start_cell_cross3d; //!< Sierpinski sequence start cell for each z coord

    SierpinskiFillProvider(const SliceMeshStorage* mesh_data, const AABB3D aabb_3d, coord_t min_line_distance, const coord_t line_width, float density,
                           bool dense_at_top, float cross_infill_top_density, bool use_skin);

    SierpinskiFillProvider(const SliceMeshStorage* mesh_data, const AABB3D aabb_3d, coord_t min_line_distance, coord_t line_width,
                           std::string cross_subdisivion_spec_image_file, float min_density, float max_density, float transparency_density,
                           bool dense_at_top, float cross_infill_top_density, bool use_skin);

    SierpinskiFillProvider(const SliceMeshStorage* mesh_data, const AABB3D aabb_3d, coord_t min_line_distance, coord_t line_width,
                           std::string cross_subdisivion_spec_image_file, float min_density, float max_density, float transparency_density,
                           bool dense_at_top, float cross_infill_top_density, bool use_skin, bool this_constructor_is_for_cross3d);

    Polygon generate(EFillMethod pattern, coord_t z, coord_t line_width, coord_t pocket_size) const;

    ~SierpinskiFillProvider();
protected:
    /*!
     * Get the parameters with which to generate a sierpinski fractal for this object
     * \param make_3d Whether to include z in the calculations for the 3D pattern
     */
    FractalConfig getFractalConfig(const AABB3D aabb_3d, coord_t min_line_distance, bool make_3d);
};
} // namespace cura


#endif // INFILL_SIERPINSKI_FILL_PROVIDER_H

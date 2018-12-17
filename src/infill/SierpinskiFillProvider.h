//Copyright (C) 2017 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef INFILL_SIERPINSKI_FILL_PROVIDER_H
#define INFILL_SIERPINSKI_FILL_PROVIDER_H

#include "../utils/optional.h"
#include "../utils/math.h"
#include "../utils/MappingFunction.h"

#include "../settings/Settings.h"

#include "SierpinskiFill.h"
#include "Cross3D.h"
#include "Cross3DPrismEdgeNetwork.h"
#include "DensityProvider.h"
#include "ImageBasedDensityProvider.h"
#include "UniformDensityProvider.h"
#include "CombinedDensityProvider.h"
#include "CompensatedDensityProvider.h"
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
    MappingFunction in_out_density_correction; //!< Function mapping realistic output density to naive input density to compensate for naive density estimation
    CompensatedDensityProvider* compensated_density_provider; //!< The density provider compensated for differences between in and output density due to non-overlap and continuity alterations to the homogeneous cross pattern

    DensityProvider* density_provider; //!< The object which determines the requested density at each region

    std::optional<SierpinskiFill> fill_pattern_for_all_layers; //!< The fill pattern if one and the same pattern is used on all layers

    std::optional<Cross3D> subdivision_structure_3d; //!< The 3D prism subdivision structure from which to generate the patterns with varying density across Z
    std::map<coord_t, const Cross3D::Cell*> z_to_start_cell_cross3d; //!< Sierpinski sequence start cell for each z coord
    std::optional<Cross3DPrismEdgeNetwork> edge_network;

    SierpinskiFillProvider(const SliceMeshStorage* mesh_data, const AABB3D aabb_3d, coord_t min_line_distance, const coord_t line_width, float density,
                           bool dense_at_top, float cross_infill_top_density, bool use_skin);

    SierpinskiFillProvider(const SliceMeshStorage* mesh_data, const AABB3D aabb_3d, coord_t min_line_distance, coord_t line_width,
                           std::string cross_subdisivion_spec_image_file, float min_density, float max_density, float transparency_density,
                           bool dense_at_top, float cross_infill_top_density, bool use_skin);

    SierpinskiFillProvider(const SliceMeshStorage* mesh_data, const AABB3D aabb_3d, coord_t min_line_distance, coord_t line_width,
                           std::string cross_subdisivion_spec_image_file, float min_density, float max_density, float transparency_density,
                           bool dense_at_top, float cross_infill_top_density, bool use_skin, bool this_constructor_is_for_cross3d);

    Polygon generate(EFillMethod pattern, coord_t z, coord_t line_width, coord_t pocket_size) const;

    void generateSubdivStructureLines(EFillMethod pattern, coord_t z, coord_t line_width, Polygons& result_polygons, Polygons& result_lines, bool closed) const;

    ~SierpinskiFillProvider();
    
    double polygon_creation_time = 0;

    /*!
     * output the generated space filling surface to an stl file
     * 
     * The generated stl doesn't take overlapping lines into account, but it does ensure vertical and horizontal continuity of the surface.
     */
    void writeToSTL(const std::string filename);
protected:
    /*!
     * Get the parameters with which to generate a sierpinski fractal for this object
     * \param make_3d Whether to include z in the calculations for the 3D pattern
     */
    FractalConfig getFractalConfig(const AABB3D aabb_3d, coord_t min_line_distance, bool make_3d);
private:
    MappingFunction getMappingFunction();
};
} // namespace cura


#endif // INFILL_SIERPINSKI_FILL_PROVIDER_H

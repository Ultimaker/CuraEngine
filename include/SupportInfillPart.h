// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <optional>
#include <vector>

#include "geometry/Polygon.h"
#include "geometry/SingleShape.h"
#include "utils/AABB.h"
#include "utils/ExtrusionLine.h"


namespace curaengine
{

/*!
 * A SupportInfillPart represents an connected area for support infill on a layer.
 * The support infill areas on a layer can be isolated islands, and in this case, a SupportInfillPart represents a single island.
 *
 * This data structure is required for gradual support, which needs to partition a support area into a number of sub-areas with different density.
 * Because support is handled as a whole in the engine, that is, we have a global support areas instead of support areas for each mesh.
 * With this data structure, we can keep track of which gradual support infill areas belongs to which support area, so we can print them together.
 */
struct SupportInfillPart
{
    ::curaengine::geometry::SingleShape outline_; //!< The outline of the support infill area
    std::optional<::curaengine::geometry::Shape> base_outside_contour_; //!< Outline contour including the outer base
    std::optional<::curaengine::geometry::Shape> base_inside_contour_; //!< If an inside base is generated, contains the inner contour
    ::curaengine::utils::AABB outline_boundary_box_; //!< The boundary box for the infill area
    ::curaengine::utils::coord_t support_line_width_; //!< The support line width
    ::curaengine::utils::coord_t inset_width_to_generate_; //!< The width of insets need to be generated from the outline. This is not the actual insets that will be generated.
    std::vector<std::vector<::curaengine::geometry::Shape>> infill_area_per_combine_per_density_; //!< a list of separated sub-areas which requires different infill densities and combined thicknesses
                                                                          //   for infill_areas[x][n], x means the density level and n means the thickness
    std::vector<::curaengine::utils::VariableWidthLines> wall_toolpaths_; //!< Any walls go here, not in the areas, where they could be combined vertically (don't combine walls). Binned by inset_idx.

    ::curaengine::utils::coord_t custom_line_distance_; //!< The distance between support infill lines. 0 means use the default line distance instead.
    bool use_fractional_config_; //!< Request to use the configuration used to fill a partial layer height here, instead of the normal full layer height configuration.

    SupportInfillPart(const ::curaengine::geometry::SingleShape& outline, ::curaengine::utils::coord_t support_line_width, bool use_fractional_config, ::curaengine::utils::coord_t inset_width_to_generate = 0, ::curaengine::utils::coord_t custom_line_distance = 0);

    const ::curaengine::geometry::Shape& getInfillArea() const;
};

} // namespace curaengine

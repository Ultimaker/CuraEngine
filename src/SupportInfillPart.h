// Copyright (c) 2017 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SUPPORT_INFILL_PART_H
#define SUPPORT_INFILL_PART_H

#include <vector>

#include "utils/AABB.h"
#include "utils/polygon.h"


namespace cura
{

/*!
 * A SupportInfillPart represents an connected area for support infill on a layer.
 * The support infill areas on a layer can be isolated islands, and in this case, a SupportInfillPart represents a single island.
 *
 * This data structure is required for gradual support, which needs to partition a support area into a number of sub-areas with different density.
 * Because support is handled as a whole in the engine, that is, we have a global support areas instead of support areas for each mesh.
 * With this data structure, we can keep track of which gradual support infill areas belongs to which support area, so we can print them together.
 */
class SupportInfillPart
{
public:
    PolygonsPart outline;  //!< The outline of the support infill area
    std::vector<Polygons> insets;  //!< The insets are also known as perimeters or the walls.
    AABB outline_boundary_box;  //!< The boundary box for the infill area
    coord_t support_line_width;  //!< The support line width
    int inset_count_to_generate;  //!< The number of insets need to be generated from the outline. This is not the actual insets that will be generated.
    std::vector<std::vector<Polygons>> infill_area_per_combine_per_density;  //!< a list of separated sub-areas which requires different infill densities and combined thicknesses
                                                                              //   for infill_areas[x][n], x means the density level and n means the thickness

    SupportInfillPart(const PolygonsPart& outline, coord_t support_line_width, int inset_count_to_generate = 0);

    /*!
     * Initializes this SupportInfillPart by generating its insets and infill area.
     *
     * \return false if the area is too small and no insets and infill area can be generated, otherwise true.
     */
    bool generateInsetsAndInfillAreas(const coord_t max_resolution, const coord_t max_deviation);

    const Polygons& getInfillArea() const;

private:
    Polygons infill_area;  //!< The support infill area for generating patterns
};

inline const Polygons& SupportInfillPart::getInfillArea() const
{
    // if there is no wall, we use the original outline as the infill area
    return (inset_count_to_generate == 0) ? outline : infill_area;
}

} // namespace cura

#endif // SUPPORT_INFILL_PART_H

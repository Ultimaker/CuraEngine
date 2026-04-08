// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "geometry/MendedShape.h"

#include "utils/Simplify.h"
#include "utils/actions/smooth.h"
#include "utils/polygonUtils.h"

using namespace cura;

MendedShape::MendedShape(const Settings& settings, const SectionType section_type, const Shape& shape)
{
    // Sometimes small slivers of polygons mess up the prepared_outline. By performing an open-close operation
    // with half the minimum printable feature size or minimum line width, these slivers are removed, while still
    // keeping enough information to not degrade the print quality;
    // These features can't be printed anyhow. See PR CuraEngine#1811 for some screenshots
    const coord_t allowed_distance = settings.get<coord_t>("meshfix_maximum_deviation");
    const coord_t open_close_distance
        = settings.get<bool>("fill_outline_gaps") ? settings.get<coord_t>("min_feature_size") / 2 - 5 : settings.get<coord_t>("min_wall_line_width") / 2 - 5;
    const coord_t epsilon_offset = (allowed_distance / 2) - 1;
    const double small_area_length = settings.get<double>("wall_line_width_0") / 2.0;

    // Simplify outline for boost::voronoi consumption. Absolutely no self intersections or near-self intersections allowed:
    // TODO: Open question: Does this indeed fix all (or all-but-one-in-a-million) cases for manifold but otherwise possibly complex polygons?
    shape_ = shape.offset(-open_close_distance).offset(open_close_distance * 2).offset(-open_close_distance);
    shape_.removeSmallAreas(small_area_length * small_area_length, false);
    shape_ = Simplify(settings).polygon(shape_);
    if (settings.get<bool>("meshfix_fluid_motion_enabled") && section_type != SectionType::SUPPORT)
    {
        // No need to smooth support walls
        auto smoother = actions::smooth(settings);
        for (Polygon& polygon : shape_)
        {
            polygon.setPoints(smoother(polygon.getPoints()));
        }
    }

    PolygonUtils::fixSelfIntersections(epsilon_offset, shape_);
    shape_.removeDegenerateVerts();
    shape_.removeColinearEdges(AngleRadians(0.005));
    // Removing collinear edges may introduce self intersections, so we need to fix them again
    PolygonUtils::fixSelfIntersections(epsilon_offset, shape_);
    shape_.removeDegenerateVerts();
    shape_ = shape_.unionPolygons();
    shape_ = Simplify(settings).polygon(shape_).removeNearSelfIntersections();

    // NOTE: It's somewhat unclear that 'removeNearSelfIntersections()' (as opposed to 'fixSelfIntersections') is still needed.
    //       Or, if it _is_ needed/useful, that the 'last call' is the best place for it.
    //       I'm _definitely_ not going to attempt to remove it just before a release though.
}

const Shape& MendedShape::getShape() const
{
    return shape_;
}

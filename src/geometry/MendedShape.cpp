// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "geometry/MendedShape.h"

#include "utils/Simplify.h"
#include "utils/actions/smooth.h"
#include "utils/polygonUtils.h"

using namespace cura;

MendedShape::MendedShape(const Settings* settings, const SectionType section_type, const Shape* shape)
    : settings_(settings)
    , section_type_(section_type)
    , shape_(shape)
{
}

const Shape& MendedShape::getShape() const
{
    if (! mended_shape_.has_value())
    {
        Shape mended_shape;
        if (settings_ != nullptr && shape_ != nullptr)
        {
            // Sometimes small slivers of polygons mess up the prepared_outline. By performing an open-close operation
            // with half the minimum printable feature size or minimum line width, these slivers are removed, while still
            // keeping enough information to not degrade the print quality;
            // These features can't be printed anyhow. See PR CuraEngine#1811 for some screenshots
            const coord_t allowed_distance = settings_->get<coord_t>("meshfix_maximum_deviation");
            const coord_t open_close_distance
                = settings_->get<bool>("fill_outline_gaps") ? settings_->get<coord_t>("min_feature_size") / 2 - 5 : settings_->get<coord_t>("min_wall_line_width") / 2 - 5;
            const coord_t epsilon_offset = (allowed_distance / 2) - 1;
            const double small_area_length = settings_->get<double>("wall_line_width_0") / 2.0;

            // Simplify outline for boost::voronoi consumption. Absolutely no self intersections or near-self intersections allowed:
            // TODO: Open question: Does this indeed fix all (or all-but-one-in-a-million) cases for manifold but otherwise possibly complex polygons?
            mended_shape = shape_->offset(-open_close_distance).offset(open_close_distance * 2).offset(-open_close_distance);
            mended_shape.removeSmallAreas(small_area_length * small_area_length, false);
            mended_shape = Simplify(*settings_).polygon(mended_shape);
            if (settings_->get<bool>("meshfix_fluid_motion_enabled") && section_type_ != SectionType::SUPPORT)
            {
                // No need to smooth support walls
                auto smoother = actions::smooth(*settings_);
                for (Polygon& polygon : mended_shape)
                {
                    polygon.setPoints(smoother(polygon.getPoints()));
                }
            }

            PolygonUtils::fixSelfIntersections(epsilon_offset, mended_shape);
            mended_shape.removeDegenerateVerts();
            mended_shape.removeColinearEdges(AngleRadians(0.005));
            // Removing collinear edges may introduce self intersections, so we need to fix them again
            PolygonUtils::fixSelfIntersections(epsilon_offset, mended_shape);
            mended_shape.removeDegenerateVerts();
            mended_shape = mended_shape.unionPolygons();
            mended_shape = Simplify(*settings_).polygon(mended_shape).removeNearSelfIntersections();

            // NOTE: It's somewhat unclear that 'removeNearSelfIntersections()' (as opposed to 'fixSelfIntersections') is still needed.
            //       Or, if it _is_ needed/useful, that the 'last call' is the best place for it.
            //       I'm _definitely_ not going to attempt to remove it just before a release though.
        }

        const_cast<MendedShape*>(this)->mended_shape_ = mended_shape;
    }

    return *mended_shape_;
}

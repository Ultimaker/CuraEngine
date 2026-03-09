// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_MENDEDSHAPE_H
#define GEOMETRY_MENDEDSHAPE_H

#include "geometry/Shape.h"
#include "settings/Settings.h"
#include "utils/section_type.h"

namespace cura
{
/*!
 * \brief A MendedShape is a wrapper class for a Shape,
 *   so we can't forget to 'prepare' the shape for those algorithm(s) that need that.
 *
 * Some algorithms (well, just one at the moment, SkeletalTrapezoidation) needs polygons that
 * - don't (near) self-intersect,
 * - don't have any colinear segments,
 * - don't have any degenerate vertices,
 * - ... etc.
 * When this is forgotten, it can _sometimes_ produce crashes, especially in more complicated models;
 * since this doesn't always happen (or even in the majority of cases), it can slip by our QA process.
 * This class will make it so we can't forget that anymore, since the typesystem will remind people.
 *
 * (Previously basically all of its code was in WallToolPaths instead.)
 */
class MendedShape
{
public:
    MendedShape()
        : shape_(Shape())
    {
    }
    MendedShape(const Settings& settings, const SectionType section_type, const Shape& shape);
    const Shape& getShape() const;

protected:
    Shape shape_;
};
} // namespace cura

#endif // GEOMETRY_MENDEDSHAPE_H

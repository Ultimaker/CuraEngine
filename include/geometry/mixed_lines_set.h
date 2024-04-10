// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_MIXED_LINES_SET_H
#define GEOMETRY_MIXED_LINES_SET_H

#include <memory>

#include "utils/Coord_t.h"

namespace cura
{

class Polyline;
class OpenPolyline;
class ClosedPolyline;
class Polygon;
class Shape;
template<class LineType>
class LinesSet;


/*!
 * \brief Convenience definition for a container that can hold either open or closed polylines.
 */
class MixedLinesSet : public std::vector<std::shared_ptr<Polyline>>
{
public:
    Shape offset(coord_t distance, ClipperLib::JoinType join_type = ClipperLib::jtMiter, double miter_limit = 1.2) const;

    void push_back(const OpenPolyline& line);

    void push_back(OpenPolyline&& line);

    void push_back(ClosedPolyline&& line);

    void push_back(const Polygon& line);

    void push_back(const std::shared_ptr<OpenPolyline>& line);

    void push_back(const std::shared_ptr<Polyline>& line);

    void push_back(LinesSet<OpenPolyline>&& lines_set);

    void push_back(const LinesSet<OpenPolyline>& lines_set);

    void push_back(LinesSet<ClosedPolyline>&& lines_set);

    void push_back(const LinesSet<Polygon>& lines_set);

    void push_back(const Shape& shape);

    coord_t length() const;
};

} // namespace cura

#endif // GEOMETRY_MIXED_LINES_SET_H

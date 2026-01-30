// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_OPEN_LINES_SET_H
#define GEOMETRY_OPEN_LINES_SET_H

#include "geometry/LinesSet.h"
#include "geometry/OpenPolyline.h"

namespace cura
{

/*!
 * \brief Container that can hold only open polylines. This makes it explicit what the lines actually represent and adds some processing functions that can only be applied
 * to open polylines.
 */
class OpenLinesSet : public LinesSet<OpenPolyline>
{
public:
    OpenLinesSet() = default;

    OpenLinesSet(LinesSet<OpenPolyline>&& other)
        : LinesSet(std::move(other))
    {
    }

    explicit OpenLinesSet(const std::initializer_list<OpenPolyline>& initializer)
        : LinesSet(initializer)
    {
    }

    explicit OpenLinesSet(const OpenPolyline& line)
        : LinesSet(line)
    {
    }

    /*! \brief Constructor that takes ownership of the data from the given set of lines */
    explicit OpenLinesSet(ClipperLib::Paths&& paths);

    /*! \brief Add a simple line consisting of two points */
    void addSegment(const Point2LL& from, const Point2LL& to);

    /*!
     * Split the given line in two parts, on the given point
     * @param line_index The index of the line to be split
     * @param point_index The point at which the line will be split
     * @note This is implemented only for containers of OpenPolyline, since splitting a closed polyline has a very different meaning
     */
    void split(const size_t line_index, const size_t point_index);
};

} // namespace cura

#endif // GEOMETRY_OPEN_LINES_SET_H

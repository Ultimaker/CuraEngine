// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_MIXED_LINES_SET_H
#define GEOMETRY_MIXED_LINES_SET_H

#include "geometry/lines_set.h"
#include "geometry/polyline.h"

namespace cura
{

using MixedLinesSet = LinesSet<Polyline>;

#if 0
/*!
 * \brief Container that can hold either open or closed polylines. We often have to handle "a bunch
 *        of lines" which are either open or closed without taking care of what they actually are.
 */
class MixedLinesSet : public LinesSet<Polyline>
{
public:
    MixedLinesSet() = default;

    MixedLinesSet(const MixedLinesSet& other) = default;

    MixedLinesSet(MixedLinesSet&& other) = default;

    MixedLinesSet(const LinesSet<OpenPolyline>& open_lines)
        : open_lines_(open_lines)
    {
    }

    MixedLinesSet(LinesSet<OpenPolyline>&& open_lines)
        : open_lines_(std::move(open_lines))
    {
    }

    MixedLinesSet(const LinesSet<ClosedPolyline>& closed_lines)
        : closed_lines_(closed_lines)
    {
    }

    MixedLinesSet(LinesSet<ClosedPolyline>&& closed_lines)
        : closed_lines_(std::move(closed_lines))
    {
    }

    const LinesSet<OpenPolyline>& getOpenLines() const
    {
        return open_lines_;
    }

    LinesSet<OpenPolyline>& getOpenLines()
    {
        return open_lines_;
    }

    void setOpenLines(const LinesSet<OpenPolyline>& open_lines)
    {
        open_lines_ = open_lines;
    }

    void setOpenLines(LinesSet<OpenPolyline>&& open_lines)
    {
        open_lines_ = std::move(open_lines);
    }

    const LinesSet<ClosedPolyline>& getClosedLines() const
    {
        return closed_lines_;
    }

    LinesSet<ClosedPolyline>& getClosedLines()
    {
        return closed_lines_;
    }

    void setClosedLines(const LinesSet<ClosedPolyline>& closed_lines)
    {
        closed_lines_ = closed_lines;
    }

    void setClosedLines(LinesSet<ClosedPolyline>&& closed_lines)
    {
        closed_lines_ = std::move(closed_lines);
    }

    ClosedPolyline& push_back(const ClosedPolyline& line)
    {
        closed_lines_.push_back(line);
        return closed_lines_.back();
    }

    void push_back(const LinesSet<ClosedPolyline>& lines)
    {
        closed_lines_.add(lines);
    }

    OpenPolyline& push_back(const OpenPolyline& line)
    {
        open_lines_.push_back(line);
        return open_lines_.back();
    }

    void push_back(const LinesSet<OpenPolyline>& lines)
    {
        open_lines_.add(lines);
    }

    void push_back(const MixedLinesSet& lines);

    coord_t length() const
    {
        return open_lines_.length() + closed_lines_.length();
    }

    size_t size() const
    {
        return open_lines_.size() + closed_lines_.size();
    }
};
#endif
} // namespace cura

#endif // GEOMETRY_MIXED_LINES_SET_H

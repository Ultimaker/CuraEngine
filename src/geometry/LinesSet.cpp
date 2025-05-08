// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/LinesSet.h"

#include <cassert>
#include <numeric>

#include "geometry/ClosedLinesSet.h"
#include "geometry/OpenLinesSet.h"
#include "geometry/OpenPolyline.h"
#include "geometry/Polygon.h"
#include "geometry/Shape.h"

namespace cura
{

template<class LineType>
bool LinesSet<LineType>::checkAdd(const LineType& line, CheckNonEmptyParam check_non_empty)
{
    switch (check_non_empty)
    {
    case CheckNonEmptyParam::EvenIfEmpty:
        return true;
    case CheckNonEmptyParam::OnlyIfNotEmpty:
        return ! line.empty();
    case CheckNonEmptyParam::OnlyIfValid:
        return line.isValid();
    }

    return false;
}

template<class LineType>
void LinesSet<LineType>::push_back(const LineType& line, CheckNonEmptyParam check_non_empty)
{
    if (checkAdd(line, check_non_empty))
    {
        lines_.push_back(line);
    }
}

template<class LineType>
void LinesSet<LineType>::push_back(LineType&& line, CheckNonEmptyParam check_non_empty)
{
    if (checkAdd(line, check_non_empty))
    {
        lines_.push_back(std::move(line));
    }
}

template<class LineType>
template<class OtherLineType>
void LinesSet<LineType>::push_back(LinesSet<OtherLineType>&& lines_set)
{
    reserve(size() + lines_set.size());
    for (OtherLineType& line : lines_set)
    {
        emplace_back(std::move(line));
    }
}

template<class LineType>
size_t LinesSet<LineType>::pointCount() const
{
    return std::accumulate(
        lines_.begin(),
        lines_.end(),
        0ULL,
        [](size_t total, const LineType& line)
        {
            return total + line.size();
        });
}

template<>
void OpenLinesSet::addSegment(const Point2LL& from, const Point2LL& to)
{
    lines_.emplace_back(std::initializer_list<Point2LL>{ from, to });
}

template<class LineType>
void LinesSet<LineType>::removeAt(size_t index)
{
    if (lines_.size() == 1)
    {
        lines_.clear();
    }
    else if (lines_.size() > 1)
    {
        assert(index < lines_.size());
        if (index < lines_.size() - 1)
        {
            lines_[index] = std::move(lines_.back());
        }
        lines_.resize(lines_.size() - 1);
    }
}

template<class LineType>
void LinesSet<LineType>::splitIntoSegments(OpenLinesSet& result) const
{
    for (const LineType& line : lines_)
    {
        line.splitIntoSegments(result);
    }
}

template<class LineType>
OpenLinesSet LinesSet<LineType>::splitIntoSegments() const
{
    OpenLinesSet result;
    for (const LineType& line : lines_)
    {
        line.splitIntoSegments(result);
    }
    return result;
}

template<class LineType>
coord_t LinesSet<LineType>::length() const
{
    return std::accumulate(
        lines_.begin(),
        lines_.end(),
        0LL,
        [](coord_t total, const LineType& line)
        {
            return total += line.length();
        });
}

template<class LineType>
Shape LinesSet<LineType>::createTubeShape(const coord_t inner_offset, const coord_t outer_offset) const
{
    return offset(outer_offset).difference(offset(-inner_offset));
}

template<class LineType>
void LinesSet<LineType>::translate(const Point2LL& delta)
{
    if (delta.X != 0 || delta.Y != 0)
    {
        for (LineType& line : getLines())
        {
            line.translate(delta);
        }
    }
}

template<>
Shape LinesSet<ClosedPolyline>::offset(coord_t distance, ClipperLib::JoinType join_type, double miter_limit) const
{
    if (empty())
    {
        return {};
    }
    if (distance == 0)
    {
        Shape result;
        for (const ClosedPolyline& line : getLines())
        {
            result.emplace_back(line.getPoints(), line.isExplicitelyClosed());
        }
        return result;
    }
    ClipperLib::Paths ret;
    ClipperLib::ClipperOffset clipper(miter_limit, 10.0);
    addPaths(clipper, join_type, ClipperLib::etClosedLine);
    clipper.MiterLimit = miter_limit;
    clipper.Execute(ret, static_cast<double>(distance));
    return Shape{ std::move(ret) };
}

template<>
Shape LinesSet<Polygon>::offset(coord_t distance, ClipperLib::JoinType join_type, double miter_limit) const
{
    if (empty())
    {
        return {};
    }
    if (distance == 0)
    {
        return { getLines() };
    }
    ClipperLib::Paths ret;
    ClipperLib::ClipperOffset clipper(miter_limit, 10.0);
    Shape(getLines()).unionPolygons().addPaths(clipper, join_type, ClipperLib::etClosedPolygon);
    clipper.MiterLimit = miter_limit;
    clipper.Execute(ret, static_cast<double>(distance));
    return Shape{ std::move(ret) };
}

template<>
Shape OpenLinesSet::offset(coord_t distance, ClipperLib::JoinType join_type, double miter_limit) const
{
    if (empty() || distance == 0)
    {
        return {};
    }

    ClipperLib::ClipperOffset clipper(miter_limit, 10.0);
    const ClipperLib::EndType end_type{ join_type == ClipperLib::jtMiter ? ClipperLib::etOpenSquare : ClipperLib::etOpenRound };
    addPaths(clipper, join_type, end_type);
    clipper.MiterLimit = miter_limit;
    ClipperLib::Paths result_paths;
    clipper.Execute(result_paths, static_cast<double>(distance));

    return Shape{ std::move(result_paths) };
}

template<class LineType>
void LinesSet<LineType>::removeDegenerateVerts()
{
    for (size_t poly_idx = 0; poly_idx < lines_.size(); poly_idx++)
    {
        LineType& poly = lines_[poly_idx];
        const bool for_polyline = (dynamic_cast<OpenPolyline*>(&poly) != nullptr);
        ClipperLib::Path result;

        auto is_degenerate = [](const Point2LL& last, const Point2LL& now, const Point2LL& next)
        {
            Point2LL last_line = now - last;
            Point2LL next_line = next - now;
            return dot(last_line, next_line) == -1 * vSize(last_line) * vSize(next_line);
        };

        // With polylines, skip the first and last vertex.
        const size_t start_vertex = for_polyline ? 1 : 0;
        const size_t end_vertex = for_polyline ? poly.size() - 1 : poly.size();
        for (size_t i = 0; i < start_vertex; ++i)
        {
            result.push_back(poly[i]); // Add everything before the start vertex.
        }

        bool is_changed = false;
        for (size_t idx = start_vertex; idx < end_vertex; idx++)
        {
            const Point2LL& last = (result.size() == 0) ? poly.back() : result.back();
            if (idx + 1 >= poly.size() && result.size() == 0)
            {
                break;
            }
            const Point2LL& next = (idx + 1 >= poly.size()) ? result[0] : poly[idx + 1];
            if (is_degenerate(last, poly[idx], next))
            { // lines are in the opposite direction
                // don't add vert to the result
                is_changed = true;
                while (result.size() > 1 && is_degenerate(result[result.size() - 2], result.back(), next))
                {
                    result.pop_back();
                }
            }
            else
            {
                result.push_back(poly[idx]);
            }
        }

        for (size_t i = end_vertex; i < poly.size(); ++i)
        {
            result.push_back(poly[i]); // Add everything after the end vertex.
        }

        if (is_changed)
        {
            if (for_polyline || result.size() > 2)
            {
                poly.setPoints(std::move(result));
            }
            else
            {
                removeAt(poly_idx);
                poly_idx--; // effectively the next iteration has the same poly_idx (referring to a new poly which is not yet processed)
            }
        }
    }
}

template<class LineType>
template<class OtherLineLine>
void LinesSet<LineType>::addPath(ClipperLib::Clipper& clipper, const OtherLineLine& line, ClipperLib::PolyType poly_typ) const
{
    // In this context, the "Closed" argument means "Is a surface" so it should be only
    // true for actual filled polygons. Closed polylines are to be treated as lines here.
    if constexpr (std::is_same<OtherLineLine, Polygon>::value)
    {
        clipper.AddPath(line.getPoints(), poly_typ, true);
    }
    else
    {
        clipper.AddPath(line.getPoints(), poly_typ, false);
    }
}

template<class LineType>
void LinesSet<LineType>::addPaths(ClipperLib::Clipper& clipper, ClipperLib::PolyType poly_typ) const
{
    for (const LineType& line : getLines())
    {
        addPath(clipper, line, poly_typ);
    }
}

template<class LineType>
void LinesSet<LineType>::addPaths(ClipperLib::ClipperOffset& clipper, ClipperLib::JoinType joint_type, ClipperLib::EndType endType) const
{
    for (const LineType& line : getLines())
    {
        clipper.AddPath(line.getPoints(), joint_type, endType);
    }
}

template size_t OpenLinesSet::pointCount() const;
template void OpenLinesSet::removeAt(size_t index);
template void OpenLinesSet::splitIntoSegments(OpenLinesSet& result) const;
template OpenLinesSet OpenLinesSet::splitIntoSegments() const;
template coord_t OpenLinesSet::length() const;
template Shape OpenLinesSet::createTubeShape(const coord_t inner_offset, const coord_t outer_offset) const;
template void OpenLinesSet::translate(const Point2LL& delta);
template void OpenLinesSet::removeDegenerateVerts();
template void OpenLinesSet::addPaths(ClipperLib::Clipper& clipper, ClipperLib::PolyType PolyTyp) const;
template void OpenLinesSet::addPaths(ClipperLib::ClipperOffset& clipper, ClipperLib::JoinType jointType, ClipperLib::EndType endType) const;
template void OpenLinesSet::push_back(const OpenPolyline& line, CheckNonEmptyParam checkNonEmpty);
template void OpenLinesSet::push_back(OpenPolyline&& line, CheckNonEmptyParam checkNonEmpty);
template void OpenLinesSet::push_back(OpenLinesSet&& lines_set);

template size_t ClosedLinesSet::pointCount() const;
template void ClosedLinesSet::removeAt(size_t index);
template void ClosedLinesSet::splitIntoSegments(OpenLinesSet& result) const;
template OpenLinesSet ClosedLinesSet::splitIntoSegments() const;
template coord_t ClosedLinesSet::length() const;
template Shape ClosedLinesSet::createTubeShape(const coord_t inner_offset, const coord_t outer_offset) const;
template void ClosedLinesSet::translate(const Point2LL& delta);
template void ClosedLinesSet::removeDegenerateVerts();
template void ClosedLinesSet::addPaths(ClipperLib::Clipper& clipper, ClipperLib::PolyType PolyTyp) const;
template void ClosedLinesSet::addPaths(ClipperLib::ClipperOffset& clipper, ClipperLib::JoinType jointType, ClipperLib::EndType endType) const;
template void ClosedLinesSet::push_back(const ClosedPolyline& line, CheckNonEmptyParam checkNonEmpty);
template void ClosedLinesSet::push_back(ClosedPolyline&& line, CheckNonEmptyParam checkNonEmpty);
template void ClosedLinesSet::push_back(ClosedLinesSet&& lines_set);
template void ClosedLinesSet::push_back(LinesSet<Polygon>&& lines_set);

template size_t LinesSet<Polygon>::pointCount() const;
template void LinesSet<Polygon>::removeAt(size_t index);
template void LinesSet<Polygon>::splitIntoSegments(OpenLinesSet& result) const;
template OpenLinesSet LinesSet<Polygon>::splitIntoSegments() const;
template coord_t LinesSet<Polygon>::length() const;
template Shape LinesSet<Polygon>::createTubeShape(const coord_t inner_offset, const coord_t outer_offset) const;
template void LinesSet<Polygon>::translate(const Point2LL& delta);
template void LinesSet<Polygon>::removeDegenerateVerts();
template void LinesSet<Polygon>::addPaths(ClipperLib::Clipper& clipper, ClipperLib::PolyType PolyTyp) const;
template void LinesSet<Polygon>::addPaths(ClipperLib::ClipperOffset& clipper, ClipperLib::JoinType jointType, ClipperLib::EndType endType) const;
template void LinesSet<Polygon>::push_back(const Polygon& line, CheckNonEmptyParam checkNonEmpty);
template void LinesSet<Polygon>::push_back(Polygon&& line, CheckNonEmptyParam checkNonEmpty);
template void LinesSet<Polygon>::push_back(LinesSet<Polygon>&& lines_set);
template void LinesSet<Polygon>::addPath(ClipperLib::Clipper& clipper, const Polygon& line, ClipperLib::PolyType poly_typ) const;

} // namespace cura

// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/LinesSet.h"

#include <numeric>

#include "geometry/OpenLinesSet.h"
#include "geometry/Polygon.h"
#include "geometry/Shape.h"

namespace cura
{

template<>
template<>
LinesSet<OpenPolyline>::LinesSet<OpenPolyline>(ClipperLib::Paths&& paths)
{
    reserve(paths.size());
    for (ClipperLib::Path& path : paths)
    {
        lines_.emplace_back(std::move(path));
    }
}

template<class LineType>
void LinesSet<LineType>::push_back(const LineType& line, bool checkNonEmpty)
{
    if (! checkNonEmpty || ! line.empty())
    {
        lines_.push_back(line);
    }
}

template<class LineType>
void LinesSet<LineType>::push_back(LineType&& line, bool checkNonEmpty)
{
    if (! checkNonEmpty || ! line.empty())
    {
        lines_.push_back(line);
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
        size_t(0),
        [](size_t total, const LineType& line)
        {
            return total + line.size();
        });
}

template<>
void LinesSet<OpenPolyline>::addSegment(const Point2LL& from, const Point2LL& to)
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
        0,
        [](coord_t total, const LineType& line)
        {
            return total += line.length();
        });
}

template<class LineType>
Shape LinesSet<LineType>::tubeShape(const coord_t inner_offset, const coord_t outer_offset) const
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
        return Shape();
    }
    else if (distance == 0)
    {
        Shape result;
        for (const ClosedPolyline& line : getLines())
        {
            result.emplace_back(line.getPoints(), line.isExplicitelyClosed());
        }
        return result;
    }
    else
    {
        ClipperLib::Paths ret;
        ClipperLib::ClipperOffset clipper(miter_limit, 10.0);
        addPaths(clipper, join_type, ClipperLib::etClosedLine);
        clipper.MiterLimit = miter_limit;
        clipper.Execute(ret, static_cast<double>(distance));
        return Shape(std::move(ret));
    }
}

template<>
Shape LinesSet<Polygon>::offset(coord_t distance, ClipperLib::JoinType join_type, double miter_limit) const
{
    if (empty())
    {
        return Shape();
    }
    else if (distance == 0)
    {
        return Shape(getLines());
    }
    else
    {
        ClipperLib::Paths ret;
        ClipperLib::ClipperOffset clipper(miter_limit, 10.0);
        Shape(getLines()).unionPolygons().addPaths(clipper, join_type, ClipperLib::etClosedPolygon);
        clipper.MiterLimit = miter_limit;
        clipper.Execute(ret, static_cast<double>(distance));
        return Shape(std::move(ret));
    }
}

template<>
Shape LinesSet<OpenPolyline>::offset(coord_t distance, ClipperLib::JoinType join_type, double miter_limit) const
{
    if (empty() || distance == 0)
    {
        return Shape();
    }
    else
    {
        Shape result;

        ClipperLib::ClipperOffset clipper(miter_limit, 10.0);
        ClipperLib::EndType end_type;
        if (join_type == ClipperLib::jtMiter)
        {
            end_type = ClipperLib::etOpenSquare;
        }
        else
        {
            end_type = ClipperLib::etOpenRound;
        }

        addPaths(clipper, join_type, end_type);

        clipper.MiterLimit = miter_limit;
        ClipperLib::Paths result_paths;
        clipper.Execute(result_paths, static_cast<double>(distance));
        result = Shape(std::move(result_paths));

        return result;
    }
}

template<class LineType>
void LinesSet<LineType>::removeDegenerateVerts()
{
    for (size_t poly_idx = 0; poly_idx < lines_.size(); poly_idx++)
    {
        LineType& poly = lines_[poly_idx];
        const bool for_polyline = (dynamic_cast<OpenPolyline*>(&poly) != nullptr);
        ClipperLib::Path result;

        auto isDegenerate = [](const Point2LL& last, const Point2LL& now, const Point2LL& next)
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

        bool isChanged = false;
        for (size_t idx = start_vertex; idx < end_vertex; idx++)
        {
            const Point2LL& last = (result.size() == 0) ? poly.back() : result.back();
            if (idx + 1 >= poly.size() && result.size() == 0)
            {
                break;
            }
            const Point2LL& next = (idx + 1 >= poly.size()) ? result[0] : poly[idx + 1];
            if (isDegenerate(last, poly[idx], next))
            { // lines are in the opposite direction
                // don't add vert to the result
                isChanged = true;
                while (result.size() > 1 && isDegenerate(result[result.size() - 2], result.back(), next))
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

        if (isChanged)
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
void LinesSet<LineType>::addPaths(ClipperLib::Clipper& clipper, ClipperLib::PolyType PolyTyp) const
{
    for (const LineType& line : getLines())
    {
        // In this context, the "Closed" argument means "Is a surface" so it should be only
        // true for actual filled polygons. Closed polylines are to be treated as lines here.
        if constexpr (std::is_same<LineType, Polygon>::value)
        {
            clipper.AddPath(line.getPoints(), PolyTyp, true);
        }
        else
        {
            clipper.AddPath(line.getPoints(), PolyTyp, false);
        }
    }
}

template<class LineType>
void LinesSet<LineType>::addPaths(ClipperLib::ClipperOffset& clipper, ClipperLib::JoinType jointType, ClipperLib::EndType endType) const
{
    for (const LineType& line : getLines())
    {
        clipper.AddPath(line.getPoints(), jointType, endType);
    }
}

template size_t LinesSet<OpenPolyline>::pointCount() const;
template void LinesSet<OpenPolyline>::removeAt(size_t index);
template void LinesSet<OpenPolyline>::splitIntoSegments(OpenLinesSet& result) const;
template OpenLinesSet LinesSet<OpenPolyline>::splitIntoSegments() const;
template coord_t LinesSet<OpenPolyline>::length() const;
template Shape LinesSet<OpenPolyline>::tubeShape(const coord_t inner_offset, const coord_t outer_offset) const;
template void LinesSet<OpenPolyline>::translate(const Point2LL& delta);
template void LinesSet<OpenPolyline>::removeDegenerateVerts();
template void LinesSet<OpenPolyline>::addPaths(ClipperLib::Clipper& clipper, ClipperLib::PolyType PolyTyp) const;
template void LinesSet<OpenPolyline>::addPaths(ClipperLib::ClipperOffset& clipper, ClipperLib::JoinType jointType, ClipperLib::EndType endType) const;
template void LinesSet<OpenPolyline>::push_back(const OpenPolyline& line, bool checkNonEmpty);
template void LinesSet<OpenPolyline>::push_back(OpenPolyline&& line, bool checkNonEmpty);
template void LinesSet<OpenPolyline>::push_back(LinesSet<OpenPolyline>&& lines_set);

template size_t LinesSet<ClosedPolyline>::pointCount() const;
template void LinesSet<ClosedPolyline>::removeAt(size_t index);
template void LinesSet<ClosedPolyline>::splitIntoSegments(OpenLinesSet& result) const;
template OpenLinesSet LinesSet<ClosedPolyline>::splitIntoSegments() const;
template coord_t LinesSet<ClosedPolyline>::length() const;
template Shape LinesSet<ClosedPolyline>::tubeShape(const coord_t inner_offset, const coord_t outer_offset) const;
template void LinesSet<ClosedPolyline>::translate(const Point2LL& delta);
template void LinesSet<ClosedPolyline>::removeDegenerateVerts();
template void LinesSet<ClosedPolyline>::addPaths(ClipperLib::Clipper& clipper, ClipperLib::PolyType PolyTyp) const;
template void LinesSet<ClosedPolyline>::addPaths(ClipperLib::ClipperOffset& clipper, ClipperLib::JoinType jointType, ClipperLib::EndType endType) const;
template void LinesSet<ClosedPolyline>::push_back(const ClosedPolyline& line, bool checkNonEmpty);
template void LinesSet<ClosedPolyline>::push_back(ClosedPolyline&& line, bool checkNonEmpty);
template void LinesSet<ClosedPolyline>::push_back(LinesSet<ClosedPolyline>&& lines_set);
template void LinesSet<ClosedPolyline>::push_back(LinesSet<Polygon>&& lines_set);

template size_t LinesSet<Polygon>::pointCount() const;
template void LinesSet<Polygon>::removeAt(size_t index);
template void LinesSet<Polygon>::splitIntoSegments(OpenLinesSet& result) const;
template OpenLinesSet LinesSet<Polygon>::splitIntoSegments() const;
template coord_t LinesSet<Polygon>::length() const;
template Shape LinesSet<Polygon>::tubeShape(const coord_t inner_offset, const coord_t outer_offset) const;
template void LinesSet<Polygon>::translate(const Point2LL& delta);
template void LinesSet<Polygon>::removeDegenerateVerts();
template void LinesSet<Polygon>::addPaths(ClipperLib::Clipper& clipper, ClipperLib::PolyType PolyTyp) const;
template void LinesSet<Polygon>::addPaths(ClipperLib::ClipperOffset& clipper, ClipperLib::JoinType jointType, ClipperLib::EndType endType) const;
template void LinesSet<Polygon>::push_back(const Polygon& line, bool checkNonEmpty);
template void LinesSet<Polygon>::push_back(Polygon&& line, bool checkNonEmpty);
template void LinesSet<Polygon>::push_back(LinesSet<Polygon>&& lines_set);

} // namespace cura

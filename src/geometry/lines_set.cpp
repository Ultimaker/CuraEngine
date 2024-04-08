// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/lines_set.h"

#include <numeric>

#include "geometry/polygon.h"
#include "geometry/polyline.h"
#include "geometry/shape.h"

namespace cura
{

template<class LineType>
LinesSet<LineType>::LinesSet(PolylineType type, std::vector<std::vector<Point2LL>>&& paths)
{
    reserve(paths.size());
    for (std::vector<Point2LL>& path : paths)
    {
        push_back(type, std::move(path));
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
void LinesSet<LineType>::push_back(PolylineType type, ClipperLib::Paths&& paths)
{
    reserve(size() + paths.size());
    for (ClipperLib::Path& path : paths)
    {
        lines_.emplace_back(type, std::move(path));
    }
}

template<class LineType>
void LinesSet<LineType>::push_back(LinesSet<LineType>&& lines_set)
{
    reserve(size() + lines_set.size());
    for (LineType& line : lines_set)
    {
        push_back(std::move(line));
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

template<class LineType>
void LinesSet<LineType>::addLine(const Point2LL& from, const Point2LL& to)
{
    lines_.emplace_back(PolylineType::Open, std::initializer_list<Point2LL>{ from, to });
}

template<class LineType>
void LinesSet<LineType>::addIfNotEmpty(const LineType& line)
{
    if (! line.empty())
    {
        lines_.push_back(line);
    }
}

template<class LineType>
void LinesSet<LineType>::addIfNotEmpty(LineType&& line)
{
    if (! line.empty())
    {
        lines_.emplace_back(std::move(line));
    }
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
void LinesSet<LineType>::splitIntoSegments(LinesSet<Polyline>& result) const
{
    for (const LineType& line : lines_)
    {
        line.splitIntoSegments(result);
    }
}

template<class LineType>
LinesSet<Polyline> LinesSet<LineType>::splitIntoSegments() const
{
    LinesSet<Polyline> result;
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
Shape LinesSet<LineType>::offset(coord_t distance, ClipperLib::JoinType joinType, double miter_limit) const
{
    if (distance == 0)
    {
        // Return a shape that contains only actual polygons
        Shape result;

        for (const LineType& line : lines_)
        {
            if (const Polygon* polygon = dynamic_cast<const Polygon*>(&line))
            {
                result.push_back(*polygon);
            }
        }

        return result;
    }
    else
    {
        Shape polygons;
        ClipperLib::ClipperOffset clipper(miter_limit, 10.0);

        for (const LineType& line : lines_)
        {
            if (const Polygon* polygon = dynamic_cast<const Polygon*>(&line))
            {
                // Union all polygons first and add them later
                polygons.push_back(*polygon);

                /*temp = Shape(asRawVector()).unionPolygons();
                actual_polygons = &temp.asRawVector();
                end_type = ClipperLib::etClosedPolygon;*/
            }
            else
            {
                ClipperLib::EndType end_type;

                if (line.isClosed())
                {
                    end_type = ClipperLib::etClosedLine;
                }
                else if (joinType == ClipperLib::jtMiter)
                {
                    end_type = ClipperLib::etOpenSquare;
                }
                else
                {
                    end_type = ClipperLib::etOpenRound;
                }

                clipper.AddPath(line.getPoints(), joinType, end_type);
            }
        }

        if (! polygons.empty())
        {
            polygons = polygons.unionPolygons();

            for (const Polygon& polygon : polygons)
            {
                clipper.AddPath(polygon.getPoints(), joinType, ClipperLib::etClosedPolygon);
            }
        }

        clipper.MiterLimit = miter_limit;

        ClipperLib::Paths result;
        clipper.Execute(result, static_cast<double>(distance));
        return Shape(std::move(result));
    }
}

template<class LineType>
void LinesSet<LineType>::removeDegenerateVerts()
{
    for (size_t poly_idx = 0; poly_idx < lines_.size(); poly_idx++)
    {
        LineType& poly = lines_[poly_idx];
        bool for_polyline = ! poly.isClosed();
        Polygon result;

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
                poly = std::move(result);
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
        clipper.AddPath(line.getPoints(), PolyTyp, line.getType() == PolylineType::Filled);
    }
}


template size_t LinesSet<Polyline>::pointCount() const;
template void LinesSet<Polyline>::addLine(const Point2LL& from, const Point2LL& to);
template void LinesSet<Polyline>::removeAt(size_t index);
template void LinesSet<Polyline>::splitIntoSegments(LinesSet<Polyline>& result) const;
template LinesSet<Polyline> LinesSet<Polyline>::splitIntoSegments() const;
template coord_t LinesSet<Polyline>::length() const;
template Shape LinesSet<Polyline>::tubeShape(const coord_t inner_offset, const coord_t outer_offset) const;
template Shape LinesSet<Polyline>::offset(coord_t distance, ClipperLib::JoinType joinType, double miter_limit) const;
template void LinesSet<Polyline>::removeDegenerateVerts();
template void LinesSet<Polyline>::addIfNotEmpty(const Polyline& line);
template void LinesSet<Polyline>::addIfNotEmpty(Polyline&& line);
template void LinesSet<Polyline>::addPaths(ClipperLib::Clipper& clipper, ClipperLib::PolyType PolyTyp) const;

template size_t LinesSet<Polygon>::pointCount() const;
template void LinesSet<Polygon>::removeAt(size_t index);
template void LinesSet<Polygon>::splitIntoSegments(LinesSet<Polyline>& result) const;
template LinesSet<Polyline> LinesSet<Polygon>::splitIntoSegments() const;
template coord_t LinesSet<Polygon>::length() const;
template Shape LinesSet<Polygon>::tubeShape(const coord_t inner_offset, const coord_t outer_offset) const;
template Shape LinesSet<Polygon>::offset(coord_t distance, ClipperLib::JoinType joinType, double miter_limit) const;
template void LinesSet<Polygon>::removeDegenerateVerts();
template void LinesSet<Polygon>::addIfNotEmpty(const Polygon& line);
template void LinesSet<Polygon>::addIfNotEmpty(Polygon&& line);
template void LinesSet<Polygon>::addPaths(ClipperLib::Clipper& clipper, ClipperLib::PolyType PolyTyp) const;

} // namespace cura

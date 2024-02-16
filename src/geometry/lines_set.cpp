// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/lines_set.h"

#include <numeric>

#include "geometry/open_polyline.h"
#include "geometry/polygon.h"
#include "geometry/polyline_type.h"
#include "geometry/shape.h"

namespace cura
{

template<class LineType>
size_t LinesSet<LineType>::pointCount() const
{
    return std::accumulate(
        this->begin(),
        this->end(),
        size_t(0),
        [](size_t total, const Polygon& polygon)
        {
            return total + polygon.size();
        });
}

template<class LineType>
void LinesSet<LineType>::addLine(const Point2LL& from, const Point2LL& to)
{
    this->emplace_back(std::initializer_list<Point2LL>{ from, to });
}

template<class LineType>
void LinesSet<LineType>::addIfNotEmpty(const LineType& line)
{
    if (! line.empty())
    {
        this->push_back(line);
    }
}

template<class LineType>
void LinesSet<LineType>::addIfNotEmpty(LineType&& line)
{
    if (! line.empty())
    {
        this->emplace_back(std::move(line));
    }
}

template<class LineType>
void LinesSet<LineType>::removeAt(size_t index)
{
    if (this->size() == 1)
    {
        this->clear();
    }
    else if (this->size() > 1)
    {
        assert(index < this->size());
        if (index < this->size() - 1)
        {
            (*this)[index] = std::move(this->back());
        }
        this->resize(this->size() - 1);
    }
}

template<class LineType>
void LinesSet<LineType>::splitIntoSegments(LinesSet<OpenPolyline>& result) const
{
    for (const LineType& line : (*this))
    {
        line.splitIntoSegments(result);
    }
}

template<class LineType>
LinesSet<OpenPolyline> LinesSet<LineType>::splitIntoSegments() const
{
    LinesSet<OpenPolyline> result;
    for (const LineType& line : (*this))
    {
        line.splitIntoSegments(result);
    }
    return result;
}

template<class LineType>
coord_t LinesSet<LineType>::length() const
{
    return std::accumulate(
        this->begin(),
        this->end(),
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
        return Shape(getCallable());
    }

    Shape temp;
    const ClipperLib::Paths* actual_polygons = &getCallable();
    Shape ret;
    ClipperLib::EndType end_type;
    if constexpr (LineType::type_ == PolylineType::Filled)
    {
        temp = Shape(getCallable()).unionPolygons();
        actual_polygons = &temp.getCallable();
        end_type = ClipperLib::etClosedPolygon;
    }
    else if constexpr (LineType::type_ == PolylineType::Closed)
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
    ClipperLib::ClipperOffset clipper(miter_limit, 10.0);
    clipper.AddPaths(*actual_polygons, joinType, end_type);
    clipper.MiterLimit = miter_limit;
    clipper.Execute(ret.getCallable(), distance);
    return ret;
}

template<class LineType>
void LinesSet<LineType>::removeDegenerateVertsForEveryone()
{
    constexpr bool for_polyline = LineType::type_ == PolylineType::Open;
    for (size_t poly_idx = 0; poly_idx < this->size(); poly_idx++)
    {
        LineType& poly = (*this)[poly_idx];
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
                poly = result;
            }
            else
            {
                removeAt(poly_idx);
                poly_idx--; // effectively the next iteration has the same poly_idx (referring to a new poly which is not yet processed)
            }
        }
    }
}

template size_t LinesSet<OpenPolyline>::pointCount() const;
template void LinesSet<OpenPolyline>::addLine(const Point2LL& from, const Point2LL& to);
template void LinesSet<OpenPolyline>::removeAt(size_t index);
template void LinesSet<OpenPolyline>::splitIntoSegments(LinesSet<OpenPolyline>& result) const;
template LinesSet<OpenPolyline> LinesSet<OpenPolyline>::splitIntoSegments() const;
template coord_t LinesSet<OpenPolyline>::length() const;
template Shape LinesSet<OpenPolyline>::tubeShape(const coord_t inner_offset, const coord_t outer_offset) const;
template Shape LinesSet<OpenPolyline>::offset(coord_t distance, ClipperLib::JoinType joinType, double miter_limit) const;
template void LinesSet<OpenPolyline>::removeDegenerateVertsForEveryone();
template void LinesSet<OpenPolyline>::addIfNotEmpty(const OpenPolyline& line);
template void LinesSet<OpenPolyline>::addIfNotEmpty(OpenPolyline&& line);

template size_t LinesSet<Polygon>::pointCount() const;
template void LinesSet<Polygon>::addLine(const Point2LL& from, const Point2LL& to);
template void LinesSet<Polygon>::removeAt(size_t index);
template void LinesSet<Polygon>::splitIntoSegments(LinesSet<OpenPolyline>& result) const;
template LinesSet<OpenPolyline> LinesSet<Polygon>::splitIntoSegments() const;
template coord_t LinesSet<Polygon>::length() const;
template Shape LinesSet<Polygon>::tubeShape(const coord_t inner_offset, const coord_t outer_offset) const;
template Shape LinesSet<Polygon>::offset(coord_t distance, ClipperLib::JoinType joinType, double miter_limit) const;
template void LinesSet<Polygon>::removeDegenerateVertsForEveryone();
template void LinesSet<Polygon>::addIfNotEmpty(const Polygon& line);
template void LinesSet<Polygon>::addIfNotEmpty(Polygon&& line);

} // namespace cura

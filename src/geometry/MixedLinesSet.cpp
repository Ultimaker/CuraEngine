// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/MixedLinesSet.h"

#include <numeric>

#include "geometry/ClosedLinesSet.h"
#include "geometry/OpenLinesSet.h"
#include "geometry/Polygon.h"
#include "geometry/Shape.h"


namespace cura
{

MixedLinesSet::MixedLinesSet(const ClosedLinesSet& lines)
{
    push_back(lines);
}

MixedLinesSet::MixedLinesSet(ClipperLib::PolyTree&& tree)
{
    ClipperLib::Paths polylines;
    ClipperLib::OpenPathsFromPolyTree(tree, polylines); // Open with clipperlib means non-surface (polyline)
    constexpr bool polyline_explicitely_closed = true;
    for (ClipperLib::Path& path : polylines)
    {
        if (path.empty())
        {
            continue;
        }

        if (path.front() == path.back())
        {
            push_back(ClosedPolyline(std::move(path), polyline_explicitely_closed));
        }
        else
        {
            push_back(OpenPolyline(std::move(path)));
        }
    }

    ClipperLib::Paths polygons;
    ClipperLib::ClosedPathsFromPolyTree(tree, polygons); // Closed with clipperlib means surface (polygon)
    constexpr bool polygon_explicitely_closed = false;
    for (ClipperLib::Path& path : polygons)
    {
        if (path.empty())
        {
            continue;
        }

        push_back(Polygon(std::move(path), polygon_explicitely_closed));
    }
}

Shape MixedLinesSet::offset(coord_t distance, ClipperLib::JoinType join_type, double miter_limit) const
{
    if (distance == 0)
    {
        // Return a shape that contains only actual polygons
        Shape result;

        for (const PolylinePtr& line : (*this))
        {
            if (const std::shared_ptr<const Polygon> polygon = dynamic_pointer_cast<const Polygon>(line))
            {
                result.push_back(*polygon);
            }
        }

        return result;
    }
    Shape polygons;
    ClipperLib::ClipperOffset clipper(miter_limit, 10.0);

    for (const PolylinePtr& line : (*this))
    {
        if (const std::shared_ptr<const Polygon> polygon = dynamic_pointer_cast<const Polygon>(line))
        {
            // Union all polygons first and add them later
            polygons.push_back(*polygon);
        }
        else
        {
            ClipperLib::EndType end_type;

            if (line->hasClosingSegment())
            {
                end_type = ClipperLib::etClosedLine;
            }
            else
            {
                end_type = (join_type == ClipperLib::jtMiter) ? ClipperLib::etOpenSquare : ClipperLib::etOpenRound;
            }

            clipper.AddPath(line->getPoints(), join_type, end_type);
        }
    }

    if (! polygons.empty())
    {
        polygons = polygons.unionPolygons();

        for (const Polygon& polygon : polygons)
        {
            clipper.AddPath(polygon.getPoints(), join_type, ClipperLib::etClosedPolygon);
        }
    }

    clipper.MiterLimit = miter_limit;

    ClipperLib::Paths result;
    clipper.Execute(result, static_cast<double>(distance));
    return Shape{ std::move(result) };
}

MixedLinesSet MixedLinesSet::intersection(const Shape& shape) const
{
    if (empty() || shape.empty())
    {
        return {};
    }

    ClipperLib::PolyTree ret;
    ClipperLib::Clipper clipper(clipper_init);
    for (const PolylinePtr& line : (*this))
    {
        line->addPath(clipper, ClipperLib::ptSubject);
    }
    shape.addPaths(clipper, ClipperLib::ptClip);
    clipper.Execute(ClipperLib::ctIntersection, ret);
    return MixedLinesSet(std::move(ret));
}

void MixedLinesSet::push_back(const OpenPolyline& line)
{
    std::vector<PolylinePtr>::push_back(std::make_shared<OpenPolyline>(line));
}

void MixedLinesSet::push_back(const ClosedPolyline& line)
{
    std::vector<PolylinePtr>::push_back(std::make_shared<ClosedPolyline>(line));
}

void MixedLinesSet::push_back(OpenPolyline&& line)
{
    std::vector<PolylinePtr>::push_back(std::make_shared<OpenPolyline>(std::move(line)));
}

void MixedLinesSet::push_back(ClosedPolyline&& line)
{
    std::vector<PolylinePtr>::push_back(std::make_shared<ClosedPolyline>(std::move(line)));
}

void MixedLinesSet::push_back(const Polygon& line)
{
    std::vector<PolylinePtr>::push_back(std::make_shared<Polygon>(line));
}

void MixedLinesSet::push_back(const std::shared_ptr<OpenPolyline>& line)
{
    std::vector<PolylinePtr>::push_back(line);
}

void MixedLinesSet::push_back(const PolylinePtr& line)
{
    std::vector<PolylinePtr>::push_back(line);
}

void MixedLinesSet::push_back(OpenLinesSet&& lines_set)
{
    reserve(size() + lines_set.size());
    for (OpenPolyline& line : lines_set)
    {
        push_back(std::move(line));
    }
}

void MixedLinesSet::push_back(const OpenLinesSet& lines_set)
{
    reserve(size() + lines_set.size());
    for (const OpenPolyline& line : lines_set)
    {
        push_back(line);
    }
}

void MixedLinesSet::push_back(ClosedLinesSet&& lines_set)
{
    reserve(size() + lines_set.size());
    for (ClosedPolyline& line : lines_set)
    {
        push_back(std::move(line));
    }
}

void MixedLinesSet::push_back(const ClosedLinesSet& lines_set)
{
    reserve(size() + lines_set.size());
    for (const ClosedPolyline& line : lines_set)
    {
        push_back(line);
    }
}

void MixedLinesSet::push_back(const MixedLinesSet& lines_set)
{
    insert(end(), lines_set.begin(), lines_set.end());
}

void MixedLinesSet::push_back(MixedLinesSet&& lines_set)
{
    insert(end(), std::make_move_iterator(lines_set.begin()), std::make_move_iterator(lines_set.end()));
}

void MixedLinesSet::push_back(const LinesSet<Polygon>& lines_set)
{
    reserve(size() + lines_set.size());
    for (const Polygon& line : lines_set)
    {
        push_back(line);
    }
}

void MixedLinesSet::push_back(LinesSet<Polygon>&& lines_set)
{
    reserve(size() + lines_set.size());
    for (Polygon& line : lines_set)
    {
        push_back(std::move(line));
    }
}

coord_t MixedLinesSet::length() const
{
    return std::accumulate(
        begin(),
        end(),
        0LL,
        [](coord_t value, const PolylinePtr& line)
        {
            return value + line->length();
        });
}

} // namespace cura

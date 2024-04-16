// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/shape.h"

#include <mapbox/geometry/wagyu/wagyu.hpp>
#include <numeric>
#include <unordered_set>

#ifdef BUILD_TESTS
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/read.hpp>
#include <fmt/format.h>
#include <range/v3/to_container.hpp>
#include <range/v3/view/c_str.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/take.hpp>
#include <range/v3/view/transform.hpp>
#endif

#include <range/v3/view/filter.hpp>
#include <range/v3/view/sliding.hpp>

#include "geometry/mixed_lines_set.h"
#include "geometry/parts_view.h"
#include "geometry/polygon.h"
#include "geometry/single_shape.h"
#include "settings/types/Ratio.h"
#include "utils/OpenPolylineStitcher.h"
#include "utils/linearAlg2D.h"

namespace cura
{

Shape::Shape(ClipperLib::Paths&& paths, bool explicitely_closed)
{
    emplace_back(std::move(paths), explicitely_closed);
}

Shape::Shape(const std::vector<Polygon>& polygons)
    : LinesSet<Polygon>(polygons)
{
}

void Shape::emplace_back(ClipperLib::Paths&& paths, bool explicitely_closed)
{
    reserve(size() + paths.size());
    for (ClipperLib::Path& path : paths)
    {
        emplace_back(std::move(path), explicitely_closed);
    }
}

void Shape::emplace_back(ClipperLib::Path&& path, bool explicitely_closed)
{
    static_cast<LinesSet<Polygon>*>(this)->emplace_back(std::move(path), explicitely_closed);
}

Shape& Shape::operator=(const Shape& other)
{
    LinesSet<Polygon>::operator=(other);
    return *this;
}

Shape& Shape::operator=(Shape&& other)
{
    LinesSet<Polygon>::operator=(std::move(other));
    return *this;
}

Shape Shape::approxConvexHull(int extra_outset) const
{
    constexpr int overshoot = MM2INT(100); // 10cm (hard-coded value).

    Shape convex_hull;
    // Perform the offset for each polygon one at a time.
    // This is necessary because the polygons may overlap, in which case the offset could end up in an infinite loop.
    // See http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/ClipperOffset/_Body.htm
    for (const Polygon& polygon : (*this))
    {
        ClipperLib::Paths offset_result;
        ClipperLib::ClipperOffset offsetter(1.2, 10.0);
        offsetter.AddPath(polygon.getPoints(), ClipperLib::jtRound, ClipperLib::etClosedPolygon);
        offsetter.Execute(offset_result, overshoot);
        convex_hull.emplace_back(std::move(offset_result));
    }

    return convex_hull.unionPolygons().offset(-overshoot + extra_outset, ClipperLib::jtRound);
}

void Shape::makeConvex()
{
    // early out if there is nothing to do
    if (empty())
    {
        return;
    }

    // Andrew’s Monotone Chain Convex Hull Algorithm
    std::vector<Point2LL> points;
    for (const Polygon& poly : getLines())
    {
        points.insert(points.end(), poly.begin(), poly.end());
    }

    Polygon convexified;
    auto make_sorted_poly_convex = [&convexified](std::vector<Point2LL>& poly)
    {
        convexified.push_back(poly[0]);

        for (const auto window : poly | ranges::views::sliding(2))
        {
            const Point2LL& current = window[0];
            const Point2LL& after = window[1];

            if (LinearAlg2D::pointIsLeftOfLine(current, convexified.back(), after) < 0)
            {
                // Track backwards to make sure we haven't been in a concave pocket for multiple vertices already.
                while (convexified.size() >= 2
                       && (LinearAlg2D::pointIsLeftOfLine(convexified.back(), convexified[convexified.size() - 2], current) >= 0
                           || LinearAlg2D::pointIsLeftOfLine(convexified.back(), convexified[convexified.size() - 2], convexified.front()) > 0))
                {
                    convexified.pop_back();
                }
                convexified.push_back(current);
            }
        }
    };

    std::sort(
        points.begin(),
        points.end(),
        [](const Point2LL& a, const Point2LL& b)
        {
            return a.X == b.X ? a.Y < b.Y : a.X < b.X;
        });
    make_sorted_poly_convex(points);
    std::reverse(points.begin(), points.end());
    make_sorted_poly_convex(points);

    setLines({ std::move(convexified) });
}

Shape Shape::difference(const Shape& other) const
{
    ClipperLib::Paths ret;
    ClipperLib::Clipper clipper(clipper_init);
    addPaths(clipper, ClipperLib::ptSubject);
    other.addPaths(clipper, ClipperLib::ptClip);
    clipper.Execute(ClipperLib::ctDifference, ret);
    return Shape(std::move(ret));
}

Shape Shape::unionPolygons(const Shape& other, ClipperLib::PolyFillType fill_type) const
{
    ClipperLib::Paths ret;
    ClipperLib::Clipper clipper(clipper_init);
    addPaths(clipper, ClipperLib::ptSubject);
    other.addPaths(clipper, ClipperLib::ptSubject);
    clipper.Execute(ClipperLib::ctUnion, ret, fill_type, fill_type);
    return Shape(std::move(ret));
}

Shape Shape::unionPolygons() const
{
    return unionPolygons(Shape());
}

Shape Shape::intersection(const Shape& other) const
{
    ClipperLib::Paths ret;
    ClipperLib::Clipper clipper(clipper_init);
    addPaths(clipper, ClipperLib::ptSubject);
    other.addPaths(clipper, ClipperLib::ptClip);
    clipper.Execute(ClipperLib::ctIntersection, ret);
    return Shape(std::move(ret));
}

Shape Shape::offset(coord_t distance, ClipperLib::JoinType join_type, double miter_limit) const
{
    if (distance == 0)
    {
        return Shape(*this);
    }
    ClipperLib::Paths ret;
    ClipperLib::ClipperOffset clipper(miter_limit, 10.0);
    unionPolygons().addPaths(clipper, join_type, ClipperLib::etClosedPolygon);
    clipper.MiterLimit = miter_limit;
    clipper.Execute(ret, static_cast<double>(distance));
    return Shape(std::move(ret));
}

bool Shape::inside(const Point2LL& p, bool border_result) const
{
    int poly_count_inside = 0;
    for (const Polygon& poly : *this)
    {
        const int is_inside_this_poly = ClipperLib::PointInPolygon(p, poly.getPoints());
        if (is_inside_this_poly == -1)
        {
            return border_result;
        }
        poly_count_inside += is_inside_this_poly;
    }
    return (poly_count_inside % 2) == 1;
}

size_t Shape::findInside(const Point2LL& p, bool border_result) const
{
    if (size() < 1)
    {
        return false;
    }

    // NOTE: Keep these vectors fixed-size, they replace an (non-standard, sized at runtime) arrays.
    std::vector<int64_t> min_x(size(), std::numeric_limits<int64_t>::max());
    std::vector<int64_t> crossings(size());

    for (size_t poly_idx = 0; poly_idx < size(); poly_idx++)
    {
        const Polygon& poly = (*this)[poly_idx];
        Point2LL p0 = poly.back();
        for (const Point2LL& p1 : poly)
        {
            short comp = LinearAlg2D::pointLiesOnTheRightOfLine(p, p0, p1);
            if (comp == 1)
            {
                crossings[poly_idx]++;
                int64_t x;
                if (p1.Y == p0.Y)
                {
                    x = p0.X;
                }
                else
                {
                    x = p0.X + (p1.X - p0.X) * (p.Y - p0.Y) / (p1.Y - p0.Y);
                }
                if (x < min_x[poly_idx])
                {
                    min_x[poly_idx] = x;
                }
            }
            else if (border_result && comp == 0)
            {
                return poly_idx;
            }
            p0 = p1;
        }
    }

    int64_t min_x_uneven = std::numeric_limits<int64_t>::max();
    size_t ret = NO_INDEX;
    size_t n_unevens = 0;
    for (size_t array_idx = 0; array_idx < size(); array_idx++)
    {
        if (crossings[array_idx] % 2 == 1)
        {
            n_unevens++;
            if (min_x[array_idx] < min_x_uneven)
            {
                min_x_uneven = min_x[array_idx];
                ret = array_idx;
            }
        }
    }
    if (n_unevens % 2 == 0)
    {
        ret = NO_INDEX;
    }
    return ret;
}

template<class LineType>
OpenLinesSet Shape::intersection(const LinesSet<LineType>& polylines, bool restitch, const coord_t max_stitch_distance) const
{
    LinesSet<OpenPolyline> split_polylines = polylines.splitIntoSegments();

    ClipperLib::PolyTree result;
    ClipperLib::Clipper clipper(clipper_init);
    split_polylines.addPaths(clipper, ClipperLib::ptSubject);
    addPaths(clipper, ClipperLib::ptClip);
    clipper.Execute(ClipperLib::ctIntersection, result);
    ClipperLib::Paths result_paths;
    ClipperLib::OpenPathsFromPolyTree(result, result_paths);

    OpenLinesSet result_lines(std::move(result_paths));

    if (restitch)
    {
        OpenLinesSet result_open_lines;
        Shape result_closed_lines;

        const coord_t snap_distance = 10_mu;
        OpenPolylineStitcher::stitch(result_lines, result_open_lines, result_closed_lines, max_stitch_distance, snap_distance);

        result_lines = std::move(result_open_lines);
        // if open polylines got stitched into closed polylines, split them back up into open polylines again, because the result only admits open polylines
        for (ClosedPolyline& closed_line : result_closed_lines)
        {
            if (! closed_line.empty())
            {
                if (closed_line.size() > 2)
                {
                    closed_line.push_back(closed_line.front());
                }
                result_lines.emplace_back(std::move(closed_line.getPoints()));
            }
        }
    }

    return result_lines;
}

Shape Shape::xorPolygons(const Shape& other, ClipperLib::PolyFillType pft) const
{
    ClipperLib::Paths ret;
    ClipperLib::Clipper clipper(clipper_init);
    addPaths(clipper, ClipperLib::ptSubject);
    other.addPaths(clipper, ClipperLib::ptClip);
    clipper.Execute(ClipperLib::ctXor, ret, pft);
    return Shape(std::move(ret));
}

Shape Shape::execute(ClipperLib::PolyFillType pft) const
{
    ClipperLib::Paths ret;
    ClipperLib::Clipper clipper(clipper_init);
    addPaths(clipper, ClipperLib::ptSubject);
    clipper.Execute(ClipperLib::ctXor, ret, pft);
    return Shape(std::move(ret));
}

Shape Shape::offsetMulti(const std::vector<coord_t>& offset_dists) const
{
    // we need as many offset-dists as points
    assert(pointCount() == offset_dists.size());

    ClipperLib::Paths ret;
    size_t i = 0;
    for (const Polygon& poly_line : (*this)
                                        | ranges::views::filter(
                                            [](const Polygon& path)
                                            {
                                                return ! path.empty();
                                            }))
    {
        Polygon ret_poly_line;

        auto prev_p = poly_line.back();
        auto prev_dist = offset_dists[i + poly_line.size() - 1];

        for (const Point2LL& p : poly_line)
        {
            auto offset_dist = offset_dists[i];

            auto vec_dir = prev_p - p;

            constexpr coord_t min_vec_len = 10;
            if (vSize2(vec_dir) > min_vec_len * min_vec_len)
            {
                auto offset_p1 = turn90CCW(normal(vec_dir, prev_dist));
                auto offset_p2 = turn90CCW(normal(vec_dir, offset_dist));

                ret_poly_line.push_back(prev_p + offset_p1);
                ret_poly_line.push_back(p + offset_p2);
            }

            prev_p = p;
            prev_dist = offset_dist;
            i++;
        }

        ret.push_back(std::move(ret_poly_line.getPoints()));
    }

    ClipperLib::SimplifyPolygons(ret, ClipperLib::PolyFillType::pftPositive);

    return Shape(std::move(ret));
}

Shape Shape::getOutsidePolygons() const
{
    Shape ret;
    ClipperLib::Clipper clipper(clipper_init);
    ClipperLib::PolyTree poly_tree;
    addPaths(clipper, ClipperLib::ptSubject);
    clipper.Execute(ClipperLib::ctUnion, poly_tree);

    for (size_t outer_poly_idx = 0; outer_poly_idx < static_cast<size_t>(poly_tree.ChildCount()); outer_poly_idx++)
    {
        ClipperLib::PolyNode* child = poly_tree.Childs[outer_poly_idx];
        ret.emplace_back(std::move(child->Contour));
    }
    return ret;
}

Shape Shape::removeEmptyHoles() const
{
    Shape ret;
    ClipperLib::Clipper clipper(clipper_init);
    ClipperLib::PolyTree poly_tree;
    addPaths(clipper, ClipperLib::ptSubject);
    clipper.Execute(ClipperLib::ctUnion, poly_tree);

    bool remove_holes = true;
    removeEmptyHoles_processPolyTreeNode(poly_tree, remove_holes, ret);
    return ret;
}

Shape Shape::getEmptyHoles() const
{
    Shape ret;
    ClipperLib::Clipper clipper(clipper_init);
    ClipperLib::PolyTree poly_tree;
    addPaths(clipper, ClipperLib::ptSubject);
    clipper.Execute(ClipperLib::ctUnion, poly_tree);

    bool remove_holes = false;
    removeEmptyHoles_processPolyTreeNode(poly_tree, remove_holes, ret);
    return ret;
}

void Shape::removeEmptyHoles_processPolyTreeNode(const ClipperLib::PolyNode& node, const bool remove_holes, Shape& ret) const
{
    for (size_t outer_poly_idx = 0; outer_poly_idx < static_cast<size_t>(node.ChildCount()); outer_poly_idx++)
    {
        ClipperLib::PolyNode* child = node.Childs[outer_poly_idx];
        if (remove_holes)
        {
            ret.emplace_back(std::move(child->Contour));
        }
        for (size_t hole_node_idx = 0; hole_node_idx < static_cast<size_t>(child->ChildCount()); hole_node_idx++)
        {
            ClipperLib::PolyNode& hole_node = *child->Childs[hole_node_idx];
            if ((hole_node.ChildCount() > 0) == remove_holes)
            {
                ret.emplace_back(std::move(hole_node.Contour));
                removeEmptyHoles_processPolyTreeNode(hole_node, remove_holes, ret);
            }
        }
    }
}

void Shape::removeSmallAreas(const double min_area_size, const bool remove_holes)
{
    auto new_end = end();
    if (remove_holes)
    {
        for (auto it = begin(); it < new_end;)
        {
            // All polygons smaller than target are removed by replacing them with a polygon from the back of the vector
            if (std::abs(INT2MM2(it->area())) < min_area_size)
            {
                *it = std::move(*--new_end);
                continue;
            }
            it++; // Skipped on removal such that the polygon just swaped in is checked next
        }
    }
    else
    {
        // For each polygon, computes the signed area, move small outlines at the end of the vector and keep references on small holes
        std::vector<Polygon*> small_holes;
        for (auto it = begin(); it < new_end;)
        {
            double area = INT2MM2(it->area());
            if (std::abs(area) < min_area_size)
            {
                if (area >= 0)
                {
                    --new_end;
                    if (it < new_end)
                    {
                        std::swap(*new_end, *it);
                        continue;
                    }
                    else
                    { // Don't self-swap the last Path
                        break;
                    }
                }
                else
                {
                    small_holes.push_back(&(*it));
                }
            }
            it++; // Skipped on removal such that the polygon just swaped in is checked next
        }

        // Removes small holes that have their first point inside one of the removed outlines
        // Iterating in reverse ensures that unprocessed small holes won't be moved
        const auto removed_outlines_start = new_end;
        for (auto hole_it = small_holes.rbegin(); hole_it < small_holes.rend(); hole_it++)
        {
            for (auto outline_it = removed_outlines_start; outline_it < end(); outline_it++)
            {
                if (outline_it->inside((*hole_it)->front()))
                {
                    **hole_it = std::move(*--new_end);
                    break;
                }
            }
        }
    }
    resize(new_end - begin());
}

Shape Shape::removePolygon(const Shape& to_be_removed, int same_distance) const
{
    Shape result;
    for (size_t poly_keep_idx = 0; poly_keep_idx < size(); poly_keep_idx++)
    {
        const Polygon& poly_keep = (*this)[poly_keep_idx];
        bool should_be_removed = false;
        if (poly_keep.size() > 0)
            //             for (int hole_poly_idx = 0; hole_poly_idx < to_be_removed.size(); hole_poly_idx++)
            for (const Polygon& poly_rem : to_be_removed)
            {
                //                 PolygonRef poly_rem = to_be_removed[hole_poly_idx];
                if (poly_rem.size() != poly_keep.size() || poly_rem.size() == 0)
                    continue;

                // find closest point, supposing this point aligns the two shapes in the best way
                size_t closest_point_idx = 0;
                coord_t smallestDist2 = -1;
                for (size_t point_rem_idx = 0; point_rem_idx < poly_rem.size(); point_rem_idx++)
                {
                    coord_t dist2 = vSize2(poly_rem[point_rem_idx] - poly_keep[0]);
                    if (dist2 < smallestDist2 || smallestDist2 < 0)
                    {
                        smallestDist2 = dist2;
                        closest_point_idx = point_rem_idx;
                    }
                }
                bool poly_rem_is_poly_keep = true;
                // compare the two polygons on all points
                if (smallestDist2 > same_distance * same_distance)
                    continue;
                for (size_t point_idx = 0; point_idx < poly_rem.size(); point_idx++)
                {
                    coord_t dist2 = vSize2(poly_rem[(closest_point_idx + point_idx) % poly_rem.size()] - poly_keep[point_idx]);
                    if (dist2 > same_distance * same_distance)
                    {
                        poly_rem_is_poly_keep = false;
                        break;
                    }
                }
                if (poly_rem_is_poly_keep)
                {
                    should_be_removed = true;
                    break;
                }
            }
        if (! should_be_removed)
            result.push_back(poly_keep);
    }
    return result;
}

Shape Shape::processEvenOdd(ClipperLib::PolyFillType poly_fill_type) const
{
    ClipperLib::Paths ret;
    ClipperLib::Clipper clipper(clipper_init);
    addPaths(clipper, ClipperLib::ptSubject);
    clipper.Execute(ClipperLib::ctUnion, ret, poly_fill_type);
    return Shape(std::move(ret));
}

Shape Shape::smooth_outward(const AngleDegrees max_angle, int shortcut_length) const
{
    Shape ret;
    for (const Polygon& poly : (*this))
    {
        if (poly.size() < 3)
        {
            continue;
        }
        if (poly.size() == 3)
        {
            ret.push_back(poly);
            continue;
        }
        poly.smooth_outward(max_angle, shortcut_length, ret.newLine());
        if (ret.back().size() < 3)
        {
            ret.resize(ret.size() - 1);
        }
    }
    return ret;
}

Shape Shape::smooth(int remove_length) const
{
    Shape ret;
    for (const Polygon& poly : (*this))
    {
        if (poly.size() < 3)
        {
            continue;
        }
        if (poly.size() == 3)
        {
            ret.push_back(poly);
            continue;
        }
        poly.smooth(remove_length, ret.newLine());
        Polygon& back = ret.back();
        if (back.size() < 3)
        {
            back.resize(back.size() - 1);
        }
    }
    return ret;
}

Shape Shape::smooth2(int remove_length, int min_area) const
{
    Shape ret;
    for (const Polygon& poly : (*this))
    {
        if (poly.size() == 0)
        {
            continue;
        }
        if (poly.area() < min_area || poly.size() <= 5) // when optimally removing, a poly with 5 pieces results in a triangle. Smaller polys dont have area!
        {
            ret.push_back(poly);
            continue;
        }
        if (poly.size() < 4)
        {
            ret.push_back(poly);
        }
        else
        {
            poly.smooth2(remove_length, ret.newLine());
        }
    }
    return ret;
}

void Shape::removeColinearEdges(const AngleRadians max_deviation_angle)
{
    Shape& thiss = *this;
    for (size_t p = 0; p < size(); p++)
    {
        thiss[p].removeColinearEdges(max_deviation_angle);
        if (thiss[p].size() < 3)
        {
            removeAt(p);
            p--;
        }
    }
}

void Shape::scale(const Ratio& ratio)
{
    if (ratio == 1.)
    {
        return;
    }

    for (auto& points : *this)
    {
        for (auto& pt : points)
        {
            pt = pt * static_cast<double>(ratio);
        }
    }
}

void Shape::translate(const Point2LL& delta)
{
    if (delta.X != 0 || delta.Y != 0)
    {
        for (Polygon& polygon : *this)
        {
            polygon.translate(delta);
        }
    }
}

double Shape::area() const
{
    return std::accumulate(
        begin(),
        end(),
        0.0,
        [](double total, const Polygon& poly)
        {
            // note: holes already have negative area
            return total + poly.area();
        });
}

std::vector<SingleShape> Shape::splitIntoParts(bool unionAll) const
{
    std::vector<SingleShape> ret;
    ClipperLib::Clipper clipper(clipper_init);
    ClipperLib::PolyTree resultPolyTree;
    addPaths(clipper, ClipperLib::ptSubject);
    if (unionAll)
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    else
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree);

    splitIntoParts_processPolyTreeNode(&resultPolyTree, ret);
    return ret;
}

void Shape::splitIntoParts_processPolyTreeNode(ClipperLib::PolyNode* node, std::vector<SingleShape>& ret) const
{
    for (size_t n = 0; n < static_cast<size_t>(node->ChildCount()); n++)
    {
        ClipperLib::PolyNode* child = node->Childs[n];
        SingleShape part;
        part.emplace_back(std::move(child->Contour));
        for (size_t i = 0; i < static_cast<size_t>(child->ChildCount()); i++)
        {
            part.emplace_back(std::move(child->Childs[i]->Contour));
            splitIntoParts_processPolyTreeNode(child->Childs[i], ret);
        }
        ret.push_back(std::move(part));
    }
}

std::vector<Shape> Shape::sortByNesting() const
{
    std::vector<Shape> ret;
    ClipperLib::Clipper clipper(clipper_init);
    ClipperLib::PolyTree resultPolyTree;
    addPaths(clipper, ClipperLib::ptSubject);
    clipper.Execute(ClipperLib::ctUnion, resultPolyTree);

    sortByNesting_processPolyTreeNode(&resultPolyTree, 0, ret);
    return ret;
}

void Shape::sortByNesting_processPolyTreeNode(ClipperLib::PolyNode* node, const size_t nesting_idx, std::vector<Shape>& ret) const
{
    for (size_t n = 0; n < static_cast<size_t>(node->ChildCount()); n++)
    {
        ClipperLib::PolyNode* child = node->Childs[n];
        if (nesting_idx >= ret.size())
        {
            ret.resize(nesting_idx + 1);
        }
        ret[nesting_idx].emplace_back(std::move(child->Contour));
        sortByNesting_processPolyTreeNode(child, nesting_idx + 1, ret);
    }
}

PartsView Shape::splitIntoPartsView(bool unionAll)
{
    Shape reordered;
    PartsView partsView(*this);
    ClipperLib::Clipper clipper(clipper_init);
    ClipperLib::PolyTree resultPolyTree;
    addPaths(clipper, ClipperLib::ptSubject);
    if (unionAll)
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    else
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree);

    splitIntoPartsView_processPolyTreeNode(partsView, reordered, &resultPolyTree);

    (*this) = std::move(reordered);
    return partsView;
}

void Shape::splitIntoPartsView_processPolyTreeNode(PartsView& partsView, Shape& reordered, ClipperLib::PolyNode* node) const
{
    for (size_t n = 0; n < static_cast<size_t>(node->ChildCount()); n++)
    {
        ClipperLib::PolyNode* child = node->Childs[n];
        partsView.emplace_back();
        size_t pos = partsView.size() - 1;
        partsView[pos].push_back(reordered.size());
        reordered.emplace_back(std::move(child->Contour));
        for (size_t i = 0; i < static_cast<size_t>(child->ChildCount()); i++)
        {
            partsView[pos].push_back(reordered.size());
            reordered.emplace_back(std::move(child->Childs[i]->Contour));
            splitIntoPartsView_processPolyTreeNode(partsView, reordered, child->Childs[i]);
        }
    }
}

Shape Shape::removeNearSelfIntersections() const
{
    using map_pt = mapbox::geometry::point<coord_t>;
    using map_ring = mapbox::geometry::linear_ring<coord_t>;
    using map_poly = mapbox::geometry::polygon<coord_t>;
    using map_mpoly = mapbox::geometry::multi_polygon<coord_t>;

    map_mpoly mwpoly;

    mapbox::geometry::wagyu::wagyu<coord_t> wagyu;

    for (auto& polygon : splitIntoParts())
    {
        mwpoly.emplace_back();
        map_poly& wpoly = mwpoly.back();
        for (auto& path : polygon)
        {
            wpoly.push_back(std::move(*reinterpret_cast<std::vector<mapbox::geometry::point<coord_t>>*>(&path.getPoints())));
            for (auto& point : wpoly.back())
            {
                point.x /= 4;
                point.y /= 4;
            }
            wagyu.add_ring(wpoly.back());
        }
    }

    map_mpoly sln;

    wagyu.execute(mapbox::geometry::wagyu::clip_type_union, sln, mapbox::geometry::wagyu::fill_type_even_odd, mapbox::geometry::wagyu::fill_type_even_odd);

    Shape polys;

    for (auto& poly : sln)
    {
        for (auto& ring : poly)
        {
            ring.pop_back();
            for (auto& point : ring)
            {
                point.x *= 4;
                point.y *= 4;
            }
            polys.emplace_back(std::move(*reinterpret_cast<ClipperLib::Path*>(&ring)));
        }
    }
    polys = polys.unionPolygons();
    polys.removeColinearEdges();

    return polys;
}

void Shape::simplify(ClipperLib::PolyFillType fill_type)
{
    // This is the actual content from clipper.cpp::SimplifyPolygons, but rewritten here in order
    // to avoid a list copy
    ClipperLib::Clipper clipper;
    ClipperLib::Paths ret;
    clipper.StrictlySimple(true);
    addPaths(clipper, ClipperLib::ptSubject);
    clipper.Execute(ClipperLib::ctUnion, ret, fill_type, fill_type);

    for (size_t i = 0; i < size(); ++i)
    {
        getLines()[i].setPoints(std::move(ret[i]));
    }
}

void Shape::ensureManifold()
{
    std::vector<Point2LL> duplicate_locations;
    std::unordered_set<Point2LL> poly_locations;
    for (const Polygon& poly : (*this))
    {
        for (const Point2LL& p : poly)
        {
            if (poly_locations.find(p) != poly_locations.end())
            {
                duplicate_locations.push_back(p);
            }
            poly_locations.emplace(p);
        }
    }
    Shape removal_dots;
    for (const Point2LL& p : duplicate_locations)
    {
        Polygon& dot = removal_dots.newLine();
        dot.push_back(p + Point2LL(0, 5));
        dot.push_back(p + Point2LL(5, 0));
        dot.push_back(p + Point2LL(0, -5));
        dot.push_back(p + Point2LL(-5, 0));
    }
    if (! removal_dots.empty())
    {
        *this = difference(removal_dots);
    }
}

void Shape::applyMatrix(const PointMatrix& matrix)
{
    for (Polygon& polygon : *this)
    {
        polygon.applyMatrix(matrix);
    }
}

void Shape::applyMatrix(const Point3Matrix& matrix)
{
    for (Polygon& polygon : *this)
    {
        polygon.applyMatrix(matrix);
    }
}

#ifdef BUILD_TESTS
[[maybe_unused]] Shape Shape::fromWkt(const std::string& wkt)
{
    typedef boost::geometry::model::d2::point_xy<double> point_type;
    typedef boost::geometry::model::polygon<point_type> polygon_type;

    polygon_type poly;
    boost::geometry::read_wkt(wkt, poly);

    Shape ret;

    Polygon outer;
    for (const auto& point : poly.outer())
    {
        outer.emplace_back(point.x(), point.y());
    }
    ret.push_back(outer);

    for (const auto& hole : poly.inners())
    {
        Polygon inner;
        for (const auto& point : hole)
        {
            inner.emplace_back(point.x(), point.y());
        }
        ret.push_back(inner);
    }

    return ret;
}

[[maybe_unused]] void Shape::writeWkt(std::ostream& stream) const
{
    stream << "POLYGON (";
    const auto paths_str = getLines()
                         | ranges::views::transform(
                               [](const Polygon& path)
                               {
                                   const auto line_string = ranges::views::concat(path, path | ranges::views::take(1))
                                                          | ranges::views::transform(
                                                                [](const Point2LL& point)
                                                                {
                                                                    return fmt::format("{} {}", point.X, point.Y);
                                                                })
                                                          | ranges::views::join(ranges::views::c_str(", ")) | ranges::to<std::string>();
                                   return "(" + line_string + ")";
                               })
                         | ranges::views::join(ranges::views::c_str(", ")) | ranges::to<std::string>();
    stream << paths_str;
    stream << ")";
}
#endif

template LinesSet<OpenPolyline> Shape::intersection(const LinesSet<OpenPolyline>& polylines, bool restitch, const coord_t max_stitch_distance) const;
template LinesSet<OpenPolyline> Shape::intersection(const LinesSet<ClosedPolyline>& polylines, bool restitch, const coord_t max_stitch_distance) const;
template LinesSet<OpenPolyline> Shape::intersection(const LinesSet<Polygon>& polylines, bool restitch, const coord_t max_stitch_distance) const;

} // namespace cura

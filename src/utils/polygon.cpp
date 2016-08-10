/** Copyright (C) 2015 Ultimaker - Released under terms of the AGPLv3 License */
#include "polygon.h"

#include "linearAlg2D.h" // pointLiesOnTheRightOfLine

#include "ListPolyIt.h"

namespace cura 
{

bool PolygonRef::shorterThan(int64_t check_length) const
{
    const PolygonRef& polygon = *this;
    const Point* p0 = &polygon.back();
    int64_t length = 0;
    for (const Point& p1 : polygon)
    {
        length += vSize(*p0 - p1);
        if (length >= check_length)
        {
            return false;
        }
        p0 = &p1;
    }
    return true;
}

bool PolygonRef::_inside(Point p, bool border_result)
{
    PolygonRef thiss = *this;
    if (size() < 1)
    {
        return false;
    }
    
    int crossings = 0;
    Point p0 = back();
    for(unsigned int n=0; n<size(); n++)
    {
        Point p1 = thiss[n];
        // no tests unless the segment p0-p1 is at least partly at, or to right of, p.X
        short comp = LinearAlg2D::pointLiesOnTheRightOfLine(p, p0, p1);
        if (comp == 1)
        {
            crossings++;
        }
        else if (comp == 0)
        {
            return border_result;
        }
        p0 = p1;
    }
    return (crossings % 2) == 1;
}

bool Polygons::inside(Point p, bool border_result) const
{
    const Polygons& thiss = *this;
    if (size() < 1)
    {
        return false;
    }
    
    int crossings = 0;
    for (const ClipperLib::Path& poly : thiss)
    {
        Point p0 = poly.back();
        for (const Point& p1 : poly)
        {
            short comp = LinearAlg2D::pointLiesOnTheRightOfLine(p, p0, p1);
            if (comp == 1)
            {
                crossings++;
            }
            else if (comp == 0)
            {
                return border_result;
            }
            p0 = p1;
        }
    }
    return (crossings % 2) == 1;
}

unsigned int Polygons::findInside(Point p, bool border_result)
{
    Polygons& thiss = *this;
    if (size() < 1)
    {
        return false;
    }
    
    int64_t min_x[size()];
    std::fill_n(min_x, size(), std::numeric_limits<int64_t>::max());  // initialize with int.max
    int crossings[size()];
    std::fill_n(crossings, size(), 0);  // initialize with zeros
    
    for (unsigned int poly_idx = 0; poly_idx < size(); poly_idx++)
    {
        PolygonRef poly = thiss[poly_idx];
        Point p0 = poly.back();
        for(Point& p1 : poly)
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
                    x = p0.X + (p1.X-p0.X) * (p.Y-p0.Y) / (p1.Y-p0.Y);
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
    unsigned int ret = NO_INDEX;
    unsigned int n_unevens = 0;
    for (unsigned int array_idx = 0; array_idx < size(); array_idx++)
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
    if (n_unevens % 2 == 0) { ret = NO_INDEX; }
    return ret;
}

Polygons PolygonRef::offset(int distance, ClipperLib::JoinType joinType, double miter_limit) const
{
    Polygons ret;
    ClipperLib::ClipperOffset clipper(miter_limit, 10.0);
    clipper.AddPath(*path, joinType, ClipperLib::etClosedPolygon);
    clipper.MiterLimit = miter_limit;
    clipper.Execute(ret.paths, distance);
    return ret;
}

Polygon Polygons::convexHull() const
{
    // Implements Andrew's monotone chain convex hull algorithm
    // See https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain

    using IntPoint = ClipperLib::IntPoint;

    size_t num_points = 0U;
    for (const ClipperLib::Path& path : paths)
    {
        num_points += path.size();
    }

    std::vector<IntPoint> all_points;
    all_points.reserve(num_points);
    for (const ClipperLib::Path& path : paths)
    {
        for (const IntPoint& point : path)
        {
            all_points.push_back(point);
        }
    }

    struct HullSort
    {
        bool operator()(const IntPoint& a,
                        const IntPoint& b)
        {
            return (a.X < b.X) ||
                (a.X == b.X && a.Y < b.Y);
        }
    };

    std::sort(all_points.begin(), all_points.end(), HullSort());

    // positive for left turn, 0 for straight, negative for right turn
    auto ccw = [](const IntPoint& p0, const IntPoint& p1, const IntPoint& p2) -> int64_t {
        IntPoint v01(p1.X - p0.X, p1.Y - p0.Y);
        IntPoint v12(p2.X - p1.X, p2.Y - p1.Y);

        return
            static_cast<int64_t>(v01.X) * v12.Y -
            static_cast<int64_t>(v01.Y) * v12.X;
    };

    Polygon hull_poly;
    ClipperLib::Path& hull_points = *hull_poly;
    hull_points.resize(num_points+1);
    // index to insert next hull point, also number of valid hull points
    size_t hull_idx = 0;

    // Build lower hull
    for (size_t pt_idx = 0U; pt_idx != num_points; ++pt_idx)
    {
        while (hull_idx >= 2 &&
               ccw(hull_points[hull_idx-2], hull_points[hull_idx-1],
                   all_points[pt_idx]) <= 0)
        {
            --hull_idx;
        }
        hull_points[hull_idx] = all_points[pt_idx];
        ++hull_idx;
    }

    // Build upper hull
    size_t min_upper_hull_chain_end_idx = hull_idx+1;
    for (int pt_idx = num_points-2; pt_idx >= 0; --pt_idx)
    {
        while (hull_idx >= min_upper_hull_chain_end_idx &&
               ccw(hull_points[hull_idx-2], hull_points[hull_idx-1],
                   all_points[pt_idx]) <= 0)
        {
            --hull_idx;
        }
        hull_points[hull_idx] = all_points[pt_idx];
        ++hull_idx;
    }

    assert(hull_idx <= hull_points.size());

    // Last point is duplicted with first.  It is removed in the resize.
    hull_points.resize(hull_idx - 2);

    return hull_poly;
}

void PolygonRef::simplify(int smallest_line_segment_squared, int allowed_error_distance_squared){

    if (size() < 3)
    {
        clear();
        return;
    }

// Algorithm Explanation:
// initial: 1
// after stage I: 2
// after first step of stage II: 3
//
//                       111111111111111111111111111
//                      1         22 332222         1
//                     1       222  3  3   222       1
//                    1      22    3   3      222     1
//                    1   222      3    3        2222 1
//                   1 222        3      3           221
//                  122          3       3              12
//                 12            3        3              1
//                 12           3          3             1
//                1 2          3            3            21
//               1  2          3            3            21
//               1  2         3              3          2  1
//              1    2        3               3         2  1
//              1    2       3                 3        2   1
//             1     2      3                  3        2   1
//              1    2      3                   3       2    1
//               1    2    3                     3      2    1
//                1   2   3                      3     2    1
//                 1  2   3                       3    2   1
//                  1  2 3                         3   2  1
//                   1 23                           3  2 1
//                    123                           3  21
//                     1                             321
//                                                    1
//
// this is neccesary in order to avoid the case where a long segment is followed by a lot of small segments would get simplified to a long segment going to the wrong end point
//  .......                _                 _______
// |                      /                 |
// |     would become    /    instead of    |
// |                    /                   |
//
// therefore every time a vert is removed the next is kept (for the moment)
//
// .......            ._._._._.         .___.___.          .________
// |                  |                 |                  |
// |    will  become  |    will become  |     will become  |
// |                  |                 |                  |
//

    ClipperLib::Path this_path = *path;
    coord_t min_length_2 = std::min(smallest_line_segment_squared, allowed_error_distance_squared);

    ListPolygon result_list_poly;
    result_list_poly.emplace_back(path->front());

    std::vector<ListPolyIt> skipped_verts;
    // add the first point as starting point for the algorithm
    // whether the first point has to be removed is checked separately afterwards
    skipped_verts.emplace_back(result_list_poly, result_list_poly.begin());

    char here_is_beyond_line = 0;
    { // stage I: convert to a ListPolygon and remove verts, but don't remove verts just after removed verts (i.e. skip them)
        Point prev = this_path[0];
        auto skip = [this, &result_list_poly, &this_path, &prev, &skipped_verts](unsigned int& poly_idx)
        {
            poly_idx++;
            if (poly_idx >= size())
            { // don't wrap around, the first point has already been added
                return;
            }
            result_list_poly.emplace_back(this_path[poly_idx]);
            prev = result_list_poly.back();
            skipped_verts.emplace_back(result_list_poly, --result_list_poly.end());
        };

        for (unsigned int poly_idx = 1; poly_idx < size(); poly_idx++)
        {
            const Point& here = this_path[poly_idx];
            const Point& next = this_path[(poly_idx + 1) % size()];
            if (here == next)
            { // disregard duplicate points without skipping the next point
                continue;
            }
            if ( vSize2(here - prev) < min_length_2 && vSize2(next - here) < min_length_2 )
            {
                // don't add [here] to the result
                // skip checking whether the next point has to be removed for now
                skip(poly_idx);
                continue;
            }
            int64_t error2 = LinearAlg2D::getDist2FromLineSegment(prev, here, next, &here_is_beyond_line);
            if (here_is_beyond_line == 0 && error2 < allowed_error_distance_squared)
            {
                // don't add [here] to the result
                // skip checking whether the next point has to be removed for now
                skip(poly_idx);
            }
            else
            {
                result_list_poly.emplace_back(here);
                prev = result_list_poly.back();
            }
        }
    }

    { // stage II: keep removing skipped verts (and skip the next vert if it was a skipped vert)
        auto skip = [&skipped_verts, &result_list_poly](unsigned int& skipped_vert_idx, const ListPolyIt skipped_vert, std::vector<ListPolyIt>& new_skipped_verts)
        {
            unsigned int next_skipped_vert_idx = skipped_vert_idx + 1;
            if (next_skipped_vert_idx < skipped_verts.size() && skipped_vert.next() == skipped_verts[next_skipped_vert_idx])
            {
                skipped_vert_idx++;
                new_skipped_verts.emplace_back(skipped_verts[next_skipped_vert_idx]);
            }
            result_list_poly.erase(skipped_vert.it);
        };
        while (!skipped_verts.empty() && result_list_poly.size() >= 3)
        {
            std::vector<ListPolyIt> new_skipped_verts;

            for (unsigned int skipped_vert_idx = 0; skipped_vert_idx < skipped_verts.size(); skipped_vert_idx++)
            {
                ListPolyIt skipped_vert = skipped_verts[skipped_vert_idx];
                const Point& here = skipped_vert.p();
                const Point& prev = skipped_vert.prev().p();

                const Point& next = skipped_vert.next().p();
                if ( vSize2(here - prev) < min_length_2 && vSize2(next - here) < min_length_2 )
                {
                    // skip checking whether the next point has to be removed for now
                    skip(skipped_vert_idx, skipped_vert, new_skipped_verts);
                    continue;
                }
                int64_t error2 = LinearAlg2D::getDist2FromLineSegment(prev, here, next, &here_is_beyond_line);
                if (here_is_beyond_line == 0 && error2 < allowed_error_distance_squared)
                {
                    // skip checking whether the next point has to be removed for now
                    skip(skipped_vert_idx, skipped_vert, new_skipped_verts);
                }
                else
                {
                    // keep the point
                }
            }
            skipped_verts = new_skipped_verts;
        }
    }

    clear();
    if (result_list_poly.size() < 3)
    {
        return;
    }
    ListPolyIt::convertListPolygonToPolygon(result_list_poly, *this);
}

std::vector<PolygonsPart> Polygons::splitIntoParts(bool unionAll) const
{
    std::vector<PolygonsPart> ret;
    ClipperLib::Clipper clipper(clipper_init);
    ClipperLib::PolyTree resultPolyTree;
    clipper.AddPaths(paths, ClipperLib::ptSubject, true);
    if (unionAll)
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    else
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree);

    splitIntoParts_processPolyTreeNode(&resultPolyTree, ret);
    return ret;
}

void Polygons::splitIntoParts_processPolyTreeNode(ClipperLib::PolyNode* node, std::vector<PolygonsPart>& ret) const
{
    for(int n=0; n<node->ChildCount(); n++)
    {
        ClipperLib::PolyNode* child = node->Childs[n];
        PolygonsPart part;
        part.add(child->Contour);
        for(int i=0; i<child->ChildCount(); i++)
        {
            part.add(child->Childs[i]->Contour);
            splitIntoParts_processPolyTreeNode(child->Childs[i], ret);
        }
        ret.push_back(part);
    }
}

unsigned int PartsView::getPartContaining(unsigned int poly_idx, unsigned int* boundary_poly_idx) 
{
    PartsView& partsView = *this;
    for (unsigned int part_idx_now = 0; part_idx_now < partsView.size(); part_idx_now++)
    {
        std::vector<unsigned int>& partView = partsView[part_idx_now];
        if (partView.size() == 0) { continue; }
        std::vector<unsigned int>::iterator result = std::find(partView.begin(), partView.end(), poly_idx);
        if (result != partView.end()) 
        { 
            if (boundary_poly_idx) { *boundary_poly_idx = partView[0]; }
            return part_idx_now;
        }
    }
    return NO_INDEX;
}

PolygonsPart PartsView::assemblePart(unsigned int part_idx) const
{
    const PartsView& partsView = *this;
    PolygonsPart ret;
    if (part_idx != NO_INDEX)
    {
        for (unsigned int poly_idx_ff : partsView[part_idx])
        {
            ret.add(polygons[poly_idx_ff]);
        }
    }
    return ret;
}

PolygonsPart PartsView::assemblePartContaining(unsigned int poly_idx, unsigned int* boundary_poly_idx) 
{
    PolygonsPart ret;
    unsigned int part_idx = getPartContaining(poly_idx, boundary_poly_idx);
    if (part_idx != NO_INDEX)
    {
        return assemblePart(part_idx);
    }
    return ret;
}

PartsView Polygons::splitIntoPartsView(bool unionAll)
{
    Polygons reordered;
    PartsView partsView(*this);
    ClipperLib::Clipper clipper(clipper_init);
    ClipperLib::PolyTree resultPolyTree;
    clipper.AddPaths(paths, ClipperLib::ptSubject, true);
    if (unionAll)
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    else
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree);

    splitIntoPartsView_processPolyTreeNode(partsView, reordered, &resultPolyTree);
    
    (*this) = reordered;
    return partsView;
}

void Polygons::splitIntoPartsView_processPolyTreeNode(PartsView& partsView, Polygons& reordered, ClipperLib::PolyNode* node) const
{
    for(int n=0; n<node->ChildCount(); n++)
    {
        ClipperLib::PolyNode* child = node->Childs[n];
        partsView.emplace_back();
        unsigned int pos = partsView.size() - 1;
        partsView[pos].push_back(reordered.size());
        reordered.add(child->Contour);
        for(int i = 0; i < child->ChildCount(); i++)
        {
            partsView[pos].push_back(reordered.size());
            reordered.add(child->Childs[i]->Contour);
            splitIntoPartsView_processPolyTreeNode(partsView, reordered, child->Childs[i]);
        }
    }
}



}//namespace cura

// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "SlicerLayer.h"

#include <algorithm> // remove_if

#include "Mesh.h"
#include "SlicedUVCoordinates.h"
#include "geometry/OpenPolyline.h"
#include "settings/ESurfaceMode.h"
#include "utils/Simplify.h"
#include "utils/SparsePointGridInclusive.h"

namespace curaengine
{

constexpr int largest_neglected_gap_first_phase = MM2INT(0.01); //!< distance between two line segments regarded as connected
constexpr int largest_neglected_gap_second_phase = MM2INT(0.02); //!< distance between two line segments regarded as connected
constexpr int max_stitch1 = MM2INT(10.0); //!< maximal distance stitched between open polylines to form polygons

void SlicerLayer::makeBasicPolygonLoops(OpenLinesSet& open_polylines)
{
    for (size_t start_segment_idx = 0; start_segment_idx < segments_.size(); start_segment_idx++)
    {
        if (! segments_[start_segment_idx].addedToPolygon)
        {
            makeBasicPolygonLoop(open_polylines, start_segment_idx);
        }
    }
}

void SlicerLayer::makeBasicPolygonLoop(OpenLinesSet& open_polylines, const size_t start_segment_idx)
{
    Polygon poly(true);
    poly.push_back(segments_[start_segment_idx].start);

    for (int segment_idx = start_segment_idx; segment_idx != -1;)
    {
        SlicerSegment& segment = segments_[segment_idx];
        poly.push_back(segment.end);
        segment.addedToPolygon = true;
        segment_idx = getNextSegmentIdx(segment, start_segment_idx);
        if (segment_idx == static_cast<int>(start_segment_idx))
        { // polyon is closed
            polygons_.push_back(std::move(poly));
            return;
        }
    }
    // polygon couldn't be closed
    open_polylines.emplace_back(std::move(poly.getPoints()));
}

int SlicerLayer::tryFaceNextSegmentIdx(const SlicerSegment& segment, const int face_idx, const size_t start_segment_idx) const
{
    decltype(face_idx_to_segment_idx_.begin()) it;
    auto it_end = face_idx_to_segment_idx_.end();
    it = face_idx_to_segment_idx_.find(face_idx);
    if (it != it_end)
    {
        const int segment_idx = (*it).second;
        Point2LL p1 = segments_[segment_idx].start;
        Point2LL diff = segment.end - p1;
        if (shorterThen(diff, largest_neglected_gap_first_phase))
        {
            if (segment_idx == static_cast<int>(start_segment_idx))
            {
                return start_segment_idx;
            }
            if (segments_[segment_idx].addedToPolygon)
            {
                return -1;
            }
            return segment_idx;
        }
    }

    return -1;
}

int SlicerLayer::getNextSegmentIdx(const SlicerSegment& segment, const size_t start_segment_idx) const
{
    int next_segment_idx = -1;

    const bool segment_ended_at_edge = segment.endVertex == nullptr;
    if (segment_ended_at_edge)
    {
        const int face_to_try = segment.endOtherFaceIdx;
        if (face_to_try == -1)
        {
            return -1;
        }
        return tryFaceNextSegmentIdx(segment, face_to_try, start_segment_idx);
    }
    else
    {
        // segment ended at vertex

        const std::vector<uint32_t>& faces_to_try = segment.endVertex->connected_faces_;
        for (int face_to_try : faces_to_try)
        {
            const int result_segment_idx = tryFaceNextSegmentIdx(segment, face_to_try, start_segment_idx);
            if (result_segment_idx == static_cast<int>(start_segment_idx))
            {
                return start_segment_idx;
            }
            else if (result_segment_idx != -1)
            {
                // not immediately returned since we might still encounter the start_segment_idx
                next_segment_idx = result_segment_idx;
            }
        }
    }

    return next_segment_idx;
}

void SlicerLayer::connectOpenPolylines(OpenLinesSet& open_polylines)
{
    constexpr bool allow_reverse = false;
    // Search a bit fewer cells but at cost of covering more area.
    // Since acceptance area is small to start with, the extra is unlikely to hurt much.
    constexpr coord_t cell_size = largest_neglected_gap_first_phase * 2;
    connectOpenPolylinesImpl(open_polylines, largest_neglected_gap_second_phase, cell_size, allow_reverse);
}

void SlicerLayer::stitch(OpenLinesSet& open_polylines)
{
    bool allow_reverse = true;
    connectOpenPolylinesImpl(open_polylines, max_stitch1, max_stitch1, allow_reverse);
}

const SlicerLayer::Terminus SlicerLayer::Terminus::INVALID_TERMINUS{ ~static_cast<Index>(0U) };

bool SlicerLayer::PossibleStitch::operator<(const PossibleStitch& other) const
{
    // better if lower distance
    if (dist2 > other.dist2)
    {
        return true;
    }
    else if (dist2 < other.dist2)
    {
        return false;
    }

    // better if in order instead of reversed
    if (! in_order() && other.in_order())
    {
        return true;
    }

    // better if lower Terminus::Index for terminus_0
    // This just defines a more total order and isn't strictly necessary.
    if (terminus_0.asIndex() > other.terminus_0.asIndex())
    {
        return true;
    }
    else if (terminus_0.asIndex() < other.terminus_0.asIndex())
    {
        return false;
    }

    // better if lower Terminus::Index for terminus_1
    // This just defines a more total order and isn't strictly necessary.
    if (terminus_1.asIndex() > other.terminus_1.asIndex())
    {
        return true;
    }
    else if (terminus_1.asIndex() < other.terminus_1.asIndex())
    {
        return false;
    }

    // The stitches have equal goodness
    return false;
}

std::priority_queue<SlicerLayer::PossibleStitch>
    SlicerLayer::findPossibleStitches(const OpenLinesSet& open_polylines, coord_t max_dist, coord_t cell_size, bool allow_reverse) const
{
    std::priority_queue<PossibleStitch> stitch_queue;

    // maximum distance squared
    int64_t max_dist2 = max_dist * max_dist;

    // Represents a terminal point of a polyline in open_polylines.
    struct StitchGridVal
    {
        unsigned int polyline_idx;
        // Depending on the SparsePointGridInclusive, either the start point or the
        // end point of the polyline
        Point2LL polyline_term_pt;
    };

    struct StitchGridValLocator
    {
        Point2LL operator()(const StitchGridVal& val) const
        {
            return val.polyline_term_pt;
        }
    };

    // Used to find nearby end points within a fixed maximum radius
    SparsePointGrid<StitchGridVal, StitchGridValLocator> grid_ends(cell_size);
    // Used to find nearby start points within a fixed maximum radius
    SparsePointGrid<StitchGridVal, StitchGridValLocator> grid_starts(cell_size);

    // populate grids

    // Inserts the ends of all polylines into the grid (does not
    //   insert the starts of the polylines).
    for (unsigned int polyline_0_idx = 0; polyline_0_idx < open_polylines.size(); polyline_0_idx++)
    {
        const OpenPolyline& polyline_0 = open_polylines[polyline_0_idx];

        if (polyline_0.size() < 1)
            continue;

        StitchGridVal grid_val;
        grid_val.polyline_idx = polyline_0_idx;
        grid_val.polyline_term_pt = polyline_0.back();
        grid_ends.insert(grid_val);
    }

    // Inserts the start of all polylines into the grid.
    if (allow_reverse)
    {
        for (unsigned int polyline_0_idx = 0; polyline_0_idx < open_polylines.size(); polyline_0_idx++)
        {
            const OpenPolyline& polyline_0 = open_polylines[polyline_0_idx];

            if (polyline_0.size() < 1)
                continue;

            StitchGridVal grid_val;
            grid_val.polyline_idx = polyline_0_idx;
            grid_val.polyline_term_pt = polyline_0[0];
            grid_starts.insert(grid_val);
        }
    }

    // search for nearby end points
    for (unsigned int polyline_1_idx = 0; polyline_1_idx < open_polylines.size(); polyline_1_idx++)
    {
        const OpenPolyline& polyline_1 = open_polylines[polyline_1_idx];

        if (polyline_1.size() < 1)
            continue;

        std::vector<StitchGridVal> nearby_ends;

        // Check for stitches that append polyline_1 onto polyline_0
        // in natural order.  These are stitches that use the end of
        // polyline_0 and the start of polyline_1.
        nearby_ends = grid_ends.getNearby(polyline_1[0], max_dist);
        for (const auto& nearby_end : nearby_ends)
        {
            Point2LL diff = nearby_end.polyline_term_pt - polyline_1[0];
            int64_t dist2 = vSize2(diff);
            if (dist2 < max_dist2)
            {
                PossibleStitch poss_stitch;
                poss_stitch.dist2 = dist2;
                poss_stitch.terminus_0 = Terminus{ nearby_end.polyline_idx, true };
                poss_stitch.terminus_1 = Terminus{ polyline_1_idx, false };
                stitch_queue.push(poss_stitch);
            }
        }

        if (allow_reverse)
        {
            // Check for stitches that append polyline_1 onto polyline_0
            // by reversing order of polyline_1.  These are stitches that
            // use the end of polyline_0 and the end of polyline_1.
            nearby_ends = grid_ends.getNearby(polyline_1.back(), max_dist);
            for (const auto& nearby_end : nearby_ends)
            {
                // Disallow stitching with self with same end point
                if (nearby_end.polyline_idx == polyline_1_idx)
                {
                    continue;
                }

                Point2LL diff = nearby_end.polyline_term_pt - polyline_1.back();
                int64_t dist2 = vSize2(diff);
                if (dist2 < max_dist2)
                {
                    PossibleStitch poss_stitch;
                    poss_stitch.dist2 = dist2;
                    poss_stitch.terminus_0 = Terminus{ nearby_end.polyline_idx, true };
                    poss_stitch.terminus_1 = Terminus{ polyline_1_idx, true };
                    stitch_queue.push(poss_stitch);
                }
            }

            // Check for stitches that append polyline_1 onto polyline_0
            // by reversing order of polyline_0.  These are stitches that
            // use the start of polyline_0 and the start of polyline_1.
            std::vector<StitchGridVal> nearby_starts = grid_starts.getNearby(polyline_1[0], max_dist);
            for (const auto& nearby_start : nearby_starts)
            {
                // Disallow stitching with self with same end point
                if (nearby_start.polyline_idx == polyline_1_idx)
                {
                    continue;
                }

                Point2LL diff = nearby_start.polyline_term_pt - polyline_1[0];
                int64_t dist2 = vSize2(diff);
                if (dist2 < max_dist2)
                {
                    PossibleStitch poss_stitch;
                    poss_stitch.dist2 = dist2;
                    poss_stitch.terminus_0 = Terminus{ nearby_start.polyline_idx, false };
                    poss_stitch.terminus_1 = Terminus{ polyline_1_idx, false };
                    stitch_queue.push(poss_stitch);
                }
            }
        }
    }

    return stitch_queue;
}

void SlicerLayer::planPolylineStitch(const OpenLinesSet& open_polylines, Terminus& terminus_0, Terminus& terminus_1, bool reverse[2]) const
{
    size_t polyline_0_idx = terminus_0.getPolylineIdx();
    size_t polyline_1_idx = terminus_1.getPolylineIdx();
    bool back_0 = terminus_0.isEnd();
    bool back_1 = terminus_1.isEnd();
    reverse[0] = false;
    reverse[1] = false;
    if (back_0)
    {
        if (back_1)
        {
            // back of both polylines
            // we can reverse either one and then append onto the other
            // reverse the smaller polyline
            if (open_polylines[polyline_0_idx].size() < open_polylines[polyline_1_idx].size())
            {
                std::swap(terminus_0, terminus_1);
            }
            reverse[1] = true;
        }
        else
        {
            // back of 0, front of 1
            // already in order, nothing to do
        }
    }
    else
    {
        if (back_1)
        {
            // front of 0, back of 1
            // in order if we swap 0 and 1
            std::swap(terminus_0, terminus_1);
        }
        else
        {
            // front of both polylines
            // we can reverse either one and then prepend to the other
            // reverse the smaller polyline
            if (open_polylines[polyline_0_idx].size() > open_polylines[polyline_1_idx].size())
            {
                std::swap(terminus_0, terminus_1);
            }
            reverse[0] = true;
        }
    }
}

void SlicerLayer::joinPolylines(OpenPolyline& polyline_0, OpenPolyline& polyline_1, const bool reverse[2])
{
    if (reverse[0])
    {
        // reverse polyline_0
        size_t size_0 = polyline_0.size();
        for (size_t idx = 0U; idx != size_0 / 2; ++idx)
        {
            std::swap(polyline_0[idx], polyline_0[size_0 - 1 - idx]);
        }
    }
    if (reverse[1])
    {
        // reverse polyline_1 by adding in reverse order
        for (int poly_idx = polyline_1.size() - 1; poly_idx >= 0; poly_idx--)
            polyline_0.push_back(polyline_1[poly_idx]);
    }
    else
    {
        // append polyline_1 onto polyline_0
        for (Point2LL& p : polyline_1)
            polyline_0.push_back(p);
    }
    polyline_1.clear();
}

SlicerLayer::TerminusTrackingMap::TerminusTrackingMap(Terminus::Index end_idx)
    : m_terminus_old_to_cur_map(end_idx)
{
    // Initialize map to everything points to itself since nothing has moved yet.
    for (size_t idx = 0U; idx != end_idx; ++idx)
    {
        m_terminus_old_to_cur_map[idx] = Terminus{ idx };
    }
    m_terminus_cur_to_old_map = m_terminus_old_to_cur_map;
}

void SlicerLayer::TerminusTrackingMap::updateMap(
    size_t num_terms,
    const Terminus* cur_terms,
    const Terminus* next_terms,
    size_t num_removed_terms,
    const Terminus* removed_cur_terms)
{
    // save old locations
    std::vector<Terminus> old_terms(num_terms);
    for (size_t idx = 0U; idx != num_terms; ++idx)
    {
        old_terms[idx] = getOldFromCur(cur_terms[idx]);
    }
    // update using maps old <-> cur and cur <-> next
    for (size_t idx = 0U; idx != num_terms; ++idx)
    {
        m_terminus_old_to_cur_map[old_terms[idx].asIndex()] = next_terms[idx];
        Terminus next_term = next_terms[idx];
        if (next_term != Terminus::INVALID_TERMINUS)
        {
            m_terminus_cur_to_old_map[next_term.asIndex()] = old_terms[idx];
        }
    }
    // remove next locations that no longer exist
    for (size_t rem_idx = 0U; rem_idx != num_removed_terms; ++rem_idx)
    {
        m_terminus_cur_to_old_map[removed_cur_terms[rem_idx].asIndex()] = Terminus::INVALID_TERMINUS;
    }
}

void SlicerLayer::connectOpenPolylinesImpl(OpenLinesSet& open_polylines, coord_t max_dist, coord_t cell_size, bool allow_reverse)
{
    // below code closes smallest gaps first

    std::priority_queue<PossibleStitch> stitch_queue = findPossibleStitches(open_polylines, max_dist, cell_size, allow_reverse);

    static const Terminus INVALID_TERMINUS = Terminus::INVALID_TERMINUS;
    Terminus::Index terminus_end_idx = Terminus::endIndexFromPolylineEndIndex(open_polylines.size());
    // Keeps track of how polyline end point locations move around
    TerminusTrackingMap terminus_tracking_map(terminus_end_idx);

    while (! stitch_queue.empty())
    {
        // Get the next best stitch
        PossibleStitch next_stitch;
        next_stitch = stitch_queue.top();
        stitch_queue.pop();
        Terminus old_terminus_0 = next_stitch.terminus_0;
        Terminus terminus_0 = terminus_tracking_map.getCurFromOld(old_terminus_0);
        if (terminus_0 == INVALID_TERMINUS)
        {
            // if we already used this terminus, then this stitch is no longer usable
            continue;
        }
        Terminus old_terminus_1 = next_stitch.terminus_1;
        Terminus terminus_1 = terminus_tracking_map.getCurFromOld(old_terminus_1);
        if (terminus_1 == INVALID_TERMINUS)
        {
            // if we already used this terminus, then this stitch is no longer usable
            continue;
        }

        size_t best_polyline_0_idx = terminus_0.getPolylineIdx();
        size_t best_polyline_1_idx = terminus_1.getPolylineIdx();

        // check to see if this completes a polygon
        bool completed_poly = best_polyline_0_idx == best_polyline_1_idx;
        if (completed_poly)
        {
            // finished polygon
            OpenPolyline& polyline_0 = open_polylines[best_polyline_0_idx];
            polygons_.push_back(Polygon(std::move(polyline_0.getPoints()), true)); // Will also clear the polyline
            Terminus cur_terms[2] = { { best_polyline_0_idx, false }, { best_polyline_0_idx, true } };
            for (size_t idx = 0U; idx != 2U; ++idx)
            {
                terminus_tracking_map.markRemoved(cur_terms[idx]);
            }
            continue;
        }

        // we need to join these polylines

        // plan how to join polylines
        bool reverse[2];
        planPolylineStitch(open_polylines, terminus_0, terminus_1, reverse);

        // need to reread since planPolylineStitch can swap terminus_0/1
        best_polyline_0_idx = terminus_0.getPolylineIdx();
        best_polyline_1_idx = terminus_1.getPolylineIdx();
        OpenPolyline& polyline_0 = open_polylines[best_polyline_0_idx];
        OpenPolyline& polyline_1 = open_polylines[best_polyline_1_idx];

        // join polylines according to plan
        joinPolylines(polyline_0, polyline_1, reverse);

        // update terminus_tracking_map
        Terminus cur_terms[4] = { { best_polyline_0_idx, false }, { best_polyline_0_idx, true }, { best_polyline_1_idx, false }, { best_polyline_1_idx, true } };
        Terminus next_terms[4] = { { best_polyline_0_idx, false }, INVALID_TERMINUS, INVALID_TERMINUS, { best_polyline_0_idx, true } };
        if (reverse[0])
        {
            std::swap(next_terms[0], next_terms[1]);
        }
        if (reverse[1])
        {
            std::swap(next_terms[2], next_terms[3]);
        }
        // cur_terms -> next_terms has movement map
        // best_polyline_1 is always removed
        terminus_tracking_map.updateMap(4U, cur_terms, next_terms, 2U, &cur_terms[2]);
    }
}

void SlicerLayer::stitch_extensive(OpenLinesSet& open_polylines)
{
    // For extensive stitching find 2 open polygons that are touching 2 closed polygons.
    //  Then find the shortest path over this polygon that can be used to connect the open polygons,
    //  And generate a path over this shortest bit to link up the 2 open polygons.
    //  (If these 2 open polygons are the same polygon, then the final result is a closed polyon)

    while (1)
    {
        unsigned int best_polyline_1_idx = -1;
        unsigned int best_polyline_2_idx = -1;
        std::optional<GapCloserResult> best_result;

        for (unsigned int polyline_1_idx = 0; polyline_1_idx < open_polylines.size(); polyline_1_idx++)
        {
            OpenPolyline& polyline_1 = open_polylines[polyline_1_idx];
            if (polyline_1.size() < 1)
                continue;

            {
                std::optional<GapCloserResult> res = findPolygonGapCloser(polyline_1[0], polyline_1.back());
                if (res && (! best_result || res->len < best_result->len))
                {
                    best_polyline_1_idx = polyline_1_idx;
                    best_polyline_2_idx = polyline_1_idx;
                    best_result = res;
                }
            }

            for (unsigned int polyline_2_idx = 0; polyline_2_idx < open_polylines.size(); polyline_2_idx++)
            {
                OpenPolyline& polyline_2 = open_polylines[polyline_2_idx];
                if (polyline_2.size() < 1 || polyline_1_idx == polyline_2_idx)
                    continue;

                std::optional<GapCloserResult> res = findPolygonGapCloser(polyline_1[0], polyline_2.back());
                if (res && (! best_result || res->len < best_result->len))
                {
                    best_polyline_1_idx = polyline_1_idx;
                    best_polyline_2_idx = polyline_2_idx;
                    best_result = res;
                }
            }
        }

        if (best_result)
        {
            if (best_polyline_1_idx == best_polyline_2_idx)
            {
                if (best_result->pointIdxA == best_result->pointIdxB)
                {
                    polygons_.push_back(Polygon(open_polylines[best_polyline_1_idx].getPoints(), true));
                    open_polylines[best_polyline_1_idx].clear();
                }
                else if (best_result->AtoB)
                {
                    Polygon& poly = polygons_.newLine();
                    for (unsigned int j = best_result->pointIdxA; j != best_result->pointIdxB; j = (j + 1) % polygons_[best_result->polygonIdx].size())
                        poly.push_back(polygons_[best_result->polygonIdx][j]);
                    for (unsigned int j = open_polylines[best_polyline_1_idx].size() - 1; int(j) >= 0; j--)
                        poly.push_back(open_polylines[best_polyline_1_idx][j]);
                    open_polylines[best_polyline_1_idx].clear();
                }
                else
                {
                    unsigned int n = polygons_.size();
                    polygons_.push_back(Polygon(open_polylines[best_polyline_1_idx].getPoints(), true));
                    for (unsigned int j = best_result->pointIdxB; j != best_result->pointIdxA; j = (j + 1) % polygons_[best_result->polygonIdx].size())
                        polygons_[n].push_back(polygons_[best_result->polygonIdx][j]);
                    open_polylines[best_polyline_1_idx].clear();
                }
            }
            else
            {
                if (best_result->pointIdxA == best_result->pointIdxB)
                {
                    for (unsigned int n = 0; n < open_polylines[best_polyline_1_idx].size(); n++)
                        open_polylines[best_polyline_2_idx].push_back(open_polylines[best_polyline_1_idx][n]);
                    open_polylines[best_polyline_1_idx].clear();
                }
                else if (best_result->AtoB)
                {
                    Polygon poly;
                    for (unsigned int n = best_result->pointIdxA; n != best_result->pointIdxB; n = (n + 1) % polygons_[best_result->polygonIdx].size())
                        poly.push_back(polygons_[best_result->polygonIdx][n]);
                    for (unsigned int n = poly.size() - 1; int(n) >= 0; n--)
                        open_polylines[best_polyline_2_idx].push_back(poly[n]);
                    for (unsigned int n = 0; n < open_polylines[best_polyline_1_idx].size(); n++)
                        open_polylines[best_polyline_2_idx].push_back(open_polylines[best_polyline_1_idx][n]);
                    open_polylines[best_polyline_1_idx].clear();
                }
                else
                {
                    for (unsigned int n = best_result->pointIdxB; n != best_result->pointIdxA; n = (n + 1) % polygons_[best_result->polygonIdx].size())
                        open_polylines[best_polyline_2_idx].push_back(polygons_[best_result->polygonIdx][n]);
                    for (unsigned int n = open_polylines[best_polyline_1_idx].size() - 1; int(n) >= 0; n--)
                        open_polylines[best_polyline_2_idx].push_back(open_polylines[best_polyline_1_idx][n]);
                    open_polylines[best_polyline_1_idx].clear();
                }
            }
        }
        else
        {
            break;
        }
    }
}

std::optional<GapCloserResult> SlicerLayer::findPolygonGapCloser(Point2LL ip0, Point2LL ip1)
{
    std::optional<ClosePolygonResult> c1 = findPolygonPointClosestTo(ip0);
    std::optional<ClosePolygonResult> c2 = findPolygonPointClosestTo(ip1);
    if (! c1 || ! c2 || c1->polygonIdx != c2->polygonIdx)
    {
        return std::nullopt;
    }

    GapCloserResult ret;
    ret.polygonIdx = c1->polygonIdx;
    ret.pointIdxA = c1->pointIdx;
    ret.pointIdxB = c2->pointIdx;
    ret.AtoB = true;

    if (ret.pointIdxA == ret.pointIdxB)
    {
        // Connection points are on the same line segment.
        ret.len = vSize(ip0 - ip1);
    }
    else
    {
        // Find out if we have should go from A to B or the other way around.
        Point2LL p0 = polygons_[ret.polygonIdx][ret.pointIdxA];
        int64_t lenA = vSize(p0 - ip0);
        for (unsigned int i = ret.pointIdxA; i != ret.pointIdxB; i = (i + 1) % polygons_[ret.polygonIdx].size())
        {
            Point2LL p1 = polygons_[ret.polygonIdx][i];
            lenA += vSize(p0 - p1);
            p0 = p1;
        }
        lenA += vSize(p0 - ip1);

        p0 = polygons_[ret.polygonIdx][ret.pointIdxB];
        int64_t lenB = vSize(p0 - ip1);
        for (unsigned int i = ret.pointIdxB; i != ret.pointIdxA; i = (i + 1) % polygons_[ret.polygonIdx].size())
        {
            Point2LL p1 = polygons_[ret.polygonIdx][i];
            lenB += vSize(p0 - p1);
            p0 = p1;
        }
        lenB += vSize(p0 - ip0);

        if (lenA < lenB)
        {
            ret.AtoB = true;
            ret.len = lenA;
        }
        else
        {
            ret.AtoB = false;
            ret.len = lenB;
        }
    }
    return ret;
}

std::optional<ClosePolygonResult> SlicerLayer::findPolygonPointClosestTo(Point2LL input)
{
    for (size_t n = 0; n < polygons_.size(); n++)
    {
        Point2LL p0 = polygons_[n][polygons_[n].size() - 1];
        for (size_t i = 0; i < polygons_[n].size(); i++)
        {
            Point2LL p1 = polygons_[n][i];

            // Q = A + Normal( B - A ) * ((( B - A ) dot ( P - A )) / VSize( A - B ));
            Point2LL pDiff = p1 - p0;
            int64_t lineLength = vSize(pDiff);
            if (lineLength > 1)
            {
                int64_t distOnLine = dot(pDiff, input - p0) / lineLength;
                if (distOnLine >= 0 && distOnLine <= lineLength)
                {
                    Point2LL q = p0 + pDiff * distOnLine / lineLength;
                    if (shorterThen(q - input, MM2INT(0.1)))
                    {
                        ClosePolygonResult ret;
                        ret.polygonIdx = n;
                        ret.pointIdx = i;
                        return ret;
                    }
                }
            }
            p0 = p1;
        }
    }

    return std::nullopt;
}

void SlicerLayer::makePolygons(const Mesh* mesh)
{
    OpenLinesSet open_polylines;

    makeBasicPolygonLoops(open_polylines);

    connectOpenPolylines(open_polylines);

    // TODO: (?) for mesh surface mode: connect open polygons. Maybe the above algorithm can create two open polygons which are actually connected when the starting segment is in
    // the middle between the two open polygons.

    if (mesh->settings_.get<ESurfaceMode>("magic_mesh_surface_mode") == ESurfaceMode::NORMAL)
    { // don't stitch when using (any) mesh surface mode, i.e. also don't stitch when using mixed mesh surface and closed polygons, because then polylines which are supposed to be
      // open will be closed
        stitch(open_polylines);
    }

    if (mesh->settings_.get<bool>("meshfix_extensive_stitching"))
    {
        stitch_extensive(open_polylines);
    }

    if (mesh->settings_.get<bool>("meshfix_keep_open_polygons"))
    {
        for (const OpenPolyline& polyline : open_polylines)
        {
            polygons_.push_back(Polygon(polyline.getPoints(), false), CheckNonEmptyParam::OnlyIfNotEmpty);
        }
    }

    for (const OpenPolyline& polyline : open_polylines)
    {
        open_polylines_.push_back(std::move(polyline), CheckNonEmptyParam::OnlyIfNotEmpty);
    }

    // Remove all the tiny polygons, or polygons that are not closed. As they do not contribute to the actual print.
    const coord_t snap_distance = std::max(mesh->settings_.get<coord_t>("minimum_polygon_circumference"), static_cast<coord_t>(1));
    auto itPolygons = std::remove_if(
        polygons_.begin(),
        polygons_.end(),
        [snap_distance](const Polygon& poly)
        {
            return poly.shorterThan(snap_distance);
        });
    polygons_.erase(itPolygons, polygons_.end());

    // Finally optimize all the polygons. Every point removed saves time in the long run.
    polygons_ = Simplify(mesh->settings_).polygon(polygons_);
    polygons_.removeDegenerateVerts(); // remove verts connected to overlapping line segments

    // Clean up polylines for Surface Mode printing
    auto itPolylines = std::remove_if(
        open_polylines_.begin(),
        open_polylines_.end(),
        [snap_distance](const OpenPolyline& line)
        {
            return line.shorterThan(snap_distance);
        });
    open_polylines_.erase(itPolylines, open_polylines_.end());

    open_polylines_.removeDegenerateVerts();

    sliced_uv_coordinates_ = std::make_shared<SlicedUVCoordinates>(segments_);

    // Clear the segment list to save memory, it is no longer needed after this point.
    segments_.clear();
}

} // namespace curaengine

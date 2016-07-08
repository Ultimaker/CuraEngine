/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#include <stdio.h>

#include <algorithm> // remove_if
#include <queue>

#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "utils/SparseGrid.h"

#include "slicer.h"
#include "debug.h" // TODO remove


namespace cura {
    
int largest_neglected_gap_first_phase = MM2INT(0.01); //!< distance between two line segments regarded as connected
int largest_neglected_gap_second_phase = MM2INT(0.02); //!< distance between two line segments regarded as connected
int max_stitch1 = MM2INT(10.0); //!< maximal distance stitched between open polylines to form polygons

void SlicerLayer::makeBasicPolygonLoops(const Mesh* mesh, Polygons& open_polylines)
{
    for(unsigned int start_segment_idx = 0; start_segment_idx < segments.size(); start_segment_idx++)
    {
        if (!segments[start_segment_idx].addedToPolygon)
        {
            makeBasicPolygonLoop(mesh, open_polylines, start_segment_idx);
        }
    }
    //Clear the segmentList to save memory, it is no longer needed after this point.
    segments.clear();
}

void SlicerLayer::makeBasicPolygonLoop(const Mesh* mesh, Polygons& open_polylines, unsigned int start_segment_idx)
{
    
    Polygon poly;
    poly.add(segments[start_segment_idx].start);
    
    for (int segment_idx = start_segment_idx; segment_idx != -1; )
    {
        SlicerSegment& segment = segments[segment_idx];
        poly.add(segment.end);
        segment.addedToPolygon = true;
        segment_idx = getNextSegmentIdx(mesh, segment, start_segment_idx);
        if (segment_idx == static_cast<int>(start_segment_idx)) 
        { // polyon is closed
            polygons.add(poly);
            return;
        }
    }
    // polygon couldn't be closed
    open_polylines.add(poly);
}

int SlicerLayer::tryFaceNextSegmentIdx(const Mesh* mesh, const SlicerSegment& segment, int face_idx, unsigned int start_segment_idx) const
{
    decltype(face_idx_to_segment_idx.begin()) it;
    auto it_end = face_idx_to_segment_idx.end();
    it = face_idx_to_segment_idx.find(face_idx);
    if (it != it_end)
    {
        int segment_idx = (*it).second;
        Point p1 = segments[segment_idx].start;
        Point diff = segment.end - p1;
        if (shorterThen(diff, largest_neglected_gap_first_phase))
        {
            if (segment_idx == static_cast<int>(start_segment_idx))
            {
                return start_segment_idx;
            }
            if (segments[segment_idx].addedToPolygon)
            {
                return -1;
            }
            return segment_idx;
        }
    }

    return -1;
}

int SlicerLayer::getNextSegmentIdx(const Mesh* mesh, const SlicerSegment& segment, unsigned int start_segment_idx)
{
    int next_segment_idx = -1;

    const std::vector<uint32_t> &faces_to_try = segment.endOtherFaces;
    bool segment_ended_at_edge = faces_to_try.empty();
    if (segment_ended_at_edge)
    {
        int face_to_try = segment.endOtherFaceIdx;
        if (face_to_try == -1)
        {
            return -1;
        }
        return tryFaceNextSegmentIdx(mesh,segment,face_to_try,start_segment_idx);
    }
    else
    {
        // segment ended at vertex

        for (int face_to_try : faces_to_try)
        {
            int result_segment_idx =
                tryFaceNextSegmentIdx(mesh,segment,face_to_try,start_segment_idx);
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

void SlicerLayer::connectOpenPolylines(Polygons& open_polylines)
{
    bool allow_reverse = false;
    // Search a bit fewer cells but at cost of covering more area.
    // Since acceptance area is small to start with, the extra is unlikely to hurt much.
    coord_t cell_size = largest_neglected_gap_first_phase * 2;
    connectOpenPolylinesImpl(open_polylines, largest_neglected_gap_second_phase, cell_size, allow_reverse);
}

void SlicerLayer::stitch(Polygons& open_polylines)
{
    bool allow_reverse = true;
    connectOpenPolylinesImpl(open_polylines, max_stitch1, max_stitch1, allow_reverse);
}

void SlicerLayer::connectOpenPolylinesImpl(Polygons& open_polylines, coord_t max_dist, coord_t cell_size, bool allow_reverse)
{
    // below code closes smallest gaps first

    struct PossibleStitch
    {
        int64_t dist2;
        // polyline_idx*2 + {0 | front, 1 | back}
        unsigned int terminus_0_idx;
        unsigned int terminus_1_idx;

        bool in_order() const
        {
            // in order if using back of line 1 and front of line 2
            return (terminus_0_idx & 1) &&
                !(terminus_1_idx & 1);
        }

        // priority_queue will give greatest first so greatest
        // must be most desirable stitch
        bool operator<(const PossibleStitch &other) const
        {
            if (dist2 > other.dist2)
            {
                return true;
            }
            else if (dist2 < other.dist2)
            {
                return false;
            }

            // use in order connections before reverse connections
            if (!in_order() && other.in_order())
            {
                return true;
            }

            if (terminus_0_idx > other.terminus_0_idx)
            {
                return true;
            }
            else if (terminus_0_idx < other.terminus_0_idx)
            {
                return false;
            }

            if (terminus_1_idx > other.terminus_1_idx)
            {
                return true;
            }
            else if (terminus_1_idx < other.terminus_1_idx)
            {
                return false;
            }

            return false;
        }
    };

    std::priority_queue<PossibleStitch> stitch_queue;

    {
        int64_t max_dist2 = max_dist * max_dist;

        struct StitchGridVal
        {
            unsigned int polyline_idx;
            Point polyline_end;
        };

        struct StitchGridValPointAccess
        {
            Point operator()(const StitchGridVal &val) const
            {
                return val.polyline_end;
            }
        };

        SparseGrid<StitchGridVal,StitchGridValPointAccess> grid(cell_size);

        // populate grid
        for(unsigned int polyline_0_idx = 0; polyline_0_idx < open_polylines.size(); polyline_0_idx++)
        {
            PolygonRef polyline_0 = open_polylines[polyline_0_idx];

            if (polyline_0.size() < 1) continue;

            StitchGridVal grid_val;
            grid_val.polyline_idx = polyline_0_idx;
            grid_val.polyline_end = polyline_0.back();
            grid.insert(grid_val);
        }

        // search for nearby end points
        for(unsigned int polyline_1_idx = 0; polyline_1_idx < open_polylines.size(); polyline_1_idx++)
        {
            PolygonRef polyline_1 = open_polylines[polyline_1_idx];
            
            if (polyline_1.size() < 1) continue;

            std::vector<StitchGridVal> nearby_ends;
            nearby_ends = grid.getNearby(polyline_1[0], max_dist);
            for (const auto &nearby_end : nearby_ends)
            {
                Point diff = nearby_end.polyline_end - polyline_1[0];
                int64_t dist2 = vSize2(diff);
                if (dist2 < max_dist2)
                {
                    PossibleStitch poss_stitch;
                    poss_stitch.dist2 = dist2;
                    poss_stitch.terminus_0_idx = nearby_end.polyline_idx*2 + 1;
                    poss_stitch.terminus_1_idx = polyline_1_idx*2;
                    stitch_queue.push(poss_stitch);
                }
            }

            if (allow_reverse)
            {
                nearby_ends = grid.getNearby(polyline_1.back(), max_dist);
                for (const auto &nearby_end : nearby_ends)
                {
                    if (nearby_end.polyline_idx == polyline_1_idx)
                    {
                        continue;
                    }

                    Point diff = nearby_end.polyline_end - polyline_1.back();
                    int64_t dist2 = vSize2(diff);
                    if (dist2 < max_dist2)
                    {
                        PossibleStitch poss_stitch;
                        poss_stitch.dist2 = dist2;
                        poss_stitch.terminus_0_idx = nearby_end.polyline_idx*2 + 1;
                        poss_stitch.terminus_1_idx = polyline_1_idx*2 + 1;
                        stitch_queue.push(poss_stitch);
                    }
                }
            }
        }
    }

    static const unsigned int INVALID_TERMINUS = ~0U;
    size_t terminus_update_map_size = open_polylines.size()*2;
    // map from old terminus index to current terminus index
    std::vector<unsigned int> terminus_old_to_cur_map(terminus_update_map_size);
    // map from current terminus index to old terminus index
    std::vector<unsigned int> terminus_cur_to_old_map(terminus_update_map_size);
    for (size_t idx=0U; idx!=terminus_update_map_size; ++idx)
    {
        terminus_old_to_cur_map[idx] = idx;
    }
    terminus_cur_to_old_map = terminus_old_to_cur_map;
    while(!stitch_queue.empty())
    {
        PossibleStitch next_stitch;
        next_stitch = stitch_queue.top();
        stitch_queue.pop();
        unsigned int old_terminus_0_idx = next_stitch.terminus_0_idx;
        unsigned int terminus_0_idx = terminus_old_to_cur_map[old_terminus_0_idx];
        if (terminus_0_idx == INVALID_TERMINUS)
        {
            // if we already used this terminus, then this stitch is no longer usable
            continue;
        }
        unsigned int old_terminus_1_idx = next_stitch.terminus_1_idx;
        unsigned int terminus_1_idx = terminus_old_to_cur_map[old_terminus_1_idx];
        if (terminus_1_idx == ~0U)
        {
            // if we already used this terminus, then this stitch is no longer usable
            continue;
        }

        unsigned int best_polyline_0_idx = terminus_0_idx/2;
        unsigned int best_polyline_1_idx = terminus_1_idx/2;

        bool completed_poly = best_polyline_0_idx == best_polyline_1_idx;
        if (completed_poly)
        {
            // finished polygon
            PolygonRef polyline_0 = open_polylines[best_polyline_0_idx];
            polygons.add(polyline_0);
            polyline_0.clear();
            unsigned int cur_terms[2] = {best_polyline_0_idx*2 + 0,
                                         best_polyline_0_idx*2 + 1};
            unsigned int old_terms[2];
            for (size_t idx=0U; idx!=2U; ++idx)
            {
                old_terms[idx] = terminus_cur_to_old_map[cur_terms[idx]];
            }
            for (size_t idx=0U; idx!=2U; ++idx)
            {
                terminus_old_to_cur_map[old_terms[idx]] = INVALID_TERMINUS;
                terminus_cur_to_old_map[cur_terms[idx]] = INVALID_TERMINUS;
            }
            continue;
        }

        // plan how to append polygons
        bool back_0 = (terminus_0_idx & 1) == 1;
        bool back_1 = (terminus_1_idx & 1) == 1;
        bool reverse[2] = {false, false};
        if (back_0)
        {
            if (back_1)
            {
                if (open_polylines[best_polyline_0_idx].size() <
                    open_polylines[best_polyline_1_idx].size())
                {
                   std::swap(terminus_0_idx,terminus_1_idx);
                }
                reverse[1] = true;
            } else {
                // nothing to do
            }
        } else {
            if (back_1)
            {
                std::swap(terminus_0_idx,terminus_1_idx);
            }
            else
            {
                reverse[0] = true;
            }
        }

        best_polyline_0_idx = terminus_0_idx/2;
        best_polyline_1_idx = terminus_1_idx/2;
        PolygonRef polyline_0 = open_polylines[best_polyline_0_idx];
        PolygonRef polyline_1 = open_polylines[best_polyline_1_idx];

        // append polygons according to plan
        if (reverse[0])
        {
            // reverse polyline_0
            size_t size_0 = polyline_0.size();
            for (size_t idx=0U; idx!=size_0/2; ++idx)
            {
                std::swap(polyline_0[idx], polyline_0[size_0-1-idx]);
            }
        }
        if (reverse[1])
        {
            for(int poly_idx = polyline_1.size()-1; poly_idx >= 0; poly_idx--)
                polyline_0.add(polyline_1[poly_idx]);
            polyline_1.clear();
        }
        else
        {
            for(Point& p : polyline_1)
                polyline_0.add(p);
            polyline_1.clear();
        }

        // update terminus_update_map
        unsigned int cur_terms[4] = {best_polyline_0_idx*2 + 0,
                                     best_polyline_0_idx*2 + 1,
                                     best_polyline_1_idx*2 + 0,
                                     best_polyline_1_idx*2 + 1};
        unsigned int next_terms[4] = {best_polyline_0_idx*2 + 0,
                                      INVALID_TERMINUS,
                                      INVALID_TERMINUS,
                                      best_polyline_0_idx*2 + 1};
        if (reverse[0])
        {
            std::swap(next_terms[0],next_terms[1]);
        }
        if (reverse[1])
        {
            std::swap(next_terms[2],next_terms[3]);
        }
        unsigned int old_terms[4];
        for (size_t idx=0U; idx!=4U; ++idx)
        {
            old_terms[idx] = terminus_cur_to_old_map[cur_terms[idx]];
        }
        for (size_t idx=0U; idx!=4U; ++idx)
        {
            terminus_old_to_cur_map[old_terms[idx]] = next_terms[idx];
            unsigned int next_term_idx = next_terms[idx];
            if (next_term_idx!=INVALID_TERMINUS)
            {
                terminus_cur_to_old_map[next_term_idx] = old_terms[idx];
            }
        }
        terminus_cur_to_old_map[cur_terms[2]] = INVALID_TERMINUS;
        terminus_cur_to_old_map[cur_terms[3]] = INVALID_TERMINUS;
    }
}

void SlicerLayer::stitch_extensive(Polygons& open_polylines)
{
    //For extensive stitching find 2 open polygons that are touching 2 closed polygons.
    // Then find the shortest path over this polygon that can be used to connect the open polygons,
    // And generate a path over this shortest bit to link up the 2 open polygons.
    // (If these 2 open polygons are the same polygon, then the final result is a closed polyon)
    
    while(1)
    {
        unsigned int best_polyline_1_idx = -1;
        unsigned int best_polyline_2_idx = -1;
        GapCloserResult best_result;
        best_result.len = POINT_MAX;
        best_result.polygonIdx = -1;
        best_result.pointIdxA = -1;
        best_result.pointIdxB = -1;
        
        for(unsigned int polyline_1_idx = 0; polyline_1_idx < open_polylines.size(); polyline_1_idx++)
        {
            PolygonRef polyline_1 = open_polylines[polyline_1_idx];
            if (polyline_1.size() < 1) continue;
            
            {
                GapCloserResult res = findPolygonGapCloser(polyline_1[0], polyline_1.back());
                if (res.len > 0 && res.len < best_result.len)
                {
                    best_polyline_1_idx = polyline_1_idx;
                    best_polyline_2_idx = polyline_1_idx;
                    best_result = res;
                }
            }

            for(unsigned int polyline_2_idx = 0; polyline_2_idx < open_polylines.size(); polyline_2_idx++)
            {
                PolygonRef polyline_2 = open_polylines[polyline_2_idx];
                if (polyline_2.size() < 1 || polyline_1_idx == polyline_2_idx) continue;
                
                GapCloserResult res = findPolygonGapCloser(polyline_1[0], polyline_2.back());
                if (res.len > 0 && res.len < best_result.len)
                {
                    best_polyline_1_idx = polyline_1_idx;
                    best_polyline_2_idx = polyline_2_idx;
                    best_result = res;
                }
            }
        }
        
        if (best_result.len < POINT_MAX)
        {
            if (best_polyline_1_idx == best_polyline_2_idx)
            {
                if (best_result.pointIdxA == best_result.pointIdxB)
                {
                    polygons.add(open_polylines[best_polyline_1_idx]);
                    open_polylines[best_polyline_1_idx].clear();
                }
                else if (best_result.AtoB)
                {
                    PolygonRef poly = polygons.newPoly();
                    for(unsigned int j = best_result.pointIdxA; j != best_result.pointIdxB; j = (j + 1) % polygons[best_result.polygonIdx].size())
                        poly.add(polygons[best_result.polygonIdx][j]);
                    for(unsigned int j = open_polylines[best_polyline_1_idx].size() - 1; int(j) >= 0; j--)
                        poly.add(open_polylines[best_polyline_1_idx][j]);
                    open_polylines[best_polyline_1_idx].clear();
                }
                else
                {
                    unsigned int n = polygons.size();
                    polygons.add(open_polylines[best_polyline_1_idx]);
                    for(unsigned int j = best_result.pointIdxB; j != best_result.pointIdxA; j = (j + 1) % polygons[best_result.polygonIdx].size())
                        polygons[n].add(polygons[best_result.polygonIdx][j]);
                    open_polylines[best_polyline_1_idx].clear();
                }
            }
            else
            {
                if (best_result.pointIdxA == best_result.pointIdxB)
                {
                    for(unsigned int n=0; n<open_polylines[best_polyline_1_idx].size(); n++)
                        open_polylines[best_polyline_2_idx].add(open_polylines[best_polyline_1_idx][n]);
                    open_polylines[best_polyline_1_idx].clear();
                }
                else if (best_result.AtoB)
                {
                    Polygon poly;
                    for(unsigned int n = best_result.pointIdxA; n != best_result.pointIdxB; n = (n + 1) % polygons[best_result.polygonIdx].size())
                        poly.add(polygons[best_result.polygonIdx][n]);
                    for(unsigned int n=poly.size()-1;int(n) >= 0; n--)
                        open_polylines[best_polyline_2_idx].add(poly[n]);
                    for(unsigned int n=0; n<open_polylines[best_polyline_1_idx].size(); n++)
                        open_polylines[best_polyline_2_idx].add(open_polylines[best_polyline_1_idx][n]);
                    open_polylines[best_polyline_1_idx].clear();
                }
                else
                {
                    for(unsigned int n = best_result.pointIdxB; n != best_result.pointIdxA; n = (n + 1) % polygons[best_result.polygonIdx].size())
                        open_polylines[best_polyline_2_idx].add(polygons[best_result.polygonIdx][n]);
                    for(unsigned int n = open_polylines[best_polyline_1_idx].size() - 1; int(n) >= 0; n--)
                        open_polylines[best_polyline_2_idx].add(open_polylines[best_polyline_1_idx][n]);
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

GapCloserResult SlicerLayer::findPolygonGapCloser(Point ip0, Point ip1)
{
    GapCloserResult ret;
    ClosePolygonResult c1 = findPolygonPointClosestTo(ip0);
    ClosePolygonResult c2 = findPolygonPointClosestTo(ip1);
    if (c1.polygonIdx < 0 || c1.polygonIdx != c2.polygonIdx)
    {
        ret.len = -1;
        return ret;
    }
    ret.polygonIdx = c1.polygonIdx;
    ret.pointIdxA = c1.pointIdx;
    ret.pointIdxB = c2.pointIdx;
    ret.AtoB = true;
    
    if (ret.pointIdxA == ret.pointIdxB)
    {
        //Connection points are on the same line segment.
        ret.len = vSize(ip0 - ip1);
    }else{
        //Find out if we have should go from A to B or the other way around.
        Point p0 = polygons[ret.polygonIdx][ret.pointIdxA];
        int64_t lenA = vSize(p0 - ip0);
        for(unsigned int i = ret.pointIdxA; i != ret.pointIdxB; i = (i + 1) % polygons[ret.polygonIdx].size())
        {
            Point p1 = polygons[ret.polygonIdx][i];
            lenA += vSize(p0 - p1);
            p0 = p1;
        }
        lenA += vSize(p0 - ip1);

        p0 = polygons[ret.polygonIdx][ret.pointIdxB];
        int64_t lenB = vSize(p0 - ip1);
        for(unsigned int i = ret.pointIdxB; i != ret.pointIdxA; i = (i + 1) % polygons[ret.polygonIdx].size())
        {
            Point p1 = polygons[ret.polygonIdx][i];
            lenB += vSize(p0 - p1);
            p0 = p1;
        }
        lenB += vSize(p0 - ip0);
        
        if (lenA < lenB)
        {
            ret.AtoB = true;
            ret.len = lenA;
        }else{
            ret.AtoB = false;
            ret.len = lenB;
        }
    }
    return ret;
}

ClosePolygonResult SlicerLayer::findPolygonPointClosestTo(Point input)
{
    ClosePolygonResult ret;
    for(unsigned int n=0; n<polygons.size(); n++)
    {
        Point p0 = polygons[n][polygons[n].size()-1];
        for(unsigned int i=0; i<polygons[n].size(); i++)
        {
            Point p1 = polygons[n][i];
            
            //Q = A + Normal( B - A ) * ((( B - A ) dot ( P - A )) / VSize( A - B ));
            Point pDiff = p1 - p0;
            int64_t lineLength = vSize(pDiff);
            if (lineLength > 1)
            {
                int64_t distOnLine = dot(pDiff, input - p0) / lineLength;
                if (distOnLine >= 0 && distOnLine <= lineLength)
                {
                    Point q = p0 + pDiff * distOnLine / lineLength;
                    if (shorterThen(q - input, 100))
                    {
                        ret.intersectionPoint = q;
                        ret.polygonIdx = n;
                        ret.pointIdx = i;
                        return ret;
                    }
                }
            }
            p0 = p1;
        }
    }
    ret.polygonIdx = -1;
    return ret;
}

void SlicerLayer::makePolygons(const Mesh* mesh, bool keep_none_closed, bool extensive_stitching)
{
    Polygons open_polylines;
    
    makeBasicPolygonLoops(mesh, open_polylines);
    
    connectOpenPolylines(open_polylines);
    
    // TODO: (?) for mesh surface mode: connect open polygons. Maybe the above algorithm can create two open polygons which are actually connected when the starting segment is in the middle between the two open polygons.

    if (mesh->getSettingAsSurfaceMode("magic_mesh_surface_mode") == ESurfaceMode::NORMAL)
    { // don't stitch when using (any) mesh surface mode, i.e. also don't stitch when using mixed mesh surface and closed polygons, because then polylines which are supposed to be open will be closed
        stitch(open_polylines);
    }
    
    if (extensive_stitching)
    {
        stitch_extensive(open_polylines);
    }

    if (keep_none_closed)
    {
        for (PolygonRef polyline : open_polylines)
        {
            if (polyline.size() > 0)
                openPolylines.add(polyline);
        }
    }
    
    for (PolygonRef polyline : open_polylines)
    {
        if (polyline.size() > 0)
            openPolylines.add(polyline);
    }

    //Remove all the tiny polygons, or polygons that are not closed. As they do not contribute to the actual print.
    int snapDistance = MM2INT(1.0); // TODO: hardcoded value
    auto it = std::remove_if(polygons.begin(), polygons.end(), [snapDistance](PolygonRef poly) { return poly.shorterThan(snapDistance); });
    polygons.erase(it, polygons.end());

    //Finally optimize all the polygons. Every point removed saves time in the long run.
    polygons.simplify();
    
    polygons.removeDegenerateVerts(); // remove verts connected to overlapping line segments
    
    int xy_offset = mesh->getSettingInMicrons("xy_offset");
    if (xy_offset != 0)
    {
        polygons = polygons.offset(xy_offset);
    }
}


Slicer::Slicer(const Mesh* mesh, int initial, int thickness, int layer_count, bool keep_none_closed, bool extensive_stitching)
: mesh(mesh)
{
    assert(layer_count > 0);

    TimeKeeper slice_timer;

    layers.resize(layer_count);
    
    for(int32_t layer_nr = 0; layer_nr < layer_count; layer_nr++)
    {
        layers[layer_nr].z = initial + thickness * layer_nr;
    }
    
    for(unsigned int mesh_idx = 0; mesh_idx < mesh->faces.size(); mesh_idx++)
    {
        const MeshFace& face = mesh->faces[mesh_idx];
        const MeshVertex& v0 = mesh->vertices[face.vertex_index[0]];
        const MeshVertex& v1 = mesh->vertices[face.vertex_index[1]];
        const MeshVertex& v2 = mesh->vertices[face.vertex_index[2]];
        Point3 p0 = v0.p;
        Point3 p1 = v1.p;
        Point3 p2 = v2.p;
        int32_t minZ = p0.z;
        int32_t maxZ = p0.z;
        if (p1.z < minZ) minZ = p1.z;
        if (p2.z < minZ) minZ = p2.z;
        if (p1.z > maxZ) maxZ = p1.z;
        if (p2.z > maxZ) maxZ = p2.z;
        int32_t layer_max = (maxZ - initial) / thickness;
        for(int32_t layer_nr = (minZ - initial) / thickness; layer_nr <= layer_max; layer_nr++)
        {
            int32_t z = layer_nr * thickness + initial;
            if (z < minZ) continue;
            if (layer_nr < 0) continue;
            
            SlicerSegment s;
            int end_edge_start_pt = -1;
            if (p0.z < z && p1.z >= z && p2.z >= z)
            {
                s = project2D(p0, p2, p1, z);
                end_edge_start_pt = 0;
                if (p1.z == z)
                {
                    s.endOtherFaces = v1.connected_faces;
                }
            }
            else if (p0.z > z && p1.z < z && p2.z < z)
            {
                s = project2D(p0, p1, p2, z);
                end_edge_start_pt = 2;

            }

            else if (p1.z < z && p0.z >= z && p2.z >= z)
            {
                s = project2D(p1, p0, p2, z);
                end_edge_start_pt = 1;
                if (p2.z == z)
                {
                    s.endOtherFaces = v2.connected_faces;
                }
            }
            else if (p1.z > z && p0.z < z && p2.z < z)
            {
                s = project2D(p1, p2, p0, z);
                end_edge_start_pt = 0;

            }

            else if (p2.z < z && p1.z >= z && p0.z >= z)
            {
                s = project2D(p2, p1, p0, z);
                end_edge_start_pt = 2;
                if (p0.z == z)
                {
                    s.endOtherFaces = v0.connected_faces;
                }
            }
            else if (p2.z > z && p1.z < z && p0.z < z)
            {
                s = project2D(p2, p0, p1, z);
                end_edge_start_pt = 1;
            }
            else
            {
                //Not all cases create a segment, because a point of a face could create just a dot, and two touching faces
                //  on the slice would create two segments
                continue;
            }
            layers[layer_nr].face_idx_to_segment_idx.insert(std::make_pair(mesh_idx, layers[layer_nr].segments.size()));
            s.faceIndex = mesh_idx;
            s.endOtherFaceIdx = face.connected_face_index[end_edge_start_pt];
            s.addedToPolygon = false;
            layers[layer_nr].segments.push_back(s);
        }
    }
    log("slice of mesh took %.3f seconds\n",slice_timer.restart());
    for(unsigned int layer_nr=0; layer_nr<layers.size(); layer_nr++)
    {
        layers[layer_nr].makePolygons(mesh, keep_none_closed, extensive_stitching);
    }
    log("slice make polygons took %.3f seconds\n",slice_timer.restart());
}

}//namespace cura

/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SLICER_H
#define SLICER_H

#include "mesh.h"
#include "utils/polygon.h"
/*
    The Slicer creates layers of polygons from an optimized 3D model.
    The result of the Slicer is a list of polygons without any order or structure.
*/
namespace cura {

class SlicerSegment
{
public:
    Point start, end;
    int faceIndex;
    bool addedToPolygon;
};

class ClosePolygonResult
{   //The result of trying to find a point on a closed polygon line. This gives back the point index, the polygon index, and the point of the connection.
    //The line on which the point lays is between pointIdx-1 and pointIdx
public:
    Point intersectionPoint;
    int polygonIdx;
    unsigned int pointIdx;
};
class GapCloserResult
{
public:
    int64_t len;
    int polygonIdx;
    unsigned int pointIdxA;
    unsigned int pointIdxB;
    bool AtoB;
};

class SlicerLayer
{
public:
    std::vector<SlicerSegment> segments;
    std::unordered_map<int, int> face_idx_to_segment_idx; // topology
    
    int z;
    Polygons polygons;
    Polygons openPolylines;
    
    void makePolygons(Mesh* mesh, bool keepNoneClosed, bool extensiveStitching);

    /*!
     * 
     * \param open_polylines (output parameter)
     */
    void makeBasicPolygonLoops(Mesh* mesh, Polygons& open_polylines);
    
    /*!
     * \param open_polylines (output parameter)
     */
    void makeBasicPolygonLoop(Mesh* mesh, Polygons& open_polylines, unsigned int start_segment_idx);
    
    int getNextSegmentIdx(Mesh* mesh, SlicerSegment& segment, unsigned int start_segment_idx);
    
    /*!
     * Connecting polygons that are not closed yet, as models are not always perfect manifold we need to join some stuff up to get proper polygons.
     * First link up polygon ends that are within 2 microns.
     * 
     */
    void connectOpenPolylines(Polygons& open_polylines);
    
    /*!
     * Link up all the missing ends, closing up the smallest gaps first. This is an inefficient implementation which can run in O(n*n*n) time.
     */
    void stitch(Polygons open_polylines);
    
private:
    GapCloserResult findPolygonGapCloser(Point ip0, Point ip1);

    ClosePolygonResult findPolygonPointClosestTo(Point input);
    
    void stitch_extensive(Polygons open_polylines);
};

class Slicer
{
public:
    std::vector<SlicerLayer> layers;
    
    Slicer(Mesh* mesh, int initial, int thickness, int layer_count, bool keepNoneClosed, bool extensiveStitching);
    
    SlicerSegment project2D(Point3& p0, Point3& p1, Point3& p2, int32_t z) const
    {
        SlicerSegment seg;
        seg.start.X = p0.x + int64_t(p1.x - p0.x) * int64_t(z - p0.z) / int64_t(p1.z - p0.z);
        seg.start.Y = p0.y + int64_t(p1.y - p0.y) * int64_t(z - p0.z) / int64_t(p1.z - p0.z);
        seg.end.X = p0.x + int64_t(p2.x - p0.x) * int64_t(z - p0.z) / int64_t(p2.z - p0.z);
        seg.end.Y = p0.y + int64_t(p2.y - p0.y) * int64_t(z - p0.z) / int64_t(p2.z - p0.z);
        return seg;
    }
    
    void dumpSegmentsToHTML(const char* filename);
};

}//namespace cura

#endif//SLICER_H

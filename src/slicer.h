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

    /*!
     * Connect the segments into polygons for this layer of this \p mesh
     * 
     * \param[in] mesh The mesh data for which we are connecting sliced segments (The face data is used)
     * \param keepNoneClosed Whether to throw away the data for segments which we couldn't stitch into a polygon
     * \param extensiveStitching Whether to perform extra work to try and close polylines into polygons when there are large gaps
     */
    void makePolygons(const Mesh* mesh, bool keepNoneClosed, bool extensiveStitching);

protected:
    /*!
     * Connect the segments into loops which correctly form polygons (don't perform stitching here)
     * 
     * \param[in] mesh The mesh data for which we are connecting sliced segments (The face data is used)
     * \param[out] open_polylines The polylines which are stiched, but couldn't be closed into a loop
     */
    void makeBasicPolygonLoops(const Mesh* mesh, Polygons& open_polylines);

    /*!
     * Connect the segments into a loop, starting from the segment with index \p start_segment_idx
     * 
     * \param[in] mesh The mesh data for which we are connecting sliced segments (The face data is used)
     * \param[out] open_polylines The polylines which are stiched, but couldn't be closed into a loop
     * \param[in] start_segment_idx The index into SlicerLayer::segments for the first segment from which to start the polygon loop
     */
    void makeBasicPolygonLoop(const Mesh* mesh, Polygons& open_polylines, unsigned int start_segment_idx);

    /*!
     * Get the next segment connected to the end of \p segment.
     * Used to make closed polygon loops.
     * Return ASAP if segment is (also) connected to SlicerLayer::segments[\p start_segment_idx]
     * 
     * \param[in] mesh The mesh data for which we are connecting sliced segments (The face data is used)
     * \param[in] segment The segment from which to start looking for the next
     * \param[in] start_segment_idx The index to the segment which when conected to \p segment will immediately stop looking for further candidates.
     */
    int getNextSegmentIdx(const Mesh* mesh, const SlicerSegment& segment, unsigned int start_segment_idx);

    /*!
     * Connecting polygons that are not closed yet, as models are not always perfect manifold we need to join some stuff up to get proper polygons.
     * First link up polygon ends that are within 2 microns.
     * 
     * Clears all open polylines which are used up in the process
     * 
     * \param[in,out] open_polylines The polylines which are stiched, but couldn't be closed into a loop
     */
    void connectOpenPolylines(Polygons& open_polylines);

    /*!
     * Link up all the missing ends, closing up the smallest gaps first. This is an inefficient implementation which can run in O(n*n*n) time.
     * 
     * Clears all open polylines which are used up in the process
     * 
     * \param[in,out] open_polylines The polylines which are stiched, but couldn't be closed into a loop yet
     */
    void stitch(Polygons& open_polylines);

    GapCloserResult findPolygonGapCloser(Point ip0, Point ip1);

    ClosePolygonResult findPolygonPointClosestTo(Point input);

    /*!
     * Try to close up polylines into polygons while they have large gaps in them.
     * 
     * Clears all open polylines which are used up in the process
     * 
     * \param[in,out] open_polylines The polylines which are stiched, but couldn't be closed into a loop yet
     */
    void stitch_extensive(Polygons& open_polylines);

private:
    /*!
     * Connecting polylines that are not closed yet.
     *
     * Any polylines that are closed by this function are added to this->polygons.
     *
     * \param[in,out] open_polylines The polylines which couldn't be closed into a loop
     * \param[in] max_dist The maximum distance that polyline ends can be separated and still be joined.
     * \param[in] cell_size The cell size to use internally in the grid.  This affects speed but not results.
     * \param[in] allow_reverse If true, then this function is allowed to reverse edge directions to merge polylines.
     */
    void connectOpenPolylinesImpl(Polygons& open_polylines, coord_t max_dist, coord_t cell_size, bool allow_reverse);
};

class Slicer
{
public:
    std::vector<SlicerLayer> layers;

    const Mesh* mesh; //!< The sliced mesh
    
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

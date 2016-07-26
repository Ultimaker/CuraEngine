/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef SLICER_H
#define SLICER_H

#include <queue>

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
    // The index of the other face connected via the edge that created end
    int endOtherFaceIdx;
    // If end corresponds to a vertex of the mesh, then this is populated
    // with the vertex that it ended on.
    const MeshVertex *endVertex;
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
     * \brief This class represents the location of an end point of a
     * polyline in a polyline vector.
     *
     * The location records the index in the polyline vector and
     * whether this is the vertex at the start of the polyline or the
     * vertex at the end.
     */
    class Terminus
    {
    public:
        /*! A representation of Terminus that can be used as an array index.
         *
         * See \ref asIndex() for more information.
         */
        using Index = size_t;

        /*! A Terminus value representing an invalid value.
         *
         * This is used to record when Terminus are removed.
         */
        static const Terminus INVALID_TERMINUS;

        /*! Constructor leaving uninitialized. */
        Terminus()
        {}

        /*! Constructor from Index representation.
         *
         * Terminus{t.asIndex()} == t for all Terminus t.
         */
        Terminus(Index idx)
        {
            m_idx = idx;
        }

        /*! Constuctor from the polyline index and which end of the polyline.
         *
         * Terminus{t.getPolylineIdx(), t.isEnd()} == t for all Terminus t.
         */
        Terminus(size_t polyline_idx, bool is_end)
        {
            m_idx = polyline_idx * 2  + (is_end ? 1 : 0);
        }

        /*! Gets the polyline index for this Terminus. */
        size_t getPolylineIdx() const
        {
            return m_idx / 2;
        }

        /*! Gets whether this Terminus represents the end point of the polyline. */
        bool isEnd() const
        {
            return (m_idx & 1) == 1;
        }

        /*! Gets the Index representation of this Terminus.
         *
         * The index representation much satisfy the following:
         * 1. for all Terminus t0, t1: t0 == t1 implies t0.asIndex() == t1.asIndex()
         * 2. for all Terminus t0, t1: t0 != t1 implies t0.asIndex() != t1.asIndex()
         * 3. t0.asIndex() >= 0
         * 4. if y = \ref endIndexFromPolylineEndIndex(x), then for all Terminus t
         *       if t.getPolylineIdx() < x then t.asIndex() < y
         *
         * In addition, the Index representation should be reasonably
         * compact for efficiency.  This means that for polyline index
         * in [0,x) and Terminus t with t.getPolylineIdx() < x, the
         * set of containing all t.asIndex() union {0} should be
         * small.  In other words, t.asIndex() should map to [0,y)
         * where y is as small as possible.
         */
        Index asIndex() const
        {
            return m_idx;
        }

        /*! Calculates the Terminus end Index from the polyline vector end index.
         *
         * \param[in] polyline_end_idx The index of the first invalid
         *     element of the polyline vector.
         * \return The Index for the first invalid Terminus for the polyline
         *     vector.
         */
        static Index endIndexFromPolylineEndIndex(unsigned int polyline_end_idx)
        {
            return polyline_end_idx*2;
        }

        /*! Tests for equality.
        *
        * Two Terminus are equal if they return the same results for
        * \ref getPolylineIdx() and \ref isEnd().
        */
        bool operator==(const Terminus &other)
        {
            return m_idx == other.m_idx;
        }

        /*! Tests for inequality. */
        bool operator!=(const Terminus &other)
        {
            return m_idx != other.m_idx;
        }

    private:
        /*! The Index representation of the Terminus.
         *
         * The polyline_idx and end flags are calculated from this on demand.
         */
        Index m_idx;
    };

    /*!
     * \brief Represents a possible stitch between two polylines.
     *
     * This represents the possibility of creating a new merged
     * polyline from appending terminus_1.getPolylineIdx() onto
     * terminus_0.getPolylineIdx() using the Terminus points as the
     * join point.  Consider polylines A -> B and C -> D.  If
     * terminus_0 is B and terminus_1 is C, then this stitch
     * represents A -> B -> C -> D.  If terminus_0 is C and terminus_1
     * is A, then this stitch represents D -> C -> A -> B.  In
     * general, this stitch represents the polyline:
     *   the other terminus of polyline 0 -> terminus_0 -> terminus_1
     *     -> the other terminus of polyline 1.
     *
     * This class also stores the squared distance involved in making
     * the stitch.
     */
    struct PossibleStitch
    {
        /*! Squared distance from terminus_0 to terminus_1. */
        int64_t dist2;
        /*! The Terminus representing the end of polyline_0 where the
         * join would happen. */
        Terminus terminus_0;
        /*! The Terminus representing the end of polyline_1 where the
         * join would happen. */
        Terminus terminus_1;

        /*! True if this stitch doesn't require any polyline reversals.
         *
         * If this is true, then the polylines can be appended using
         * their natural order.
         */
        bool in_order() const
        {
            // in order if using back of line 0 and front of line 1
            return terminus_0.isEnd() &&
                !terminus_1.isEnd();
        }

        /*! Orders PossibleStitch by goodness.
         *
         * Better PossibleStitch are > then worse PossibleStitch.
         * priority_queue will give greatest first so greatest
         * must be most desirable stitch
        */
        bool operator<(const PossibleStitch &other) const;
    };

    /*!
     * \brief Tracks movements of polyline end point locations (Terminus).
     *
     * Tracks the movement of polyline end point locations within the
     * polyline vector as polylines are joined, reversed, and used to
     * form polygons.
     */
    class TerminusTrackingMap
    {
    public:
        /*! Initializes the TerminusTrackingMap with the size indicated.
         *
         * \param end_idx The first invalid Terminus::Index.  This usually
         *    comes from \ref Terminus::endIndexFromPolylineEndIndex().
         */
        TerminusTrackingMap(Terminus::Index end_idx);

        /*! Given the old Terminus location returns the current location.
         *
         * If the old location is no longer the endpoint of a polyline
         * in the polyline vector, then this returns
         * Terminus::INVALID_TERMINUS.  As long as the old location is
         * still an endpoint in the polyline vector, then
         * getCurFromOld(old) will always refer to the same point.
         * Endpoints are removed from the polyline vector as polylines
         * are merged or converted to Polygons.
         *
         * \param old The old Terminus location.  Must not be
         *     INVALID_TERMINUS.
         * \return The current Terminus location or INVALID_TERMINUS
         *     if the old endpoint is no longer an endpoint.
         */
        Terminus getCurFromOld(const Terminus &old) const
        {
            return m_terminus_old_to_cur_map[old.asIndex()];
        }

        /*! Given the current Terminus location returns the old location.
         *
         * \param cur The current Terminus location.  Must not be
         *     INVALID_TERMINUS.
         * \return The old Terminus location.  Returns
         *     INVALID_TERMINUS if the old Terminus location was
         *     removed (used to form a Polygon).
         */
        Terminus getOldFromCur(const Terminus &cur) const
        {
            return m_terminus_cur_to_old_map[cur.asIndex()];
        }

        /*! Mark the current Terminus as being removed.
         *
         * This marks the current Terminus as being removed from the
         * polyline vector.
         */
        void markRemoved(const Terminus &cur)
        {
            Terminus old = getOldFromCur(cur);
            m_terminus_old_to_cur_map[old.asIndex()] = Terminus::INVALID_TERMINUS;
            m_terminus_cur_to_old_map[cur.asIndex()] = Terminus::INVALID_TERMINUS;
        }

        /*! Update the map for movement of Terminus.
         *
         * This updates the map for the movement / removal of Terminus
         * locations.  next_terms[i] should refer to the same point as
         * cur_terms[i] for i < num_terms, unless the Terminus was
         * removed.  If the Terminus was removed, next_terms[i] should
         * be INVALID_TERMINUS.
         *
         * removed_cur_terms should refer to those Terminus that are
         * no longer present after the update.  removed_cur_terms
         * should be the set of terminus values that are in cur_terms
         * but not in next_terms, i.e. viewing the inputs as sets:
         * removed_cur_terms = next_terms - cur_terms.  It is passed
         * separately to avoid calculating the set difference since
         * the caller generally has this information readily
         * available.
         *
         * \param num_terms The number of Terminus that changed.
         * \param cur_terms The current Terminus locations.  Must be
         *     of size num_terms.  Must not contain INVALID_TERMINUS.
         * \param next_terms The Terminus locations after the update.
         *     Must be of size num_terms.  A value of INVALID_TERMINUS
         *     indicates that the Terminus was removed.
         * \param num_removed_terms The number of Terminus locations
         *     that are being removed by the update.
         * \param removed_cur_terms The Terminus locations that will
         *     be removed after the update.
         */
        void updateMap(size_t num_terms,
                       const Terminus *cur_terms, const Terminus *next_terms,
                       size_t num_removed_terms,
                       const Terminus *removed_cur_terms);

    private:
        /*! map from old terminus location to current terminus location */
        std::vector<Terminus> m_terminus_old_to_cur_map;
        /*! map from current terminus location to old terminus location */
        std::vector<Terminus> m_terminus_cur_to_old_map;
    };

    /*!
     * Try to find a segment from face \p face_idx to continue \p segment.
     *
     * \param[in] mesh The mesh being sliced.
     * \param[in] segment The previous segment that we want to find a continuation for.
     * \param[in] face_idx The index of the face that might have generated a continuation segment.
     * \param[in] start_segment_idx The index of the segment that started this polyline.
     */
    int tryFaceNextSegmentIdx(const Mesh* mesh, const SlicerSegment& segment,
                              int face_idx, unsigned int start_segment_idx) const;

    /*!
     * Find possible allowed stitches in goodness order.
     *
     * This finds all stitches that are allowed by the parameters.
     * The stitches are returned in a priority_queue that returns them
     * in order from best to worst stitch.
     *
     * \param open_polylines The polylines to try to stitch together.
     * \param max_dist The maximum distance between end points for an
     *     allowed stitch.
     * \param cell_size The cell size to use for the SparseGrid.  This
     *     affects speed, but does not otherwise affect the results.
     *     This value should generally be close to max_dist.
     * \param allow_reverse Whether stitches are allowed that reverse
     *     the order of a polyline.
     * \return The stitches that are allowed in order from best to worst.
     */
    std::priority_queue<PossibleStitch> findPossibleStitches(
        const Polygons& open_polylines, coord_t max_dist, coord_t cell_size,
        bool allow_reverse) const;

    /*! Plans the best way to perform a stitch.
     *
     * Let polyline_0 be open_polylines[terminus_0.getPolylineIdx()] and
     *     polyline_1 be open_polylines[terminus_1.getPolylineIdx()].
     *
     * The plan consists of appending polyline_1 to polyline_0.  If
     * reverse[0] is true, then polyline_0 should be reversed before
     * appending.  If reverse[1] is true, then polyline_1 should be
     * reversed before appending.  Note that terminus_0 and terminus_1
     * may be swapped by this function.
     *
     * \param[in] open_polylines The polyline storage vector.
     * \param[in,out] terminus_0 the Terminus on polyline_0 to join at.
     * \param[in,out] terminus_1 the Terminus on polyline_1 to join at.
     * \param[out] reverse Whether the polylines need to be reversed.
     */
    void planPolylineStitch(const Polygons& open_polylines,
                            Terminus& terminus_0, Terminus& terminus_1,
                            bool reverse[2]) const;

    /*! Joins polyline_1 onto polyline_0.
     *
     * Appends polyline_1 to polyline_0.  It reverses the polylines first if either
     * reverse[i] is true.  Clears polyline_1.
     *
     * \param[in,out] polyline_0 On input, the polyline that will form
     *     the first part of the joined polyline.  On output, the
     *     joined polyline.
     * \param[in,out] polyline_1 On input, the polyline that will form
     *     the second of the joined polyline.  On output, an empty
     *     polyline.
     * \param[in] reverse Whether to reverse the polylines before
     *     joining.  reverse[0] indicates whether to reverse
     *     polyline_0 and reverse[1] indicates whether to reverse
     *     polyline_1
     */
    void joinPolylines(PolygonRef& polyline_0, PolygonRef& polyline_1,
                       const bool reverse[2]) const;

    /*!
     * Connecting polylines that are not closed yet.
     *
     * Any polylines that are closed by this function are added to
     * this->polygons.  All possible polyline joins that meet the
     * distance and reversal criteria will be performed.  This
     * function will not introduce any copies of the same polyline
     * segment.
     *
     * \param[in,out] open_polylines The polylines which couldn't be
     *    closed into a loop
     * \param[in] max_dist The maximum distance that polyline ends can
     *     be separated and still be joined.
     * \param[in] cell_size The cell size to use internally in the
     *     grid.  This affects speed but not results.
     * \param[in] allow_reverse If true, then this function is allowed
     *     to reverse edge directions to merge polylines.
     */
    void connectOpenPolylinesImpl(Polygons& open_polylines,
                                  coord_t max_dist, coord_t cell_size,
                                  bool allow_reverse);
};

class Slicer
{
public:
    std::vector<SlicerLayer> layers;

    const Mesh* mesh; //!< The sliced mesh

    Slicer(const Mesh* mesh, int initial, int thickness, int slice_layer_count, bool keepNoneClosed, bool extensiveStitching);

    /*!
     * Linear interpolation
     *
     * Get the Y of a point with X \p x in the line through (\p x0, \p y0) and (\p x1, \p y1)
     */
    int64_t interpolate(int64_t x, int64_t x0, int64_t x1, int64_t y0, int64_t y1) const
    {
        int64_t dx_01 = x1 - x0;
        int64_t num = (y1 - y0) * (x - x0);
        num += num > 0 ? dx_01/2 : -dx_01/2; // add in offset to round result
        int64_t y = y0 + num / dx_01;
        return y;
    }

    SlicerSegment project2D(Point3& p0, Point3& p1, Point3& p2, int32_t z) const
    {
        SlicerSegment seg;

        seg.start.X = interpolate(z, p0.z, p1.z, p0.x, p1.x);
        seg.start.Y = interpolate(z, p0.z, p1.z, p0.y, p1.y);
        seg.end  .X = interpolate(z, p0.z, p2.z, p0.x, p2.x);
        seg.end  .Y = interpolate(z, p0.z, p2.z, p0.y, p2.y);

        return seg;
    }

    void dumpSegmentsToHTML(const char* filename);
};

}//namespace cura

#endif//SLICER_H

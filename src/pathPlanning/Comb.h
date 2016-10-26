/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef PATH_PLANNING_COMB_H
#define PATH_PLANNING_COMB_H

#include <memory> // shared_ptr

#include "../utils/polygon.h"
#include "../utils/SparsePointGridInclusive.h"
#include "../utils/polygonUtils.h"
#include "../utils/LazyInitialization.h"

#include "LinePolygonsCrossings.h"
#include "CombPath.h"
#include "CombPaths.h"

namespace cura 
{

class SliceDataStorage;

/*!
 * Class for generating a full combing actions from a travel move from a start point to an end point.
 * A single Comb object is used for each layer.
 * 
 * Comb::calc is the main function of this class.
 * 
 * Typical output: A combing path to the boundary of the polygon + a move through air avoiding other parts in the layer + a combing path from the boundary of the ending polygon to the end point.
 * Each of these three is a CombPath; the first and last are within Comb::boundary_inside while the middle is outside of Comb::boundary_outside.
 * Between these there is a little gap where the nozzle crosses the boundary of an object approximately perpendicular to its boundary.
 * 
 * As an optimization, the combing paths inside are calculated on specifically those PolygonsParts within which to comb, while the coundary_outside isn't split into outside parts, 
 * because generally there is only one outside part; encapsulated holes occur less often.
 */
class Comb
{
    friend class LinePolygonsCrossings;
private:
    /*!
     * A crossing from the inside boundary to the outside boundary.
     * 
     * 'dest' is either the startPoint or the endpoint of a whole combing move.
     */
    class Crossing
    {
    public:
        bool dest_is_inside; //!< Whether the startPoint or endPoint is inside the inside boundary
        Point in_or_mid; //!< The point on the inside boundary, or in between the inside and outside boundary if the start/end point isn't inside the inside boudary
        Point out; //!< The point on the outside boundary
        PolygonsPart dest_part; //!< The assembled inside-boundary PolygonsPart in which the dest_point lies. (will only be initialized when Crossing::dest_is_inside holds)
        PolygonRef dest_crossing_poly; //!< The polygon of the part in which dest_point lies, which will be crossed (often will be the outside polygon)

        /*!
         * Simple constructor
         * 
         * \param dest_point Either the eventual startPoint or the eventual endPoint of this combing move.
         * \param dest_is_inside Whether the startPoint or endPoint is inside the inside boundary.
         * \param dest_part_idx The index into Comb:partsView_inside of the part in which the \p dest_point is.
         * \param dest_part_boundary_crossing_poly_idx The index in \p boundary_inside of the polygon of the part in which dest_point lies, which will be crossed (often will be the outside polygon).
         * \param boundary_inside The boundary within which to comb.
         */
        Crossing(const Point& dest_point, const bool dest_is_inside, const unsigned int dest_part_idx, const unsigned int dest_part_boundary_crossing_poly_idx, const Polygons& boundary_inside);

        /*!
         * Find the not-outside location (Combing::in_or_mid) of the crossing between to the outside boundary
         * 
         * \param partsView_inside Structured indices onto Comb::boundary_inside which shows which polygons belong to which part. 
         * \param close_to[in] Try to get a crossing close to this point
         */
        void findCrossingInOrMid(const PartsView& partsView_inside, const Point close_to);

        /*!
         * Find the outside location (Combing::out)
         * 
         * \param outside The outside boundary polygons
         * \param close_to A point to get closer to when there are multiple candidates on the outside boundary which are almost equally close to the Crossing::in_or_mid
         * \param fail_on_unavoidable_obstacles When moving over other parts is inavoidable, stop calculation early and return false.
         * \param comber[in] The combing calculator which has references to the offsets and boundaries to use in combing.
         */
        bool findOutside(const Polygons& outside, const Point close_to, const bool fail_on_unavoidable_obstacles, Comb& comber);

    private:
        const Point dest_point; //!< Either the eventual startPoint or the eventual endPoint of this combing move
        unsigned int dest_part_idx; //!< The index into Comb:partsView_inside of the part in which the \p dest_point is.

        /*!
         * Find the best crossing from some inside polygon to the outside boundary.
         * 
         * The detour from \p estimated_start to \p estimated_end is minimized.
         * 
         * \param outside The outside boundary polygons
         * \param from From which inside boundary the crossing to the outside starts or ends
         * \param estimated_start The one point to which to stay close when evaluating crossings which cross about the same distance
         * \param estimated_end The other point to which to stay close when evaluating crossings which cross about the same distance
         * \param comber[in] The combing calculator which has references to the offsets and boundaries to use in combing.
         * \return A pair of which the first is the crossing point on the inside boundary and the second the crossing point on the outside boundary
         */
        std::shared_ptr<std::pair<ClosestPolygonPoint, ClosestPolygonPoint>> findBestCrossing(const Polygons& outside, const PolygonRef from, Point estimated_start, Point estimated_end, Comb& comber);
    };


    SliceDataStorage& storage; //!< The storage from which to compute the outside boundary, when needed.
    const int layer_nr; //!< The layer number for the layer for which to compute the outside boundary, when needed.
    
    const int64_t offset_from_outlines; //!< Offset from the boundary of a part to the comb path. (nozzle width / 2)
    const int64_t max_moveInside_distance2; //!< Maximal distance of a point to the Comb::boundary_inside which is still to be considered inside. (very sharp corners not allowed :S)
    const int64_t offset_from_outlines_outside; //!< Offset from the boundary of a part to a travel path which avoids it by this distance.
    const int64_t offset_from_inside_to_outside; //!< The sum of the offsets for the inside and outside boundary Comb::offset_from_outlines and Comb::offset_from_outlines_outside
    const int64_t max_crossing_dist2; //!< The maximal distance by which to cross the in_between area between inside and outside
    static const int64_t max_moveOutside_distance2 = INT64_MAX; //!< Any point which is not inside should be considered outside.
    static const int64_t offset_dist_to_get_from_on_the_polygon_to_outside = 40; //!< in order to prevent on-boundary vs crossing boundary confusions (precision thing)
    static const int64_t offset_extra_start_end = 100; //!< Distance to move start point and end point toward eachother to extra avoid collision with the boundaries.

    const bool avoid_other_parts; //!< Whether to perform inverse combing a.k.a. avoid parts.
    
    Polygons& boundary_inside; //!< The boundary within which to comb.
    LazyInitialization<Polygons> boundary_outside; //!< The boundary outside of which to stay to avoid collision with other layer parts. This is a pointer cause we only compute it when we move outside the boundary (so not when there is only a single part in the layer)
    LazyInitialization<SparseLineGrid<PolygonsPointIndex, PolygonsPointIndexSegmentLocator>, Comb*, const int64_t> outside_loc_to_line; //!< The SparsePointGridInclusive mapping locations to line segments of the outside boundary.
    PartsView partsView_inside; //!< Structured indices onto boundary_inside which shows which polygons belong to which part. 

    /*!
     * Get the SparsePointGridInclusive mapping locations to line segments of the outside boundary. Calculate it when it hasn't been calculated yet.
     */
    SparseLineGrid<PolygonsPointIndex, PolygonsPointIndexSegmentLocator>& getOutsideLocToLine();

     /*!
      * Get the boundary_outside, which is an offset from the outlines of all meshes in the layer. Calculate it when it hasn't been calculated yet.
      */
    Polygons& getBoundaryOutside();

    /*!
     * Move the startPoint or endPoint inside when it should be inside
     * \param is_inside[in] Whether the \p dest_point should be inside
     * \param dest_point[in,out] The point to move
     * \param start_inside_poly[out] The polygon in which the point has been moved
     * \return Whether we have moved the point inside
     */
    bool moveInside(bool is_inside, Point& dest_point, unsigned int& start_inside_poly);

public:
    /*!
     * Initializes the combing areas for every mesh in the layer (not support)
     * \param storage Where the layer polygon data is stored
     * \param layer_nr The number of the layer for which to generate the combing areas.
     * \param comb_boundary_inside The comb boundary within which to comb within layer parts.
     * \param offset_from_outlines The offset from the outline polygon, to create the combing boundary in case there is no second wall.
     * \param travel_avoid_other_parts Whether to avoid other layer parts when traveling through air.
     * \param travel_avoid_distance The distance by which to avoid other layer parts when traveling through air.
     */
    Comb(SliceDataStorage& storage, int layer_nr, Polygons& comb_boundary_inside, int64_t offset_from_outlines, bool travel_avoid_other_parts, int64_t travel_avoid_distance);
    
    ~Comb();

    /*!
     * Calculate the comb paths (if any) - one for each polygon combed alternated with travel paths
     * 
     * \param startPoint Where to start moving from
     * \param endPoint Where to move to
     * \param combPoints Output parameter: The points along the combing path, excluding the \p startPoint (?) and \p endPoint
     * \param startInside Whether we want to start inside the comb boundary
     * \param endInside Whether we want to end up inside the comb boundary
     * \param via_outside_makes_combing_fail When going through air is inavoidable, stop calculation early and return false.
     * \param fail_on_unavoidable_obstacles When moving over other parts is inavoidable, stop calculation early and return false.
     * \return Whether combing has succeeded; otherwise a retraction is needed.
     */    
    bool calc(Point startPoint, Point endPoint, CombPaths& combPaths, bool startInside, bool endInside, int64_t max_comb_distance_ignored, bool via_outside_makes_combing_fail, bool fail_on_unavoidable_obstacles);
};

}//namespace cura

#endif//PATH_PLANNING_COMB_H

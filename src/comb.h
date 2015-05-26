/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef COMB_H
#define COMB_H

#include "utils/polygon.h"

namespace cura 
{
    
struct CombPath : public  std::vector<Point> //!< A single path either inside or outise the parts
{
    bool throughAir = false;
};
struct CombPaths : public  std::vector<CombPath> //!< A list of paths alternating between inside a part and outside a part
{
}; 

/*!
 * Class for generating a combing move action from point a to point b and avoiding collision with other parts when moving through air.
 * 
 * The general implementation is by rotating everything such that the the line segment from a to b is aligned with the x-axis.
 * We call the line on which a and b lie the 'scanline'.
 */
class LinePolygonsCrossings
{
private:
    
    struct Crossing
    {
        int64_t x; //!< x coordinate of crossings between the polygon and the scanline.
        unsigned int point_idx; 
        Crossing(int64_t x, unsigned int point_idx)
        : x(x), point_idx(point_idx)
        {
        }
    };
    
    struct PolyCrossings
    {
        unsigned int poly_idx; 
        Crossing min;
        Crossing max;
        PolyCrossings(unsigned int poly_idx) 
        : poly_idx(poly_idx)
        , min(INT64_MAX, NO_INDEX), max(INT64_MIN, NO_INDEX) 
        { 
        }
    };

    struct PartCrossings : public std::vector<PolyCrossings>
    {
        //unsigned int part_idx;
    };
    
    int64_t dist_to_move_boundary_point_outside; //!< The distance used to move outside or inside so that a boundary point doesn't intersect with the boundary anymore. Neccesary due to computational rounding problems.
    
    PartCrossings crossings;
    unsigned int min_crossing_idx;
    unsigned int max_crossing_idx;
    
    Polygons& boundary;
    Point startPoint;
    Point endPoint;
    
    PointMatrix transformation_matrix; //!< The transformation which rotates everything such that the scanline is aligned with the x-axis.
    Point transformed_startPoint; //!< The LinePolygonsCrossings::startPoint as transformed by Comb::transformation_matrix
    Point transformed_endPoint; //!< The LinePolygonsCrossings::endPoint as transformed by Comb::transformation_matrix

    
    /*!
     * Check if we are crossing the boundaries, and pre-calculate some values.
     * 
     * Sets Comb::transformation_matrix, Comb::transformed_startPoint and Comb::transformed_endPoint
     * \return Whether the line segment from LinePolygonsCrossings::startPoint to LinePolygonsCrossings::endPoint collides with the boundary
     */
    bool lineSegmentCollidesWithBoundary();
    
    /*!
     * Calculate Comb::crossings, Comb::min_crossing_idx and Comb::max_crossing_idx.
     */
    void calcScanlineCrossings();
    
    /*! 
     * Get the basic combing path and optimize it.
     * 
     * \param offsettedBoundary The boundary which the optimized comb path shouldn't cross
     * \param combPath Output parameter: the points along the combing path.
     */
    void getCombingPath(Polygons& offsettedBoundary, CombPath& combPath);
    
    /*! 
     * Get the basic combing path, without shortcuts. The path goes straight toward the endPoint and follows the boundary when it hits it, until it passes the scanline again.
     * 
     * Walk trough the crossings, for every boundary we cross, find the initial cross point and the exit point. Then add all the points in between
     * to the \p combPath and continue with the next boundary we will cross, until there are no more boundaries to cross.
     * This gives a path from the start to finish curved around the holes that it encounters.
     * 
     * \param combPath Output parameter: the points along the combing path.
     */
    void getBasicCombingPath(CombPath& combPath);
    
    /*! 
     * Get the basic combing path, following a single boundary polygon when it hits it, until it passes the scanline again.
     * 
     * Find the initial cross point and the exit point. Then add all the points in between
     * to the \p combPath and continue with the next boundary we will cross, until there are no more boundaries to cross.
     * This gives a path from the start to finish curved around the polygon that it encounters.
     * 
     * \param combPath Output parameter: where to add the points along the combing path.
     */
    void getBasicCombingPath(PolyCrossings crossings, CombPath& combPath);
    
    /*!
     * Find the first polygon cutting the scanline after \p x.
     * 
     * Note that this function only looks at the first segment cutting the scanline (see Comb::minX)!
     * It doesn't return the next polygon which crosses the scanline, but the first polygon crossing the scanline for the first time.
     * 
     * \param x The point on the scanline from where to look.
     * \return The next PolyCrossings fully beyond \p x or one with PolyCrossings::poly_idx set to NO_INDEX if there's none left.
     */
    PolyCrossings getNextPolygonAlongScanline(int64_t x);
    
    /*!
     * Optimize the \p comb_path: skip each point we could already reach by not crossing a boundary. This smooths out the path and makes it skip some unneeded corners.
     * 
     * \param boundary_offsetted The polygons which not to cross. (Make sure the comb path doesnt lie on the offsettedBoundary)
     * \param comb_path The unoptimized combing path.
     * \param optimized_comb_path Output parameter: The points of optimized combing path
     * \return Whether it turns out that the basic comb path already crossed a boundary
     */
    bool optimizePath(Polygons& boundary_offsetted, CombPath& comb_path, CombPath& optimized_comb_path);
public: 
    
    /*!
     * The main function of this class: calculate one combing path within the boundary.
     * \param boundary The polygons to follow when calculating the basic combing path
     * \param boundary_offsetted The polygons which not to cross. (Make sure the comb path doesnt lie on the offsettedBoundary)
     * \param startPoint From where to start the combing move.
     * \param endPoint Where to end the combing move.
     * \param combPath Output parameter: the combing path generated.
     */
    static void comb(Polygons& boundary, Polygons& boundary_offsetted, Point startPoint, Point endPoint, CombPath& combPath, int64_t dist_to_move_boundary_point_outside)
    {
        LinePolygonsCrossings linePolygonsCrossings(boundary, startPoint, endPoint, dist_to_move_boundary_point_outside);
        linePolygonsCrossings.getCombingPath(boundary_offsetted, combPath);
    };
    
    LinePolygonsCrossings(Polygons& boundary, Point& start, Point& end, int64_t dist_to_move_boundary_point_outside)
    : boundary(boundary), startPoint(start), endPoint(end), dist_to_move_boundary_point_outside(dist_to_move_boundary_point_outside)
    {
    }
};

class SliceDataStorage;

class Comb 
{
    friend class LinePolygonsCrossings;
private:
    Polygons boundary;
    Polygons boundary_inside;
    Polygons* boundary_outside;
    Polygons* boundary_outside_extra_offset;
    PartsView partsView_inside;
    static const int64_t offset_from_outlines = MM2INT(0.2); // TODO: nozzle width / 2 !
    static const int64_t max_moveInside_distance2 = MM2INT(0.4)*MM2INT(0.4); // very sharp corners not allowed :S
    static const int64_t offset_from_outlines_outside = MM2INT(1.0); 
    static const int64_t max_moveOutside_distance2 = MM2INT(2.0)*MM2INT(2.0); // very sharp corners not allowed :S
    static const int64_t offset_dist_to_get_from_on_the_polygon_to_outside = 20;
    static const int64_t max_comb_distance_ignored = MM2INT(1.5);
    static const int64_t offset_extra_start_end = 100;

    /*!
     * Get the outside boundary, which is an offset from Comb::boundary. Calculate it when it hasn't been calculated yet.
     */
    Polygons* getBoundaryOutside();
    
    /*!
     * Get the outside boundary with the extra offset, which is an offset from Comb::boundary_outside. Calculate it when it hasn't been calculated yet.
     * 
     * The extra offset is used to ensure there is no overlap between the ouside polygons and the comb path, which is neccesary when optimizating the path.
     */
    Polygons* getBoundaryOutsideExtraOffset();
    
    /*!
     * Calculates the outlines for every mesh in the layer (not support)
     * \param storage Where the layer polygon data is stored
     * \param layer_nr The number of the layer for which to generate the combing areas.
     */
    static Polygons getLayerOutlines(SliceDataStorage& storage, unsigned int layer_nr);
    
    static Polygons getLayerOuterWalls(SliceDataStorage& storage, unsigned int layer_nr);
public:
    /*!
     * Initializes the combing areas for every mesh in the layer (not support)
     * \param storage Where the layer polygon data is stored
     * \param layer_nr The number of the layer for which to generate the combing areas.
     */
    Comb(SliceDataStorage& storage, unsigned int layer_nr);
    
    ~Comb();
    
    //! Utility function for `boundary.inside(p)`.
    bool inside(const Point p) { return boundary.inside(p); }
    
    /*!
     * Moves the point \p from inside the comb boundary or leaves the point as-is, when the comb boundary is not within 3 mm distance.
     * 
     * \param from The point to move.
     * \param distance The distance by which to offset the point from the boundary.
     * \return Whether we succeeded in moving inside the comb boundary
     */
    unsigned int moveInside_(Point& from, int distance = 100);

    /*!
     * Calculate the comb paths (if any) - one for each polygon combed alternated with travel paths
     * 
     * \param startPoint Where to start moving from
     * \param endPoint Where to move to
     * \param combPoints Output parameter: The points along the combing path, excluding the \p startPoint (?) and \p endPoint
     * \return Whether combing has succeeded; otherwise a retraction is needed.
     */    
    bool calc(Point startPoint, Point endPoint, CombPaths& combPaths);
    
};

}//namespace cura

#endif//COMB_H

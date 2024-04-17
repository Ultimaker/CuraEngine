// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_SHAPE_H
#define GEOMETRY_SHAPE_H

#include "geometry/lines_set.h"
#include "settings/types/Angle.h"

namespace cura
{

class Polygon;
class Ratio;
class SingleShape;
class PartsView;
class PointMatrix;
class Point3Matrix;

/*!
 *  @brief A Shape is a set of polygons that together form a complex shape. Some of the polygons may
 *         be contained inside others, being actually "holes" of the shape. For example, if you
 *         wanted to represent the "8" digit with polygons, you would need 1 for the outline and 2
 *         for the "holes" so the shape would contain a total of 3 polygons.
 *  @sa https://github.com/Ultimaker/CuraEngine/wiki/Geometric-Base-Types#shape
 */
class Shape : public LinesSet<Polygon>
{
public:
    // Clipper expects and returns implicitely closed polygons
    static constexpr bool clipper_explicitely_closed_ = false;

public:
    /*! \brief Constructor of an empty shape */
    Shape() = default;

    /*!
     * \brief Creates a copy of the given shape
     * \warning A copy of the points list is made, so this constructor can be very slow
     */
    Shape(const Shape& other) = default;

    /*!
     * \brief Constructor that takes the inner polygons list from the given shape
     * \warning This constructor is fast because it does not allocate data, but it will clear
     *          the source object
     */
    Shape(Shape&& other) = default;

    /*!
     * \brief Constructor with an existing set of polygons
     * \warning A copy of the polygons set is made, so this constructor can be very slow
     */
    Shape(const std::vector<Polygon>& polygons);

    /*!
     * \brief Constructor that takes ownership of the given list of points
     * \param explicitely_closed Specify whether the given points form an explicitely closed line
     * \warning This constructor is fast because it does not allocate data, but it will clear
     *          the source object
     */
    explicit Shape(ClipperLib::Paths&& paths, bool explicitely_closed = clipper_explicitely_closed_);

    Shape& operator=(const Shape& other);

    Shape& operator=(Shape&& other);

    void emplace_back(ClipperLib::Paths&& paths, bool explicitely_closed = clipper_explicitely_closed_);

    void emplace_back(ClipperLib::Path&& path, bool explicitely_closed = clipper_explicitely_closed_);

    template<typename... Args>
    void emplace_back(Args&&... args)
    {
        LinesSet::emplace_back(args...);
    }


    Shape difference(const Shape& other) const;

    Shape unionPolygons(const Shape& other, ClipperLib::PolyFillType fill_type = ClipperLib::pftNonZero) const;

    /*!
     * Union all polygons with each other (When polygons.add(polygon) has been called for overlapping polygons)
     */
    Shape unionPolygons() const;

    Shape intersection(const Shape& other) const;

    /*!
     *  @brief Overridden definition of LinesSet<Polygon>::offset()
     *  @note The behavior of this method is exactly the same, but it just exists because it allows
     *        for a performance optimization
     */
    Shape offset(coord_t distance, ClipperLib::JoinType join_type = ClipperLib::jtMiter, double miter_limit = 1.2) const;

    /*!
     * Intersect polylines with the area covered by the shape.
     *
     * \note Due to a clipper bug with polylines with nearly collinear segments, the polylines are cut up into separate polylines, and restitched back together at the end.
     *
     * \param polylines The polylines to limit to the area of this Polygons object
     * \param restitch Whether to stitch the resulting segments into longer polylines, or leave every segment as a single segment
     * \param max_stitch_distance The maximum distance for two polylines to be stitched together with a segment
     * \return The resulting polylines limited to the area of this Polygons object
     * \todo This should technically return a MixedLinesSet, because it can definitely contain open and closed polylines, but that is a heavy change
     */
    template<class LineType>
    LinesSet<OpenPolyline> intersection(const LinesSet<LineType>& polylines, bool restitch = true, const coord_t max_stitch_distance = 10_mu) const;

    Shape xorPolygons(const Shape& other, ClipperLib::PolyFillType pft = ClipperLib::pftEvenOdd) const;

    Shape execute(ClipperLib::PolyFillType pft = ClipperLib::pftEvenOdd) const;

    /*!
     * Check if we are inside the polygon.
     *
     * We do this by counting the number of polygons inside which this point lies.
     * An odd number is inside, while an even number is outside.
     *
     * Returns false if outside, true if inside; if the point lies exactly on the border, will return \p border_result.
     *
     * \param p The point for which to check if it is inside this polygon
     * \param border_result What to return when the point is exactly on the border
     * \return Whether the point \p p is inside this polygon (or \p border_result when it is on the border)
     */
    bool inside(const Point2LL& p, bool border_result = false) const;

    /*!
     * Find the polygon inside which point \p p resides.
     *
     * We do this by tracing from the point towards the positive X direction,
     * every line we cross increments the crossings counter. If we have an even number of crossings then we are not inside the polygon.
     * We then find the polygon with an uneven number of crossings which is closest to \p p.
     *
     * If \p border_result, we return the first polygon which is exactly on \p p.
     *
     * \param p The point for which to check in which polygon it is.
     * \param border_result Whether a point exactly on a polygon counts as inside
     * \return The index of the polygon inside which the point \p p resides
     */
    size_t findInside(const Point2LL& p, bool border_result = false) const;

    /*!
     * \brief Approximates the convex hull of the polygons.
     * \p extra_outset Extra offset outward
     * \return the convex hull (approximately)
     *
     */
    Shape approxConvexHull(int extra_outset = 0) const;

    /*! \brief Make each of the polygons convex */
    void makeConvex();

    /*!
     * Compute the area enclosed within the polygons (minus holes)
     *
     * \return The area in square micron
     */
    double area() const;

    /*!
     * Smooth out small perpendicular segments
     * Smoothing is performed by removing the inner most vertex of a line segment smaller than \p remove_length
     * which has an angle with the next and previous line segment smaller than roughly 150*
     *
     * Note that in its current implementation this function doesn't remove line segments with an angle smaller than 30*
     * Such would be the case for an N shape.
     *
     * \param remove_length The length of the largest segment removed
     * \return The smoothed polygon
     */
    Shape smooth(int remove_length) const;

    /*!
     * Smooth out sharp inner corners, by taking a shortcut which bypasses the corner
     *
     * \param angle The maximum angle of inner corners to be smoothed out
     * \param shortcut_length The desired length of the shortcut line segment introduced (shorter shortcuts may be unavoidable)
     * \return The resulting polygons
     */
    Shape smooth_outward(const AngleDegrees angle, int shortcut_length) const;

    Shape smooth2(int remove_length, int min_area) const; //!< removes points connected to small lines

    void removeColinearEdges(const AngleRadians max_deviation_angle = AngleRadians(0.0005));

    /*!
     * Remove all but the polygons on the very outside.
     * Exclude holes and parts within holes.
     * \return the resulting polygons.
     */
    Shape getOutsidePolygons() const;

    /*!
     * Split up the polygons into groups according to the even-odd rule.
     * Each SingleShape in the result has an outline as first polygon, whereas the rest are holes.
     */
    std::vector<SingleShape> splitIntoParts(bool unionAll = false) const;

    /*!
     * Sort the polygons into bins where each bin has polygons which are contained within one of the polygons in the previous bin.
     *
     * \warning When polygons are crossing each other the result is undefined.
     */
    std::vector<Shape> sortByNesting() const;

    /*!
     * Split up the polygons into groups according to the even-odd rule.
     * Each vector in the result has the index to an outline as first index, whereas the rest are indices to holes.
     *
     * \warning Note that this function reorders the polygons!
     */
    PartsView splitIntoPartsView(bool unionAll = false);

    /*!
     * Removes polygons with area smaller than \p min_area_size (note that min_area_size is in mm^2, not in micron^2).
     * Unless \p remove_holes is true, holes are not removed even if their area is below \p min_area_size.
     * However, holes that are contained within outlines whose area is below the threshold are removed though.
     */
    void removeSmallAreas(const double min_area_size, const bool remove_holes = false);

    /*!
     * Removes the same polygons from this set (and also empty polygons).
     * Shape are considered the same if all points lie within [same_distance] of their counterparts.
     */
    Shape removePolygon(const Shape& to_be_removed, int same_distance = 0) const;

    Shape processEvenOdd(ClipperLib::PolyFillType poly_fill_type = ClipperLib::PolyFillType::pftEvenOdd) const;

    /*!
     * Ensure the polygon is manifold, by removing small areas where the polygon touches itself.
     *  ____                  ____
     * |    |                |    |
     * |    |____     ==>    |   / ____
     *  """"|    |            """ /    |
     *      |____|                |____|
     *
     */
    void ensureManifold();

    void applyMatrix(const PointMatrix& matrix);

    void applyMatrix(const Point3Matrix& matrix);

    Shape offsetMulti(const std::vector<coord_t>& offset_dists) const;

    /*!
     * @brief Remove self-intersections from the polygons
     * _note_: this function uses wagyu to remove the self intersections.
     * since wagyu uses a different internal representation of the polygons
     * we need to convert back and forward between data structures which
     * might impact performance, use wisely!
     *
     * @return Polygons - the cleaned polygons
     */
    Shape removeNearSelfIntersections() const;

    /*!
     * \brief Simplify the polygon lines using ClipperLib::SimplifyPolygons
     */
    void simplify(ClipperLib::PolyFillType fill_type = ClipperLib::pftEvenOdd);

#ifdef BUILD_TESTS
    /*!
     * @brief Import the polygon from a WKT string
     * @param wkt The WKT string to read from
     * @return Polygons The polygons read from the stream
     */
    [[maybe_unused]] static Shape fromWkt(const std::string& wkt);

    /*!
     * @brief Export the polygon to a WKT string
     * @param stream The stream to write to
     */
    [[maybe_unused]] void writeWkt(std::ostream& stream) const;
#endif

private:
    /*!
     * recursive part of \ref Polygons::removeEmptyHoles and \ref Polygons::getEmptyHoles
     * \param node The node of the polygons part to process
     * \param remove_holes Whether to remove empty holes or everything but the empty holes
     * \param ret Where to store polygons which are not empty holes
     */
    void removeEmptyHoles_processPolyTreeNode(const ClipperLib::PolyNode& node, const bool remove_holes, Shape& ret) const;
    void splitIntoParts_processPolyTreeNode(ClipperLib::PolyNode* node, std::vector<SingleShape>& ret) const;
    void sortByNesting_processPolyTreeNode(ClipperLib::PolyNode* node, const size_t nesting_idx, std::vector<Shape>& ret) const;
    void splitIntoPartsView_processPolyTreeNode(PartsView& partsView, Shape& reordered, ClipperLib::PolyNode* node) const;
};

} // namespace cura

#endif // GEOMETRY_SHAPE_H

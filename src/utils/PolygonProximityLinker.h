/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef UTILS_POLYGON_PROXIMITY_LINKER_H
#define UTILS_POLYGON_PROXIMITY_LINKER_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include <utility> // pair

#include <functional> // hash function object

#include "intpoint.h"
#include "polygon.h"

#include "ListPolyIt.h"
#include "ProximityPointLink.h"

namespace cura 
{

/*!
 * Class for computing which parts of polygons are close to which other parts of polygons
 * A link always occurs between a point already on a polygon and either another point of a polygon or a point on a line segment of a polygon.
 * 
 * In the latter case we insert the point into the polygon so that we can later look up by how much to reduce the extrusion at the corresponding line segment.
 * This is the reason that the polygons are converted to (linked) lists before the proximity linking computation takes place, after which they are converted back.
 * 
 * At the end of a sequence of proximity links the polygon segments diverge away from each other.
 * Therefore points are introduced on the line segments involved and a link is created with a link distance of exactly the PolygonProximityLinker::proximity_distance.
 * 
 * We end up with links which include a boolean field to represent whether the link is already processed from outside.
 * This is used by functions which use the PolygonProximityLinker class when there is being looped over points in a polygon, which by definition loops over all links twice.
 * 
 * Each point on the polygons maps to a link, so that we can easily look up which links corresponds to the current line segment being handled when compensating for wall overlaps for example.
 * 
 * The main functionality of this class is performed by the constructor.
 */
class PolygonProximityLinker
{
public:
    typedef std::unordered_set<ProximityPointLink> ProximityPointLinks; //!< The type of PolygonProximityLinker::overlap_point_links
    typedef std::unordered_multimap<Point, ProximityPointLink> Point2Link; //!< The type of PolygonProximityLinker::point_to_link

private:
    /////////////////////////////////////////////////////////////////////////////////////////////

    Polygons& polygons; //!< The polygons for which to compensate overlapping walls for
    ListPolygons list_polygons; //!< The PolygonProximityLinker::polygons converted

    int proximity_distance; //!< The line width of the walls
    int proximity_distance_2; //!< The squared line width of the walls

    ProximityPointLinks proximity_point_links; //!< mapping from each link to its attributes

    Point2Link point_to_link; //!< mapping from each point to the/a corresponding link (collisions are ignored as of yet)

    void findProximatePoints(); //!< find the basic proximity links (for trapezoids) and record them into PolygonProximityLinker::overlap_point_links
    /*!
     * find the basic proximity links (for trapezoids) between a given point and a polygon and record them into PolygonProximityLinker::overlap_point_links
     * 
     * \param from The point from which to check for proximity
     * \param to_list_poly_idx The index into PolygonProximityLinker::list_polygons for the polygon to check
     */
    void findProximatePoints(ListPolyIt from, unsigned int to_list_poly_idx);

    /*!
     * Find the basic proximity link (for a trapezoid) between a given point and a line segment
     * and record them into PolygonProximityLinker::overlap_point_links
     * 
     * \param a_from_it Iterator to the point from which to check for proximity
     * \param to_list_poly The polygon in which the line segment occurs
     * \param b_from_it iterator to the one end point of the line segment
     * \param b_to_it iterator to the other end point of the line segment
     */
    void findProximatePoints(const ListPolyIt a_from_it, ListPolygon& to_list_poly, const ListPolyIt b_from_it, const ListPolyIt b_to_it);

    /*!
     * Add a link between \p from and \p to to PolygonProximityLinker::overlap_point_links and add the appropriate mappings to PolygonProximityLinker::point_to_link
     * 
     * \param from The one point of the link
     * \param to The other point of the link
     * \param dist The distance between the two points
     * \param type The type of the link being introduced
     * \return Whether the point has been added
     */
    bool addProximityLink(ListPolyIt from, ListPolyIt to, int64_t dist, const ProximityPointLinkType type);


    /*!
     * Add a link for the corner at \p corner_point to PolygonProximityLinker::overlap_point_links and add the appropriate mappings to PolygonProximityLinker::point_to_link
     * 
     * This is done by adding a link between the point and itself.
     * 
     * \param corner_point The one point of the link
     * \param type The type of the link being introduced
     * \return Whether the point has been added
     */
    bool addCornerLink(ListPolyIt corner_point, const ProximityPointLinkType type);

    /*!
     * Add links for the ending points of proximity regions, supporting the residual triangles.
     */
    void addProximityEndings();

    /*!
     * Add a link for the ending point of a given proximity region, if it is an ending.
     * 
     * \param link_pair The link which might be an ending
     * \param a_next The next point from ListPolyIt::a of \p link 
     * \param b_next The next point from ListPolyIt::b of \p link (in the opposite direction of \p a_next)
     * \param a_before_middle Where to insert a new point for a if this is indeed en ending
     * \param b_before_middle Where to insert a new point for b if this is indeed en ending
     */
    void addProximityEnding(const ProximityPointLink& link, const ListPolyIt& a_next, const ListPolyIt& b_next, const ListPolyIt& a_before_middle, const ListPolyIt& b_before_middle);

    /*!
     * Compute the distance between the points of the last link and the points introduced to account for the proximity endings.
     */
    int64_t proximityEndingDistance(Point& a1, Point& a2, Point& b1, Point& b2, int a1b1_dist);

    /*!
     * Add proximity links for sharp corners, so that the proximity of two consecutive line segments is compensated for.
     * 
     * Currently UNIMPLEMENTED.
     */
    void addSharpCorners();

    /*!
     * Map a point to a link in PolygonProximityLinker::point_to_link
     * 
     * \param p The key
     * \param it The value
     */
    void addToPoint2LinkMap(Point p, ProximityPointLinks::iterator it);

public:
    void proximity2HTML(const char* filename) const; //!< debug

    /*!
     * Computes the neccesary priliminaries in order to efficiently compute the flow when generatign gcode paths.
     * \param polygons The wall polygons for which to compute the overlaps
     */
    PolygonProximityLinker(Polygons& polygons, int proximity_distance);

    /*!
     * Check whether a point has any links
     * \param from the point for which to check whether it has any links
     * \return Whether a link has been created between the point and another point
     */
    bool isLinked(Point from);

    /*!
     * Get all links connected to a given point.
     * 
     * The returned pair is an iterator range;
     * The first is the starting iterator (inclusive)
     * and the second is the end iterator (exclusive).
     * 
     * Note that the returned iterators point to a pair,
     * for which the second is the actual link.
     * The first is \p from
     * 
     * \param from The point to get all connected links for
     * \return a pair containing two iterators
     */
    std::pair<PolygonProximityLinker::Point2Link::iterator, PolygonProximityLinker::Point2Link::iterator> getLinks(Point from);

    /*!
     * Check whether two points are linked
     * \param a an iterator to the first point (in \ref PolygonProximityLinker::list_polygons)
     * \param b an iterator to the second point (in \ref PolygonProximityLinker::list_polygons)
     * \return Whether a link has been created between the two points
     */
    bool isLinked(ListPolyIt a, ListPolyIt b);

    /*!
     * Get the link between two points if they are linked already
     * \param a an iterator to the first point (in \ref PolygonProximityLinker::list_polygons)
     * \param b an iterator to the second point (in \ref PolygonProximityLinker::list_polygons)
     * \return The link between the two points, or nullptr
     */
    const ProximityPointLink* getLink(ListPolyIt a, ListPolyIt b);
};


}//namespace cura


#endif//UTILS_POLYGON_PROXIMITY_LINKER_H

#ifndef UTILS_POLYGON_PROXIMITY_LINKER_H
#define UTILS_POLYGON_PROXIMITY_LINKER_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <list>

#include <functional> // hash function object

#include "intpoint.h"
#include "polygon.h"
#include "linearAlg2D.h"

#include "ListPolyIt.h"

namespace cura 
{

/*!
 * TODO: update description
 * Class for computing and compensating for overlapping (outer) wall lines.
 * The overlapping area is approximated with connected trapzoids.
 * All places where the wall is closer than the nozzle width to another piece of wall are recorded.
 * The area of a trapezoid is then the length between two such locations multiplied by the average overlap at the two locations.
 * 
 * The amount of overlap between two locations is recorded in a link, so that we can look up the overlap at a given point in the polygon.
 * A link always occurs between a point already on a polygon and either another point of a polygon or a point on a line segment of a polygon.
 * In the latter case we insert the point into the polygon so that we can later look up by how much to reduce the extrusion at the corresponding line segment.
 * This is the reason that the polygons are converted to linked lists before the wall overlap compensation computation takes place, after which they are converted back.
 * 
 * At the end of a sequence of trapezoids the overlap area generally ends with a residual triangle.
 * Therefore points are introduced on the line segments involved and a link is created with overlap zero.
 * 
 * We end up with a mapping from each link to a boolean value representing whether the trapezoid is already compensated for.
 * Each point on the polygons then maps to a link (and its corresponding boolean), so that we can easily look up which links corresponds 
 * to the current line segment being produced when producing gcode.
 * 
 * When producing gcode, the first line crossing the overlap area is laid down normally and the second line is reduced by the overlap amount.
 * For this reason the function PolygonProximityLinker::getFlow changes the internal state of this PolygonProximityLinker.
 * 
 * The main functionality of this class is performed by the constructor.
 * The adjustment during gcode generation is made with the help of PolygonProximityLinker::getFlow
 */
class PolygonProximityLinker
{
    /*!
     * A class recording the amount of overlap implicitly by recording the distance between two points on two different polygons or one and the same polygon.
     * The order of the two points doesn't matter.
     */
    struct ProximityPointLink
    {
        const ListPolyIt a; //!< the one point (invalidated after list_polygons have been cleared!)
        const ListPolyIt b; //!< the other point (invalidated after list_polygons have been cleared!)
        ProximityPointLink(const ListPolyIt a, const ListPolyIt b) : a(a), b(b) { }
        bool operator==(const ProximityPointLink& other) const { return (a == other.a && b == other.b) || (a == other.b && b == other.a); }
    };
    
    /*!
     * The hash function object for WallOverlapPointLink
     */
    struct ProximityPointLink_Hasher
    {
        std::size_t operator()(const ProximityPointLink& pp) const
        {
            return std::hash<Point>()(*pp.a.it) + std::hash<Point>()(*pp.b.it);
        }
    };

    struct ProximityPointLinkAttributes
    {
        int dist; //!< The distance between the two points
        bool passed; //!< Whether this point has been passed while writing gcode
        ProximityPointLinkAttributes(int dist, bool passed) : dist(dist), passed(passed) { }
    };
    
    typedef std::unordered_map<ProximityPointLink, ProximityPointLinkAttributes, ProximityPointLink_Hasher> ProximityPointLinks; //!< The type of PolygonProximityLinker::overlap_point_links
    typedef std::unordered_map<Point, ProximityPointLinks::iterator> Point2Link; //!< The type of PolygonProximityLinker::point_to_link \warning mapping to iterators which might get invalidated!
    
    
    /////////////////////////////////////////////////////////////////////////////////////////////
    
    
    Polygons& polygons; //!< The polygons for which to compensate overlapping walls for
    ListPolygons list_polygons; //!< The PolygonProximityLinker::polygons converted
    
    int proximity_distance; //!< The line width of the walls
    
    ProximityPointLinks proximity_point_links; //!< mapping from each link to its attributes
    ProximityPointLinks proximity_point_links_endings; //!< mapping from each ending link to its attributes (which has a distance field equal to PolygonProximityLinker::line_width). Note that this is a separate map from PolygonProximityLinker::overlap_point_links, because that magically solved a bug .
    
    Point2Link point_to_link; //!< mapping from each point to the/a corresponding link (collisions are ignored as of yet) \warning mapping to iterators which might get invalidated!

    void findProximatePoints(); //!< find the basic overlap links (for trapezoids) and record them into PolygonProximityLinker::overlap_point_links
    /*!
     * find the basic overlap links (for trapezoids) between a given point and a polygon and record them into PolygonProximityLinker::overlap_point_links
     * 
     * \param from The point from which to check for overlap
     * \param to_list_poly_idx The index into PolygonProximityLinker::list_polygons for the polygon to check
     */
    void findProximatePoints(ListPolyIt from, unsigned int to_list_poly_idx);
    /*!
     * Find the basic overlap links (for trapezoids) between a given point and a polygon up from a particular index and record them into PolygonProximityLinker::overlap_point_links
     * 
     * This function is used for finding overlaps within a single polygon. It then uses a \p start different from the first point in the polygon.
     * 
     * \param from The point from which to check for overlap
     * \param to_list_poly_idx The index into PolygonProximityLinker::list_polygons for the polygon to check
     * \param start Where to start looking into the polygon with index \p to_list_poly_idx
     */
    void findProximatePoints(ListPolyIt from, unsigned int to_list_poly_idx, const ListPolygon::iterator start);
    
    /*!
     * Add a link between \p from and \p to to PolygonProximityLinker::overlap_point_links and add the appropriate mappings to PolygonProximityLinker::point_to_link
     * 
     * \param from The one point of the link
     * \param to The other point of the link
     * \param dist The distance between the two points
     * \return Whether the point has been added
     */
    bool addProximityLink(ListPolyIt from, ListPolyIt to, int64_t dist);
    /*!
     * Add a link between \p from and \p to to PolygonProximityLinker::overlap_point_links_endings and add the appropriate mappings to PolygonProximityLinker::point_to_link
     * 
     * \param from The one point of the link
     * \param to The other point of the link
     * \param dist The distance between the two points
     * \return Whether the point has been added
     */
    bool addProximityLink_endings(ListPolyIt from, ListPolyIt to, int64_t dist);
    
    /*!
     * Add links for the ending points of overlap regions, supporting the residual triangles.
     */
    void addProximityEndings();
    
    /*!
     * Add a link for the ending point of a given overlap region, if it is an ending.
     * 
     * \param link_pair The link which might be an ending
     * \param a_next The next point from ListPolyIt::a of \p link 
     * \param b_next The next point from ListPolyIt::b of \p link (in the opposite direction of \p a_next)
     * \param a_before_middle Where to insert a new point for a if this is indeed en ending
     * \param b_before_middle Where to insert a new point for b if this is indeed en ending
     */
    void addProximityEnding(std::pair<ProximityPointLink, ProximityPointLinkAttributes> link_pair, const ListPolyIt& a_next, const ListPolyIt& b_next, const ListPolyIt& a_before_middle, const ListPolyIt& b_before_middle);
    
    /*!
     * Compute the distance between the points of the last link and the points introduced to account for the overlap endings.
     */
    int64_t proximityEndingDistance(Point& a1, Point& a2, Point& b1, Point& b2, int a1b1_dist);
    
    
    /*!
     * Add overlap links for sharp corners, so that the overlap of two consecutive line segments is compensated for.
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

    void debugCheckNonePassedYet(); //!< check whether no link has passed set to true
    
    void proximity2HTML(const char* filename) const; //!< debug
    
    /*!
     * Computes the neccesary priliminaries in order to efficiently compute the flow when generatign gcode paths.
     * \param polygons The wall polygons for which to compute the overlaps
     */
    PolygonProximityLinker(Polygons& polygons, int proximity_distance);
    
};


}//namespace cura



#endif//UTILS_POLYGON_PROXIMITY_LINKER_H

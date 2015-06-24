#ifndef WALL_OVERLAP_H
#define WALL_OVERLAP_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <list>

#include <hash_fun.h> // hash function object

#include "utils/intpoint.h"
#include "utils/polygon.h"
#include "utils/polygonUtils.h"

#include "debug.h" // TODO remove

namespace cura 
{
    
    


// struct WallOverlapPointLink
// {
//     Point a;
//     Point b;
//     int dist;
//     WallOverlapPointLink(Point a, Point b, int dist) : a(a), b(b), dist(dist) { }
//     bool operator==(const WallOverlapPointLink& other) const { return (a == other.a && b == other.b) || (a == other.b && b == other.a); }
// };
} // namespace cura 


namespace std {
//     template <>
//     struct hash<cura::WallOverlapPointLink> 
//     {
//         size_t operator()(const cura::WallOverlapPointLink & pp) const
//         {
//             return std::hash<Point>()(*pp.a.it) + std::hash<Point>()(*pp.b.it);
//         }
//     };
    
//     template <>
//     struct hash<cura::WallOverlapPointLink> 
//     {
//         size_t operator()(const cura::WallOverlapPointLink & pp) const
//         {
//             static int prime = 31;
//             int result = 1;
//             result = result * prime + std::hash<Point>()(pp.a);
//             result = result * prime + std::hash<Point>()(pp.b);
//             return result; 
            // a and b should be at the same level
//             return std::hash<Point>()(pp.a) + std::hash<Point>()(pp.b);
//         }
//     };
}   // namespace std 


namespace cura 
{
    
// struct PolyPointRef
// {
//     unsigned int poly_idx; 
//     unsigned int point_idx;
//     
//     PolyPointRef(unsigned int poly_idx, unsigned int point_idx) : poly_idx(poly_idx), point_idx(point_idx) { }
// };



class WallOverlapComputation
{
//     struct NewPolyPoint
//     {
//         Point p;
//         PolyPointRef poly_point_before;
//         NewPolyPoint(Point& p, unsigned int poly_idx, unsigned int point_idx) : p(p), poly_point_before(poly_idx, point_idx) { }
//     };
    
    typedef std::list<Point> ListPolygon;
    typedef std::vector<ListPolygon> ListPolygons;
        
    struct ListPolyIt
    {
        ListPolygon& poly;
        ListPolygon::iterator it; 
        ListPolyIt(ListPolygon& poly, ListPolygon::iterator it)
        : poly(poly), it(it) { }
        Point& p() const { return *it; }
        bool operator==(const ListPolyIt& other) const { return it == other.it; }
        ListPolyIt& operator++() 
        { 
            it++; 
            if (it == poly.end()) { it = poly.begin(); }
            return *this; 
        }
        ListPolyIt& operator--() 
        { 
            if (it == poly.begin()) { it = poly.end(); }
            it--; 
            return *this; 
        }
    };

    struct WallOverlapPointLink
    {
        const ListPolyIt a; // invalidated after list_polygons have been cleared!
        const ListPolyIt b;
        int dist;
        WallOverlapPointLink(const ListPolyIt a, const ListPolyIt b, int dist)
        : a(a), b(b), dist(dist) 
        { 
            Point ap = a.p();
            Point bp = b.p();
            if (std::abs(vSize(ap - bp) - dist) > 10)
            {
//                 DEBUG_PRINTLN(vSize(ap - bp) << "!="<< dist);
            }
        }
        bool operator==(const WallOverlapPointLink& other) const { return (a == other.a && b == other.b) || (a == other.b && b == other.a); }
    };
    
    struct WallOverlapPointLink_Hasher
    {
        std::size_t operator()(const WallOverlapPointLink& pp) const
        {
            return std::hash<Point>()(*pp.a.it) + std::hash<Point>()(*pp.b.it);
        }
    };

    typedef std::unordered_map<WallOverlapPointLink, bool, WallOverlapPointLink_Hasher> WallOverlapPointLinks;
    
    
    Polygons& polygons;
    int lineWidth;
    
    WallOverlapPointLinks overlap_point_links;
    
    typedef std::unordered_map<Point, WallOverlapPointLinks::iterator> Point2Link;
    Point2Link point_to_link;
    
        
    
    ListPolygons list_polygons;

//     typedef std::unordered_map<Point, ListPolygon::iterator> Loc2ListPolyIndex;
//     
//     Loc2ListPolyIndex loc_to_list_poly_idx;
//     
//     std::unordered_set<WallOverlapPointLink> end_points;
    
    void findOverlapPoints();
//     void findOverlapPoints(ListPolyIt from, PolygonRef to_poly);
    void findOverlapPoints(ListPolyIt from, unsigned int to_list_poly_idx);
    void findOverlapPoints(ListPolyIt from, unsigned int to_list_poly_idx, ListPolygon::iterator start);
    
    void convertPolygonsToLists(Polygons& polys, ListPolygons& result);
    void convertPolygonToList(PolygonRef poly, ListPolygon& result);
    void convertListPolygonToPolygon(ListPolygon& poly, PolygonRef result);
    
    void addOverlapPoint(ListPolyIt from, ListPolyIt to, int64_t dist);
    
    void addOverlapEndings();
    
    int64_t overlapEndingDistance(Point& a1, Point& a2, Point& b1, Point& b2, int a1b1_dist);
        
    void createPoint2LinkMap();
    void addToPoint2LinkMap(Point p, WallOverlapPointLinks::iterator it);
    
public:
    float getFlow(Point& from, Point& to);
    
    void debugOutputCSV();
    
    void debugCheck();
    
    WallOverlapComputation(Polygons& polygons) : polygons(polygons), lineWidth(400) // TODO
    { 
        findOverlapPoints();
//         createPoint2LinkMap();
        addOverlapEndings();
//         loc_to_list_poly_idx.clear();
        // TODO: add corners
        
//         debugCheck();
//         debugOutputCSV();
    }
    
};


}//namespace cura



#endif//WALL_OVERLAP_H

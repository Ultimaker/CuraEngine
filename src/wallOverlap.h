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


namespace cura 
{
struct WallOverlapPointLink
{
    Point a;
    Point b;
    int dist;
    bool passed;
    WallOverlapPointLink(Point a, Point b, int dist) : a(a), b(b), dist(dist), passed(false) { }
    bool operator==(const WallOverlapPointLink& other) const { return a == other.a && b == other.b; }
};
} // namespace cura 


namespace std {
    template <>
    struct hash<cura::WallOverlapPointLink> 
    {
        size_t operator()(const cura::WallOverlapPointLink & pp) const
        {
            static int prime = 31;
            int result = 1;
            result = result * prime + std::hash<Point>()(pp.a);
            result = result * prime + std::hash<Point>()(pp.b);
            return result; 
        }
    };
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
    

//     struct Links: public std::unordered_map<Point, Link>
    struct WallOverlapPointLinks : public std::unordered_set<WallOverlapPointLink>
    {

    };
    
    
    Polygons& polygons;
    int lineWidth;
    int lineR; // half the lineWidth
    
    WallOverlapPointLinks overlap_point_links;
    
    typedef std::unordered_map<Point, std::vector<WallOverlapPointLinks::iterator>> Point2Link;
    Point2Link point_to_link;
    
        
    typedef std::list<Point> ListPolygon;
    typedef std::list<ListPolygon> ListPolygons;

    
    void findOverlapPoints(Point from);
    void findOverlapPoints(Point from, PolygonRef to_poly);
    void findOverlapPoints(Point from, ListPolygon& to_list_poly);
    
    void convertPolygonsToLists(Polygons& polys, ListPolygons& result);
    void convertPolygonToList(PolygonRef poly, ListPolygon& result);
    void convertListPolygonToPolygon(ListPolygon& poly, PolygonRef result);

    void findOverlapPoints();
    
    void createPoint2LinkMap();
    void addToPoint2LinkMap(Point p, WallOverlapPointLinks::iterator it);
    
public:
    float getFlow(Point& from, Point& to);
    
    WallOverlapComputation(Polygons& polygons) : polygons(polygons) 
    { 
        findOverlapPoints();
        // TODO: handle ends of overlaps
        // TODO: add corners
        createPoint2LinkMap();
    }
};


}//namespace cura



#endif//WALL_OVERLAP_H

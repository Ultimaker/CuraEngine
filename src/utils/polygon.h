#ifndef UTILS_POLYGON_H
#define UTILS_POLYGON_H

#include <vector>
#include <assert.h>
#include <float.h>
using std::vector;
#include <clipper/clipper.hpp>

#include "intpoint.h"

//#define CHECK_POLY_ACCESS
#ifdef CHECK_POLY_ACCESS
#define POLY_ASSERT(e) assert(e)
#else
#define POLY_ASSERT(e) do {} while(0)
#endif

namespace cura {

const static int clipper_init = (0);
#define NO_INDEX (std::numeric_limits<unsigned int>::max())

class PolygonRef
{
    ClipperLib::Path* polygon;
    PolygonRef();
public:
    PolygonRef(ClipperLib::Path& polygon);

    ClipperLib::Path::iterator begin();
    ClipperLib::Path::iterator end();
    
    unsigned int size() const;
    Point operator[] (unsigned int index) const;
    void* data();
    void add(const Point p);
    void remove(unsigned int index);
    void clear();
    bool orientation() const;
    void reverse();
    int64_t polygonLength() const;
    double area() const;
    Point centerOfMass() const;
    Point closestPointTo(Point p);

    //Check if we are inside the polygon. We do this by tracing from the point towards the negative X direction,
    //  every line we cross increments the crossings counter. If we have an even number of crossings then we are not inside the polygon.
    bool inside(Point p);
    
    friend class Polygons;
    friend class Polygon;
};

class Polygon : public PolygonRef
{
    ClipperLib::Path poly;
public:
    Polygon();
    Polygon(const PolygonRef& other);
};

class Polygons
{
private:
    ClipperLib::Paths polygons;
public:
    ClipperLib::Paths::iterator begin();
    ClipperLib::Paths::iterator end();

    unsigned int size();
    PolygonRef operator[] (unsigned int index);
    void remove(unsigned int index);
    void clear();
    void add(const PolygonRef& poly);
    void add(const Polygons& other);
    PolygonRef newPoly();

    Polygons();
    Polygons(const Polygons& other);
    Polygons& operator=(const Polygons& other);
    Polygons difference(const Polygons& other) const;
    Polygons unionPolygons(const Polygons& other) const;
    Polygons intersection(const Polygons& other) const;
    Polygons offset(int distance) const;
    vector<Polygons> splitIntoParts(bool unionAll = false) const;
    Polygons processEvenOdd() const;
    int64_t polygonLength() const;
    bool inside(Point p);
    void applyMatrix(const PointMatrix& matrix);
private:
    void _processPolyTreeNode(ClipperLib::PolyNode* node, vector<Polygons>& ret) const;
};

/* Axis aligned boundary box */
class AABB
{
public:
    Point min, max;
    
    AABB();
    AABB(Polygons polys);
    void calculate(Polygons polys);
    bool hit(const AABB& other) const;
};

}//namespace cura

#endif//UTILS_POLYGON_H

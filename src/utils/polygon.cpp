#include "polygon.h"

namespace cura {
// ----------------- //
// POLYGON REF BELOW //
// ----------------- //

PolygonRef::PolygonRef() : polygon(nullptr) {}

PolygonRef::PolygonRef(ClipperLib::Path& polygon) : polygon(&polygon) {}

ClipperLib::Path::iterator PolygonRef::begin() { return polygon->begin(); }

ClipperLib::Path::iterator PolygonRef::end() { return polygon->end(); }
unsigned int PolygonRef::size() const { return polygon->size(); }

Point PolygonRef::operator[](unsigned int index) const
{
    POLY_ASSERT(index < size());
    return (*polygon)[index];
}

void* PolygonRef::data() { return polygon->data(); }

void PolygonRef::add(const Point p) { polygon->push_back(p); }

void PolygonRef::remove(unsigned int index)
{
    POLY_ASSERT(index < size());
    polygon->erase(polygon->begin() + index);
}

void PolygonRef::clear() { polygon->clear(); }

bool PolygonRef::orientation() const
{ return ClipperLib::Orientation(*polygon); }

void PolygonRef::reverse() { ClipperLib::ReversePath(*polygon); }

int64_t PolygonRef::polygonLength() const
{
    int64_t length = 0;
    Point p0 = (*polygon)[polygon->size() - 1];
    for (unsigned int n = 0; n < polygon->size(); n++)
    {
        Point p1 = (*polygon)[n];
        length += vSize(p0 - p1);
        p0 = p1;
    }
    return length;
}

double PolygonRef::area() const { return ClipperLib::Area(*polygon); }

Point PolygonRef::centerOfMass() const
{
    double x = 0, y = 0;
    Point p0 = (*polygon)[polygon->size() - 1];
    for (unsigned int n = 0; n < polygon->size(); n++)
    {
        Point p1 = (*polygon)[n];
        double second_factor = (p0.X * p1.Y) - (p1.X * p0.Y);

        x += double(p0.X + p1.X) * second_factor;
        y += double(p0.Y + p1.Y) * second_factor;
        p0 = p1;
    }

    double area = Area(*polygon);
    x = x / 6 / area;
    y = y / 6 / area;

    if (x < 0)
    {
        x = -x;
        y = -y;
    }
    return Point(x, y);
}

Point PolygonRef::closestPointTo(Point p)
{
    Point ret = p;
    float bestDist = FLT_MAX;
    for (unsigned int n = 0; n < polygon->size(); n++)
    {
        float dist = vSize2f(p - (*polygon)[n]);
        if (dist < bestDist)
        {
            ret = (*polygon)[n];
            bestDist = dist;
        }
    }
    return ret;
}

// Check if we are inside the polygon. We do this by tracing from the point
// towards the negative X direction,
//  every line we cross increments the crossings counter. If we have an even
// number of crossings then we are not inside the polygon.
bool PolygonRef::inside(Point p)
{
    if (polygon->size() < 1)
        return false;

    int crossings = 0;
    Point p0 = (*polygon)[polygon->size() - 1];
    for (unsigned int n = 0; n < polygon->size(); n++)
    {
        Point p1 = (*polygon)[n];

        if ((p0.Y >= p.Y && p1.Y < p.Y) || (p1.Y > p.Y && p0.Y <= p.Y))
        {
            int64_t x = p0.X + (p1.X - p0.X) * (p.Y - p0.Y) / (p1.Y - p0.Y);
            if (x >= p.X)
                crossings++;
        }
        p0 = p1;
    }
    return (crossings % 2) == 1;
}

// ----------------- //
// POLYGON REF ABOVE //
// POLYGON BELOW     //
// ----------------- //
Polygon::Polygon() : PolygonRef(poly) {}

Polygon::Polygon(const PolygonRef& other) : PolygonRef(poly)
{ poly = *other.polygon; }

// ----------------- //
// POLYGON ABOVE     //
// POLYGONS BELOW    //
// ----------------- //
ClipperLib::Paths::iterator Polygons::begin() { return polygons.begin(); }

ClipperLib::Paths::iterator Polygons::end() { return polygons.end(); }

unsigned int Polygons::size() { return polygons.size(); }

PolygonRef Polygons::operator[](unsigned int index)
{
    POLY_ASSERT(index < size());
    return PolygonRef(polygons[index]);
}
void Polygons::remove(unsigned int index)
{
    POLY_ASSERT(index < size());
    polygons.erase(polygons.begin() + index);
}
void Polygons::clear() { polygons.clear(); }
void Polygons::add(const PolygonRef& poly)
{ polygons.push_back(*poly.polygon); }
void Polygons::add(const Polygons& other)
{
    for (unsigned int n = 0; n < other.polygons.size(); n++)
        polygons.push_back(other.polygons[n]);
}
PolygonRef Polygons::newPoly()
{
    polygons.push_back(ClipperLib::Path());
    return PolygonRef(polygons[polygons.size() - 1]);
}

Polygons::Polygons() {}
Polygons::Polygons(const Polygons& other) { polygons = other.polygons; }
Polygons& Polygons::operator=(const Polygons& other)
{
    polygons = other.polygons;
    return *this;
}
Polygons Polygons::difference(const Polygons& other) const
{
    Polygons ret;
    ClipperLib::Clipper clipper(clipper_init);
    clipper.AddPaths(polygons, ClipperLib::ptSubject, true);
    clipper.AddPaths(other.polygons, ClipperLib::ptClip, true);
    clipper.Execute(ClipperLib::ctDifference, ret.polygons);
    return ret;
}
Polygons Polygons::unionPolygons(const Polygons& other) const
{
    Polygons ret;
    ClipperLib::Clipper clipper(clipper_init);
    clipper.AddPaths(polygons, ClipperLib::ptSubject, true);
    clipper.AddPaths(other.polygons, ClipperLib::ptSubject, true);
    clipper.Execute(ClipperLib::ctUnion, ret.polygons, ClipperLib::pftNonZero,
                    ClipperLib::pftNonZero);
    return ret;
}
Polygons Polygons::intersection(const Polygons& other) const
{
    Polygons ret;
    ClipperLib::Clipper clipper(clipper_init);
    clipper.AddPaths(polygons, ClipperLib::ptSubject, true);
    clipper.AddPaths(other.polygons, ClipperLib::ptClip, true);
    clipper.Execute(ClipperLib::ctIntersection, ret.polygons);
    return ret;
}
Polygons Polygons::offset(int distance) const
{
    Polygons ret;
    ClipperLib::ClipperOffset clipper;
    clipper.AddPaths(polygons, ClipperLib::jtMiter,
                     ClipperLib::etClosedPolygon);
    clipper.MiterLimit = 2.0;
    clipper.Execute(ret.polygons, distance);
    return ret;
}
vector<Polygons> Polygons::splitIntoParts(bool unionAll) const
{
    vector<Polygons> ret;
    ClipperLib::Clipper clipper(clipper_init);
    ClipperLib::PolyTree resultPolyTree;
    clipper.AddPaths(polygons, ClipperLib::ptSubject, true);
    if (unionAll)
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree,
                        ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    else
        clipper.Execute(ClipperLib::ctUnion, resultPolyTree);

    _processPolyTreeNode(&resultPolyTree, ret);
    return ret;
}

void Polygons::_processPolyTreeNode(ClipperLib::PolyNode* node,
                                    vector<Polygons>& ret) const
{
    for (int n = 0; n < node->ChildCount(); n++)
    {
        ClipperLib::PolyNode* child = node->Childs[n];
        Polygons polygons;
        polygons.add(child->Contour);
        for (int i = 0; i < child->ChildCount(); i++)
        {
            polygons.add(child->Childs[i]->Contour);
            _processPolyTreeNode(child->Childs[i], ret);
        }
        ret.push_back(polygons);
    }
}

Polygons Polygons::processEvenOdd() const
{
    Polygons ret;
    ClipperLib::Clipper clipper(clipper_init);
    clipper.AddPaths(polygons, ClipperLib::ptSubject, true);
    clipper.Execute(ClipperLib::ctUnion, ret.polygons);
    return ret;
}

int64_t Polygons::polygonLength() const
{
    int64_t length = 0;
    for (unsigned int i = 0; i < polygons.size(); i++)
    {
        Point p0 = polygons[i][polygons[i].size() - 1];
        for (unsigned int n = 0; n < polygons[i].size(); n++)
        {
            Point p1 = polygons[i][n];
            length += vSize(p0 - p1);
            p0 = p1;
        }
    }
    return length;
}

bool Polygons::inside(Point p)
{
    if (size() < 1)
        return false;
    if (!(*this)[0].inside(p))
        return false;
    for (unsigned int n = 1; n < polygons.size(); n++)
    {
        if ((*this)[n].inside(p))
            return false;
    }
    return true;
}

void Polygons::applyMatrix(const PointMatrix& matrix)
{
    for (unsigned int i = 0; i < polygons.size(); i++)
    {
        for (unsigned int j = 0; j < polygons[i].size(); j++)
        {
            polygons[i][j] = matrix.apply(polygons[i][j]);
        }
    }
}
// ----------------- //
// POLYGONS ABOVE    //
// AAAB BELOW        //
// ----------------- //
AABB::AABB() : min(POINT_MIN, POINT_MIN), max(POINT_MIN, POINT_MIN) {}
AABB::AABB(Polygons polys)
    : min(POINT_MIN, POINT_MIN), max(POINT_MIN, POINT_MIN)
{ calculate(polys); }

void AABB::calculate(Polygons polys)
{
    min = Point(POINT_MAX, POINT_MAX);
    max = Point(POINT_MIN, POINT_MIN);
    for (unsigned int i = 0; i < polys.size(); i++)
    {
        for (unsigned int j = 0; j < polys[i].size(); j++)
        {
            if (min.X > polys[i][j].X)
                min.X = polys[i][j].X;
            if (min.Y > polys[i][j].Y)
                min.Y = polys[i][j].Y;
            if (max.X < polys[i][j].X)
                max.X = polys[i][j].X;
            if (max.Y < polys[i][j].Y)
                max.Y = polys[i][j].Y;
        }
    }
}

bool AABB::hit(const AABB& other) const
{
    if (max.X < other.min.X)
        return false;
    if (min.X > other.max.X)
        return false;
    if (max.Y < other.min.Y)
        return false;
    if (min.Y > other.max.Y)
        return false;
    return true;
}

}//namespace cura

/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef COMB_H
#define COMB_H

#include "utils/polygon.h"

namespace cura {

class Comb
{
private:
    Polygons& boundary; //!< The comb boundary used

    int64_t* minX;
    int64_t* maxX;
    unsigned int* minIdx;
    unsigned int* maxIdx;

    PointMatrix matrix;
    Point sp;
    Point ep;

    bool preTest(Point startPoint, Point endPoint);    
    bool collisionTest(Point startPoint, Point endPoint);

    void calcMinMax();
    
    unsigned int getPolygonAbove(int64_t x);
    
    Point getBoundaryPointWithOffset(unsigned int polygonNr, unsigned int idx);
    
public:
    Comb(Polygons& _boundary);
    ~Comb();
    
    //! Utility function for `comb_boundary.inside(p)`.
    bool inside(const Point p) { return boundary.inside(p); }
    
    /*!
     * Moves the point \p p inside the comb boundary or leaves the point as-is, when the comb boundary is not within \p distance.
     * 
     * \param p The point to move.
     * \param distance The distance by which to move the point.
     * \return Whether we succeeded in moving inside the comb boundary
     */
    bool moveInside(Point* p, int distance = 100);
    
    /*!
     * Calculate the comb path (if any)
     * 
     * \param startPoint Where to start moving from
     * \param endPoint Where to move to
     * \param combPoints Output parameter: The points along the combing path, excluding the \p startPoint (?) and \p endPoint
     * \return Whether combing has succeeded; otherwise a retraction is needed.
     */
    bool calc(Point startPoint, Point endPoint, std::vector<Point>& combPoints);
};

}//namespace cura

#endif//COMB_H

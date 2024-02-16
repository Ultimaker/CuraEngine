// Copyright (c) 2023 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_POINTS_SET_H
#define GEOMETRY_POINTS_SET_H

#include "geometry/point2ll.h"
#include "utils/Coord_t.h"

namespace cura
{

class PointMatrix;
class Point3Matrix;

const static int clipper_init = (0);
#define NO_INDEX (std::numeric_limits<size_t>::max())

#warning Move this somewhere else, or remove it...
template<typename T>
bool shorterThan(const T& shape, const coord_t check_length)
{
    const auto* p0 = &shape.back();
    int64_t length = 0;
    for (const auto& p1 : shape)
    {
        length += vSize(*p0 - p1);
        if (length >= check_length)
        {
            return false;
        }
        p0 = &p1;
    }
    return true;
}

class PointsSet : public std::vector<Point2LL>
{
public:
    PointsSet() = default;

    PointsSet(const std::initializer_list<Point2LL>& initializer);

    PointsSet(const std::vector<Point2LL>& points);

    PointsSet(std::vector<Point2LL>&& points);

    /*PointsSet& operator=(const PointsSet& other)
    {
        std::vector<point_t>::operator=(other);
        return *this;
    }*/

    const std::vector<Point2LL>& asRawVector() const
    {
        return *reinterpret_cast<const std::vector<Point2LL>*>(this);
    }

    std::vector<Point2LL>& asRawVector()
    {
        return *reinterpret_cast<std::vector<Point2LL>*>(this);
    }

    Point2LL min() const;

    Point2LL max() const;

    Point2LL closestPointTo(const Point2LL& p) const;

    /*!
     * Translate the whole polygon in some direction.
     *
     * \param translation The direction in which to move the polygon
     */
    void translate(const Point2LL& translation);

    /*!
     * Apply a matrix to each vertex in this set
     */
    void applyMatrix(const PointMatrix& matrix);
    void applyMatrix(const Point3Matrix& matrix);
};

} // namespace cura

#endif // GEOMETRY_POINTS_SET_H

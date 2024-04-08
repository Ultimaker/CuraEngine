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

/*!
 * \brief Base class for all geometry containers representing a set of points.
 */
class PointsSet
{
private:
    std::vector<Point2LL> points_;

public:
    PointsSet() = default;

    PointsSet(const PointsSet& other) = default;

    PointsSet(PointsSet&& other) = default;

    PointsSet(const std::initializer_list<Point2LL>& initializer);

    PointsSet(const std::vector<Point2LL>& points);

    PointsSet(std::vector<Point2LL>&& points);

    /*PointsSet& operator=(const PointsSet& other)
    {
        std::vector<point_t>::operator=(other);
        return *this;
    }*/

    const std::vector<Point2LL>& getPoints() const
    {
        return points_;
    }

    std::vector<Point2LL>& getPoints()
    {
        return points_;
    }

    size_t size() const
    {
        return points_.size();
    }

    void push_back(const Point2LL& point)
    {
        points_.push_back(point);
    }

    void pop_back()
    {
        points_.pop_back();
    }

    std::vector<Point2LL>::const_iterator begin() const
    {
        return points_.begin();
    }

    std::vector<Point2LL>::iterator begin()
    {
        return points_.begin();
    }

    std::vector<Point2LL>::const_iterator end() const
    {
        return points_.end();
    }

    std::vector<Point2LL>::iterator end()
    {
        return points_.end();
    }

    const Point2LL& front() const
    {
        return points_.front();
    }

    Point2LL& front()
    {
        return points_.front();
    }

    const Point2LL& back() const
    {
        return points_.back();
    }

    Point2LL& back()
    {
        return points_.back();
    }

    bool empty() const
    {
        return points_.empty();
    }

    void resize(size_t size)
    {
        points_.resize(size);
    }

    Point2LL& operator[](size_t index)
    {
        return points_[index];
    }

    const Point2LL& operator[](size_t index) const
    {
        return points_[index];
    }

    PointsSet& operator=(const PointsSet& other) = default;

    PointsSet& operator=(PointsSet&& other) = default;

#warning seems to be unused
    Point2LL min() const;

#warning seems to be unused
    Point2LL max() const;

#warning seems to be unused
    Point2LL closestPointTo(const Point2LL& p) const;

    /*!
     * Translate all the points in some direction.
     *
     * \param translation The direction in which to move the points
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

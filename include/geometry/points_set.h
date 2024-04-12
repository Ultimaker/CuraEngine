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

/*!
 * \brief Base class for all geometry containers representing a set of points.
 */
class PointsSet
{
private:
    ClipperLib::Path points_;

public:
    // Requires for some std calls as a container
    typedef Point2LL value_type;

public:
    PointsSet() = default;

    PointsSet(const PointsSet& other) = default;

    PointsSet(PointsSet&& other) = default;

    PointsSet(const std::initializer_list<Point2LL>& initializer);

    PointsSet(const ClipperLib::Path& points);

    PointsSet(ClipperLib::Path&& points);

    const ClipperLib::Path& getPoints() const
    {
        return points_;
    }

    ClipperLib::Path& getPoints()
    {
        return points_;
    }

    void setPoints(ClipperLib::Path&& points)
    {
        points_ = points;
    }

    size_t size() const
    {
        return points_.size();
    }

    void push_back(const Point2LL& point)
    {
        points_.push_back(point);
    }

    template<typename... Args>
    void emplace_back(Args&&... args)
    {
        points_.emplace_back(args...);
    }

    void pop_back()
    {
        points_.pop_back();
    }

    template<typename... Args>
    void insert(Args&&... args)
    {
        points_.insert(args...);
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

    std::vector<Point2LL>::const_reverse_iterator rbegin() const
    {
        return points_.rbegin();
    }

    std::vector<Point2LL>::reverse_iterator rbegin()
    {
        return points_.rbegin();
    }

    std::vector<Point2LL>::const_reverse_iterator rend() const
    {
        return points_.rend();
    }

    std::vector<Point2LL>::reverse_iterator rend()
    {
        return points_.rend();
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

    const Point2LL& at(size_t pos) const
    {
        return points_.at(pos);
    }

    Point2LL& at(size_t pos)
    {
        return points_.at(pos);
    }

    bool empty() const
    {
        return points_.empty();
    }

    void resize(size_t size)
    {
        points_.resize(size);
    }

    void reserve(size_t size)
    {
        points_.reserve(size);
    }

    void clear()
    {
        points_.clear();
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

    /*!
     * \brief Display operator, useful for debugging/testing
     */
    template<class CharT, class TraitsT>
    friend std::basic_ostream<CharT, TraitsT>& operator<<(std::basic_ostream<CharT, TraitsT>& os, const PointsSet& polygon)
    {
        return os << "PointsSet(" << polygon.getPoints() << ")";
    }
};

} // namespace cura

#endif // GEOMETRY_POINTS_SET_H

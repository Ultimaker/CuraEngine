// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef GEOMETRY_POINTS_SET_H
#define GEOMETRY_POINTS_SET_H

#include "geometry/Point2LL.h"
#include "utils/Coord_t.h"

namespace cura
{

class PointMatrix;
class Point3Matrix;

const static int clipper_init = (0);
#define NO_INDEX (std::numeric_limits<size_t>::max())

/*!
 * \brief Base class for all geometry containers representing a set of points.
 * \sa https://github.com/Ultimaker/CuraEngine/wiki/Geometric-Base-Types#pointsset
 */
class PointsSet
{
private:
    ClipperLib::Path points_;

public:
    // Required for some std calls as a container
    using value_type = Point2LL;
    using iterator = typename std::vector<value_type>::iterator;
    using const_iterator = typename std::vector<value_type>::const_iterator;
    using reverse_iterator = typename std::vector<value_type>::reverse_iterator;
    using const_reverse_iterator = typename std::vector<value_type>::const_reverse_iterator;

    /*! \brief Builds an empty set */
    PointsSet() = default;

    /*! \brief Creates a copy of the given points set */
    PointsSet(const PointsSet& other) = default;

    /*! \brief Constructor that takes ownership of the inner points from the given set */
    PointsSet(PointsSet&& other) = default;

    /*! \brief Constructor with a points initializer list, provided for convenience" */
    PointsSet(const std::initializer_list<Point2LL>& initializer);

    virtual ~PointsSet() = default;

    /*! \brief Constructor with an existing list of points */
    explicit PointsSet(const ClipperLib::Path& points);

    /*! \brief Constructor that takes ownership of the given list of points */
    explicit PointsSet(ClipperLib::Path&& points);

    [[nodiscard]] const ClipperLib::Path& getPoints() const
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

    [[nodiscard]] size_t size() const
    {
        return points_.size();
    }

    void push_back(const Point2LL& point)
    {
        points_.push_back(point);
    }

    /*! \brief Pushes an entire set at the end */
    void push_back(const PointsSet& other)
    {
        points_.insert(points_.end(), other.points_.begin(), other.points_.end());
    }

    void emplace_back(auto&&... args)
    {
        points_.emplace_back(std::forward<decltype(args)>(args)...);
    }

    void pop_back()
    {
        points_.pop_back();
    }

    void insert(auto&&... args)
    {
        points_.insert(std::forward<decltype(args)>(args)...);
    }

    [[nodiscard]] const_iterator begin() const
    {
        return points_.begin();
    }

    iterator begin()
    {
        return points_.begin();
    }

    [[nodiscard]] const_iterator end() const
    {
        return points_.end();
    }

    iterator end()
    {
        return points_.end();
    }

    [[nodiscard]] const_reverse_iterator rbegin() const
    {
        return points_.rbegin();
    }

    reverse_iterator rbegin()
    {
        return points_.rbegin();
    }

    [[nodiscard]] const_reverse_iterator rend() const
    {
        return points_.rend();
    }

    reverse_iterator rend()
    {
        return points_.rend();
    }

    [[nodiscard]] const Point2LL& front() const
    {
        return points_.front();
    }

    Point2LL& front()
    {
        return points_.front();
    }

    [[nodiscard]] const Point2LL& back() const
    {
        return points_.back();
    }

    Point2LL& back()
    {
        return points_.back();
    }

    [[nodiscard]] const Point2LL& at(const size_t pos) const
    {
        return points_.at(pos);
    }

    Point2LL& at(const size_t pos)
    {
        return points_.at(pos);
    }

    [[nodiscard]] bool empty() const
    {
        return points_.empty();
    }

    void resize(const size_t size)
    {
        points_.resize(size);
    }

    void reserve(const size_t size)
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
     * \brief Translate all the points in some direction.
     * \param translation The direction in which to move the points
     */
    void translate(const Point2LL& translation);

    /*! \brief Apply a matrix to each vertex in this set */
    void applyMatrix(const PointMatrix& matrix);
    void applyMatrix(const Point3Matrix& matrix);

    /*! \brief Display operator, useful for debugging/testing */
    template<class CharT, class TraitsT>
    friend std::basic_ostream<CharT, TraitsT>& operator<<(std::basic_ostream<CharT, TraitsT>& os, const PointsSet& polygon)
    {
        return os << "PointsSet(" << polygon.getPoints() << ")";
    }
};

} // namespace cura

#endif // GEOMETRY_POINTS_SET_H

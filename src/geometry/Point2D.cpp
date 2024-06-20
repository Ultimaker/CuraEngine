// Copyright (c) 2024 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#include "geometry/Point2D.h"

#include <cmath>

namespace cura
{

Point2D::Point2D(double x, double y)
    : x_(x)
    , y_(y)
{
}

Point2D::Point2D(const Point2LL& other)
    : x_(static_cast<double>(other.X))
    , y_(static_cast<double>(other.Y))
{
}

Point2D Point2D::operator*(const coord_t scale) const
{
    return Point2D(x_ * static_cast<double>(scale), y_ * static_cast<double>(scale));
}

Point2D Point2D::operator*(const double scale) const
{
    return Point2D(x_ * scale, y_ * scale);
}

Point2D Point2D::operator/(const double scale) const
{
    return Point2D(x_ / scale, y_ / scale);
}

double Point2D::size() const
{
    return std::sqrt(size2());
}

double Point2D::size2() const
{
    return x_ * x_ + y_ * y_;
}

Point2D Point2D::normalized() const
{
    return (*this) / size();
}

void Point2D::normalize()
{
    double actual_size = size();
    x_ /= actual_size;
    y_ /= actual_size;
}

Point2LL Point2D::toPoint2LL() const
{
    return Point2LL(std::llrint(x_), std::llrint(y_));
}

} // namespace cura

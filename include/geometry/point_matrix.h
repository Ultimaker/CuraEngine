// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef GEOMETRY_POINT_MATRIX_H
#define GEOMETRY_POINT_MATRIX_H

#include "point2ll.h"


namespace cura
{

class PointMatrix
{
public:
    double matrix[4];

    PointMatrix()
    {
        matrix[0] = 1;
        matrix[1] = 0;
        matrix[2] = 0;
        matrix[3] = 1;
    }

    PointMatrix(double rotation)
    {
        rotation = rotation / 180 * std::numbers::pi;
        matrix[0] = cos(rotation);
        matrix[1] = -sin(rotation);
        matrix[2] = -matrix[1];
        matrix[3] = matrix[0];
    }

    PointMatrix(const Point2LL p)
    {
        matrix[0] = static_cast<double>(p.X);
        matrix[1] = static_cast<double>(p.Y);
        double f = sqrt((matrix[0] * matrix[0]) + (matrix[1] * matrix[1]));
        matrix[0] /= f;
        matrix[1] /= f;
        matrix[2] = -matrix[1];
        matrix[3] = matrix[0];
    }

    static PointMatrix scale(double s)
    {
        PointMatrix ret;
        ret.matrix[0] = s;
        ret.matrix[3] = s;
        return ret;
    }

    Point2LL apply(const Point2LL p) const
    {
        const double x = static_cast<double>(p.X);
        const double y = static_cast<double>(p.Y);
        return Point2LL(std::llrint(x * matrix[0] + y * matrix[1]), std::llrint(x * matrix[2] + y * matrix[3]));
    }

    /*!
     * \warning only works on a rotation matrix! Output is incorrect for other types of matrix
     */
    Point2LL unapply(const Point2LL p) const
    {
        const double x = static_cast<double>(p.X);
        const double y = static_cast<double>(p.Y);
        return Point2LL(std::llrint(x * matrix[0] + y * matrix[2]), std::llrint(x * matrix[1] + y * matrix[3]));
    }

    PointMatrix inverse() const
    {
        PointMatrix ret;
        double det = matrix[0] * matrix[3] - matrix[1] * matrix[2];
        ret.matrix[0] = matrix[3] / det;
        ret.matrix[1] = -matrix[1] / det;
        ret.matrix[2] = -matrix[2] / det;
        ret.matrix[3] = matrix[0] / det;
        return ret;
    }
};

} // namespace cura
#endif // GEOMETRY_POINT_MATRIX_H

// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef GEOMETRY_POINT3_MATRIX_H
#define GEOMETRY_POINT3_MATRIX_H

#include <array>

#include "geometry/Point3LL.h"
#include "geometry/PointMatrix.h"

namespace cura
{

class Point3Matrix
{
public:
    std::array<double, 9> matrix{ 1, 0, 0, 0, 1, 0, 0, 0, 1 };

    Point3Matrix() noexcept = default;

    /*!
     * Initializes the top left corner with the values of \p b
     * and the rest as if it's a unit matrix
     */
    explicit Point3Matrix(const PointMatrix& b)
    {
        matrix.at(0) = b.matrix.at(0);
        matrix.at(1) = b.matrix.at(1);
        matrix.at(2) = 0;
        matrix.at(3) = b.matrix.at(2);
        matrix.at(4) = b.matrix.at(3);
        matrix.at(5) = 0;
        matrix.at(6) = 0;
        matrix.at(7) = 0;
        matrix.at(8) = 1;
    }

    Point3Matrix(Point3Matrix&& point3_matrix) = default;
    Point3Matrix(const Point3Matrix& point3_matrix) = default;
    virtual ~Point3Matrix() = default;

    Point3Matrix& operator=(Point3Matrix&& point3_matrix) = default;
    Point3Matrix& operator=(const Point3Matrix& point3_matrix) = default;

    [[nodiscard]] Point3LL apply(const Point3LL& p) const
    {
        const auto x = static_cast<double>(p.x_);
        const auto y = static_cast<double>(p.y_);
        const auto z = static_cast<double>(p.z_);
        return { std::llrint(x * matrix.at(0) + y * matrix.at(1) + z * matrix.at(2)),
                 std::llrint(x * matrix.at(3) + y * matrix.at(4) + z * matrix.at(5)),
                 std::llrint(x * matrix.at(6) + y * matrix.at(7) + z * matrix.at(8)) };
    }

    /*!
     * Apply matrix to vector as homogeneous coordinates.
     */
    [[nodiscard]] Point2LL apply(const Point2LL& p) const
    {
        Point3LL result = apply(Point3LL(p.X, p.Y, 1));
        return { result.x_ / result.z_, result.y_ / result.z_ };
    }

    static Point3Matrix translate(const Point2LL& p)
    {
        Point3Matrix ret; // uniform matrix
        ret.matrix.at(2) = static_cast<double>(p.X);
        ret.matrix.at(5) = static_cast<double>(p.Y);
        return ret;
    }

    [[nodiscard]] Point3Matrix compose(const Point3Matrix& b) const
    {
        Point3Matrix ret;
        for (int outx = 0; outx < 3; outx++)
        {
            for (int outy = 0; outy < 3; outy++)
            {
                ret.matrix.at(outy * 3 + outx) = 0;
                for (int in = 0; in < 3; in++)
                {
                    ret.matrix.at(outy * 3 + outx) += matrix.at(outy * 3 + in) * b.matrix.at(in * 3 + outx);
                }
            }
        }
        return ret;
    }
};

} // namespace cura
#endif // GEOMETRY_POINT3_MATRIX_H

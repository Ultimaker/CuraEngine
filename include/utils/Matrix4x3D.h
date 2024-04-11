// Copyright (c) 2020 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef MATRIX4X3D_H
#define MATRIX4X3D_H

#include "settings/types/Ratio.h"

namespace cura
{

class Point3D;
class Point3LL;

/*!
 * A 4x3 affine transformation matrix.
 *
 * This matrix behaves as if it's a 4x4 transformation matrix, but the bottom
 * row is always identity.
 */
class Matrix4x3D
{
public:
    /*!
     * Create a scaling matrix with a uniform scale.
     * \param scale The scale factor that this matrix should apply.
     * \param origin The coordinate origin to apply the scale from. If the scale
     * is reduced, all coordinates will go towards this origin. If the scale is
     * increased, all coordinates will go away from this origin.
     */
    static Matrix4x3D scale(const Ratio scale, const Point3LL origin);
    static Matrix4x3D scale(const Ratio scale_x, const Ratio scale_y, const Ratio scale_z, const Point3LL origin);

    /*!
     * The matrix data, row-endian.
     *
     * The first index is the column. The second index is the row.
     */
    double m[4][3];

    /*!
     * Construct an identity matrix.
     */
    Matrix4x3D();

    /*!
     * Apply this transformation to a coordinate.
     *
     * The result will also be converted to an integer-based coordinate
     * (``Point3``).
     * \param p The coordinate to transform.
     * \return A transformed coordinate.
     */
    Point3LL apply(const Point3D& p) const;

    /*!
     * Apply this transformation to a coordinate.
     * \param p The coordinate to transform.
     * \return A transformed coordinate.
     */
    Point3LL apply(const Point3LL& p) const;
};

} // namespace cura
#endif // MATRIX4X3D_H

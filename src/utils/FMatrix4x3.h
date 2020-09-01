//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef FMATRIX3X3_H
#define FMATRIX3X3_H

namespace cura
{

class FPoint3;
class Point3;
class Ratio;

/*!
 * A 4x3 affine transformation matrix.
 *
 * This matrix behaves as if it's a 4x4 transformation matrix, but the bottom
 * row is always identity.
 */
class FMatrix4x3
{
public:
	/*!
	 * Create a scaling matrix with a uniform scale.
	 * \param scale The scale factor that this matrix should apply.
	 */
	static FMatrix4x3 scale(const Ratio scale);

	/*!
	 * The matrix data, row-endian.
	 * 
	 * The first index is the column. The second index is the row.
	 */
    double m[4][3];

	/*!
	 * Construct an identity matrix.
	 */
    FMatrix4x3();

	/*!
	 * Apply this transformation to a coordinate.
	 *
	 * The result will also be converted to an integer-based coordinate
	 * (``Point3``).
	 * \param p The coordinate to transform.
	 * \return A transformed coordinate.
	 */
    Point3 apply(const FPoint3& p) const;
};

} //namespace cura
#endif //FMATRIX3X3_H
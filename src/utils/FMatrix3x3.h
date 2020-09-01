//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef FMATRIX3X3_H
#define FMATRIX3X3_H

namespace cura
{

class Point3;
class FPoint3;

/*!
 * A 3x3 linear transformation matrix.
 */
class FMatrix3x3
{
public:
	/*!
	 * The matrix data, row-endian.
	 * 
	 * The first index is the column. The second index is the row.
	 */
    double m[3][3];

	/*!
	 * Construct an identity matrix.
	 */
    FMatrix3x3();

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
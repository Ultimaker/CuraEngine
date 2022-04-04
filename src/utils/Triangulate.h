//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TRIANGULATE_H
#define TRIANGULATE_H

#include <vector>

#include "IntPoint.h"
#include "polygon.h"

namespace cura
{

/*!
 * Helper class that performs polygon triangulations.
 */
class Triangulate
{
public:
    /*!
     * Helper function to triangulate polygons.
     *
     * This breaks up a shape into a list of triangles that sum up to that shape.
     * \param polygons The shape to triangulate.
     * \return A list of vertices, forming a triangle-list representation of the
     * triangles that form the shape. Each set of 3 consecutive points is a
     * triangle.
     */
    static std::vector<Point> triangulate(const Polygons& polygons);

private:
    /*!
     * Split a shape into Y-monotone pieces.
     * \param polygons The shape to split into monotone pieces.
     * \return A set of Y-monotone polygons that sum up to form the input shape.
     */
    static std::vector<Polygon> splitYMonotone(const Polygons& polygons);

    /*!
     * Triangulate a Y-monotone polygon and add the triangles to the result.
     * \param monotone_polygon A Y-monotone polygon to be triangulated.
     * \param result A triangle-list, where every 3 points forms one triangle,
     * where the resulting triangles must be added. They will be appended to the
     * end.
     */
    static void addTriangles(const Polygon& monotone_polygon, std::vector<Point>& result);
};

}

#endif //TRIANGULATE_H

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
};

}

#endif //TRIANGULATE_H

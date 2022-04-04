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
     * Vertices are categorized into different types, which helps to split the
     * shape into monotone parts. These are the types that can be assigned to
     * each of them.
     */
    enum MonotoneVertexType
    {
        REGULAR, //The vertex has a neighbor that is left and a neighbor that is right.
        START, //The vertex is left of both its neighbors. The inside of the shape is right.
        END, //The vertex is right of both its neighbors. The inside of the shape is left.
        SPLIT, //The vertex is left of both its neighbors. The inside of the shape is left.
        MERGE, //The vertex is right of both its neighbors. The inside of the shape is right.
    };

    /*!
     * Pre-process each vertex to categorize it. This determines where a shape
     * can be split into monotone parts.
     */
    static std::vector<std::vector<MonotoneVertexType>> categorize(const Polygons& polygons);

    /*!
     * Split a shape into X-monotone pieces.
     * \param polygons The shape to split into monotone pieces.
     * \return A set of X-monotone polygons that sum up to form the input shape.
     */
    static std::vector<Polygon> splitXMonotone(const Polygons& polygons);

    /*!
     * Triangulate an X-monotone polygon and add the triangles to the result.
     * \param monotone_polygon An X-monotone polygon to be triangulated.
     * \param result A triangle-list, where every 3 points forms one triangle,
     * where the resulting triangles must be added. They will be appended to the
     * end.
     */
    static void addTriangles(const Polygon& monotone_polygon, std::vector<Point>& result);
};

}

#endif //TRIANGULATE_H

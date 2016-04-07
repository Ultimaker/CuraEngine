/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef TEXTURED_MESH_H
#define TEXTURED_MESH_H

#include <vector>

#include "mesh.h"
#include "utils/intpoint.h"

namespace cura
{

/*!
 * A mesh with a bitmap texture to it.
 */ 
class TexturedMesh : public Mesh 
{
public:
    /*!
     * Coordinates in texture bitmap 
     */
    struct Coord
    {
        double x, y; //!< Coordinates in texture bitmap 
        // 0 to 1
        Coord(double x, double y) //!< constructor
        : x(x)
        , y(y)
        {}
    };
    
    void addTextureCoord(double x, double y);
    void addFace(int vi0, int vi1, int vi2, int ti1, int ti2, int ti3);
protected:
    std::vector<Coord> texture_coords;
    std::vector<Point> face_texture_indices;
private:
    
};

} // namespace cura

#endif // TEXTURED_MESH_H
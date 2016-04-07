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
    TexturedMesh(SettingsBaseVirtual* sb);
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
    /*!
     * 
     */
    struct FaceTextureCoordIndices
    {
        int i1, i2, i3;
        FaceTextureCoordIndices(int i1, int i2, int i3)
        : i1(i1), i2(i2), i3(i3)
        {}
    };
    void addTextureCoord(double x, double y);
    void addFace(int vi0, int vi1, int vi2, int ti1, int ti2, int ti3);
    using Mesh::addFace;
protected:
    std::vector<Coord> texture_coords;
    std::vector<FaceTextureCoordIndices> face_texture_indices;
private:
    
};

} // namespace cura

#endif // TEXTURED_MESH_H
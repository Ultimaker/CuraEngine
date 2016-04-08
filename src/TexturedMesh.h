/** Copyright (C) 2016 Tim Kuipers - Released under terms of the AGPLv3 License */
#ifndef TEXTURED_MESH_H
#define TEXTURED_MESH_H

#include <vector>
#include <string>

#include "MaterialBase.h"

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
        float x, y; //!< Coordinates in texture bitmap
        // 0 to 1
        Coord(float x, float y) //!< constructor
        : x(x)
        , y(y)
        {}
    };
    /*!
     * Coordinates in a specific texture bitmap 
     */
    struct MatCoord
    {
        Coord coords;
        int mat_id; //!< Material id
        MatCoord(Coord coords, int mat_id)
        : coords(coords)
        , mat_id(mat_id)
        {}
    };
    /*!
     * 
     */
    struct FaceTextureCoordIndices
    {
        int i1, i2, i3; //!< indices into texture_coords or -1 if no texture data available
        int mat_id; //!< Material id
        FaceTextureCoordIndices(int i1, int i2, int i3, int mat_id)
        : i1(i1), i2(i2), i3(i3)
        , mat_id(mat_id)
        {}
    };
    void addTextureCoord(float x, float y);
    void addFace(int vi0, int vi1, int vi2, int ti0, int ti1, int ti2);
    using Mesh::addFace; // otherwise above addFace would shadow the parent addFace

    bool setMaterial(std::string name); //!< set the material to be used in the comming data to be loaded
    Material* addMaterial(std::string name);

    /*!
     * \param face_idx The face for which to get the material coord
     * \param loc The location on the face for which to get the material coord
     * \param result The resulting material Coordinates
     * \return Whether a Material coordinate is defined at the given location
     */
    bool getMatCoord(unsigned int face_idx, const Point3 loc, MatCoord& result);
protected:
    std::vector<Coord> texture_coords;
    std::vector<FaceTextureCoordIndices> face_texture_indices;
    MaterialBase material_base;
private:
    int current_mat; //!< material currently used in loading the face material info
};

} // namespace cura

#endif // TEXTURED_MESH_H